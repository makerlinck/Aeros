using Sandbox.ModAPI.Ingame;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using VRageMath;
using static AerOS.Program;

namespace AerOS
{
    partial class Program : MyGridProgram
    {
        private readonly DependencyContainer _dependencies = new DependencyContainer();

        private FlightController FC;
        private IGCIO CH;
        private long _lastRunTime;
        private Logger logger;
        public Program()
        {
            Runtime.UpdateFrequency = UpdateFrequency.Update1;
            logger = new Logger();
            InitializeDependencies();
            _lastRunTime = DateTime.Now.Ticks;

            CH.Subscribe("AAAFDA5688772E7B");
        }

        public void Main(string argument, UpdateType updateSource)
        {
            // 计算更新飞控时间差

            var currentTime = DateTime.Now.Ticks;
            var deltaTime = (_lastRunTime > 0) ? (currentTime - _lastRunTime) / (double)TimeSpan.TicksPerSecond : 0.01;
            _lastRunTime = currentTime;

            foreach (IMyBroadcastListener _ch in CH.GetSubscriptions())
            {
                FC.SetTargetAttitude((MatrixD)CH.GetMessage(_ch).Data);
                FC.UpdateAttitude(deltaTime);
            }
            DisplayLogger();
        }

        private void InitializeDependencies()
        {
            // 初始化硬件
            var ctr = GridTerminalSystem.GetBlockWithName("ctr") as IMyShipController;
            if (ctr == null) { logger.Log( "错误: 未找到控制器 'ctr'", 3 ); return; }
            var gyros = GetBlocksOfType<IMyGyro>();
            var thrusts = GetBlocksOfType<IMyThrust>();
            var lcds = GetBlocksOfType<IMyTextPanel>();

            // 注册依赖
            _dependencies.Register<IMyShipController>(ctr);
            _dependencies.Register<List<IMyGyro>>(gyros);
            _dependencies.Register<List<IMyThrust>>(thrusts);
            _dependencies.Register<List<IMyTextPanel>>(lcds);

            _dependencies.Register<ITimeProvider>(new TimeProvider());
            _dependencies.Register<FlightControllerConfig>(new FlightControllerConfig
            {
                PIDController = new PIDController(22.3, 0.002, 0.15),
                MaxAngularVelocity = 70,
                BrakeThreshold = 67,
                PredictiveBrakeFactor = 0.18
            });
            _dependencies.Register<IMyIntergridCommunicationSystem>(IGC);
            _dependencies.Register<ILogger>(logger);

            // 创建飞行控制器
            FC = new FlightController(_dependencies);
            //  创建通信IO控制器
            CH = new IGCIO(_dependencies, "");
        }
        private int logc = 1000;
        private void DisplayLogger()
        {
            if (logc < 20) logc++;
            else
            {
                foreach (string log in logger.GetLogs())
                {
                    Echo(log);
                }
                logc = 0;
            }
        }
        private List<T> GetBlocksOfType<T>() where T : class
        {
            var blocks = new List<T>();
            GridTerminalSystem.GetBlocksOfType(blocks);
            blocks = blocks.Where(b => (b as IMyTerminalBlock)?.IsFunctional == true).ToList();
            logger.Log($"找到 {blocks.Count} 个{blocks.FirstOrDefault()}", 1 );
            return blocks;
        }

        // ==================== 依赖注入容器 ====================
        public class DependencyContainer
        {
            private readonly Dictionary<Type, object> _registrations = new Dictionary<Type, object>();

            public void Register<T>(T instance)
            {
                _registrations[typeof(T)] = instance;
            }

            public T Resolve<T>()
            {
                return (T)_registrations[typeof(T)];
            }
        }

        // ==================== 接口类 ====================
        public interface ITimeProvider
        {
            double DeltaTime { get; set; }
        }

        public interface IPIDController
        {
            Vector3D Update(Vector3D error, Vector3D currentAngularVelocity, double deltaTime);
            void Reset();
        }

        public interface IFlightController
        {
            void UpdateAttitude(double deltaTime);
            void SetTargetAttitude(MatrixD targetAttitude);
        }
        public interface IIGCIO
        {
            void Subscribe(string channel_id);
            void Unsubscribe(IMyBroadcastListener listener);

            void CreateChannel(string channel_id);
            void DropChannel(IMyBroadcastListener listener);

            List<IMyBroadcastListener> GetChannels();
            List<IMyBroadcastListener> GetSubscriptions();

            void SendMessage<T>(string host, T msg);
            MyIGCMessage GetMessage(IMyBroadcastListener listener);
        }
        public interface IScreenController
        {
            void InitializeUI();
            void Write();
            void WriteLine();
            void Update();
            void Clear();
        }
        public interface ILogger
        {
            void Log( string msg, short level = 0);
            void Wrap();
        }

        // ==================== 实现类 ====================
        // 获取时间信息
        public class TimeProvider : ITimeProvider
        {
            public double DeltaTime { get; set; }
        }

        // PID 控制器
        public class PIDController : IPIDController
        {
            public double Kp { get; set; }
            public double Ki { get; set; }
            public double Kd { get; set; }

            private Vector3D _integral = Vector3D.Zero, _previousError = Vector3D.Zero, _previousDerivative = Vector3D.Zero;

            public PIDController(double kp, double ki, double kd)
            {
                Kp = kp;
                Ki = ki;
                Kd = kd;
            }

            public Vector3D Update(Vector3D error, Vector3D currentAngularVelocity, double deltaTime)
            {
                if (deltaTime < 1e-10) return Vector3D.Zero;

                // 积分项
                _integral += error * deltaTime;

                // 限制积分项大小
                double maxIntegral = 2.0;
                _integral.X = Math.Max(Math.Min(_integral.X, maxIntegral), -maxIntegral);
                _integral.Y = Math.Max(Math.Min(_integral.Y, maxIntegral), -maxIntegral);
                _integral.Z = Math.Max(Math.Min(_integral.Z, maxIntegral), -maxIntegral);

                // 微分项平滑导数计算
                Vector3D derivative = (error - _previousError) / deltaTime;

                _previousError = error;
                _previousDerivative = derivative;

                // PID输出
                Vector3D output = (Kp * error) + (Ki * _integral) + (Kd * derivative);

                return output;
            }

            public void Reset()
            {
                _integral = Vector3D.Zero;
                _previousError = Vector3D.Zero;
                _previousDerivative = Vector3D.Zero;
            }
        }

        public class FlightController : IFlightController
        {
            private readonly IMyShipController _controller;
            private readonly List<IMyGyro> _gyros;
            private readonly List<IMyThrust> _thrusts;
            private readonly IPIDController _pidController;
            private Vector3D _previousAngularVelocity = Vector3D.Zero;
            private MatrixD _targetAttitude;

            private double _maxAngularVelocity; // 最大角速度
            private double _brakeThreshold; // 制动阈值
            private double _predictiveBrakeFactor; // 预测制动因子

            private readonly ILogger _logger;

            public FlightController(DependencyContainer dependencies)
            {
                _controller = dependencies.Resolve<IMyShipController>();
                _gyros = dependencies.Resolve<List<IMyGyro>>();
                _thrusts = dependencies.Resolve<List<IMyThrust>>();

                // 获取配置参数
                var config = dependencies.Resolve<FlightControllerConfig>();
                _pidController = config.PIDController;
                _maxAngularVelocity = config.MaxAngularVelocity;
                _brakeThreshold = config.BrakeThreshold;
                _predictiveBrakeFactor = config.PredictiveBrakeFactor;

                _logger = dependencies.Resolve<ILogger>();

                // 默认保持当前姿态
                _targetAttitude = _controller.WorldMatrix;
            }

            public void UpdateAttitude(double deltaTime)
            {
                // 获取当前角速度
                var currentAngularVelocity = _controller.GetShipVelocities().AngularVelocity;
                // 计算当前姿态与目标姿态之间的误差
                var error = CalculateAttitudeError(_controller.WorldMatrix, _targetAttitude);
                var errorMagnitude = error.Length();
                // 预测制动点
                var predictedStopDistance = PredictStopAngle(currentAngularVelocity);
                // 控制逻辑
                Vector3D desiredAngularVelocity;

                if (errorMagnitude > _brakeThreshold)
                {
                    // 大误差区域：使用最大角速度
                    desiredAngularVelocity = Vector3D.Normalize(error) * _maxAngularVelocity;
                }
                else if (errorMagnitude > predictedStopDistance * _predictiveBrakeFactor)
                {
                    // 预测制动区域：开始减速
                    var brakeIntensity = errorMagnitude / (_brakeThreshold * _predictiveBrakeFactor);
                    desiredAngularVelocity = Vector3D.Normalize(error) * _maxAngularVelocity * brakeIntensity;
                }
                else
                {
                    // 制动区域：反向最大角速度
                    desiredAngularVelocity = -Vector3D.Normalize(currentAngularVelocity) * _maxAngularVelocity;

                    // 如果角速度已经很小，则完全停止
                    if (currentAngularVelocity.Length() < 0.02)
                    {
                        desiredAngularVelocity = Vector3D.Zero;
                    }
                }
                // 应用指令
                ApplyAngularVelocity(desiredAngularVelocity);
            }

            public void SetTargetAttitude(MatrixD targetAttitude)
            {
                _targetAttitude = targetAttitude;
                _pidController.Reset(); // 重置PID控制器积分项
                _previousAngularVelocity = Vector3D.Zero; // 重置角速度历史
            }

            // 预测停止角度（基于当前角速度）
            private double PredictStopAngle(Vector3D currentAngularVelocity)
            {
                double maxAngularAcceleration = _maxAngularVelocity * 2; // 根据实际情况调整
                return (currentAngularVelocity.LengthSquared()) / (2 * maxAngularAcceleration);
            }

            // 应用推进器，返回应用的总推力
            private float ApplyThrust(float percentOfThrust)
            {
                float currentThrust = 0;
                foreach (var thrust in _thrusts)
                {
                    thrust.ThrustOverridePercentage = percentOfThrust;
                    currentThrust += thrust.CurrentThrust;
                }
                return currentThrust;
            }

            private void ApplyAngularVelocity(Vector3D angularVelocity)
            {
                if (angularVelocity.Length() > _maxAngularVelocity)
                {
                    angularVelocity = Vector3D.Normalize(angularVelocity) * _maxAngularVelocity;
                }

                // 陀螺仪启用
                foreach (var gyro in _gyros)
                {
                    gyro.GyroOverride = true;

                    // 将全局角速度命令转换为陀螺仪的局部坐标系
                    var localAxis = Vector3D.TransformNormal(
                        -angularVelocity,
                        MatrixD.Transpose(gyro.WorldMatrix)
                    );

                    // 设置陀螺仪转速
                    gyro.Pitch = (float)localAxis.X;
                    gyro.Yaw = (float)localAxis.Y;
                    gyro.Roll = (float)localAxis.Z;
                }
            }
            // ==================== 工具函数 ====================
            private Vector3D CalculateAttitudeError(MatrixD current, MatrixD target)
            {
                // 提取3x3旋转部分
                MatrixD currentRot = ExtractRotation(current);
                MatrixD targetRot = ExtractRotation(target);
                // 计算姿态误差矩阵
                var errorMatrix = MatrixD.Transpose(currentRot) * targetRot;
                // 从误差矩阵中提取旋转轴和角度
                double angle;
                Vector3D axis;
                GetAxisAngleFromMatrix(errorMatrix, out axis, out angle);
                // 将轴角转换为误差向量
                return axis * angle;
            }

            private static MatrixD ExtractRotation(MatrixD matrix)
            {
                // 提取3x3旋转部分
                return new MatrixD(
                    matrix.M11, matrix.M12, matrix.M13, 0,
                    matrix.M21, matrix.M22, matrix.M23, 0,
                    matrix.M31, matrix.M32, matrix.M33, 0,
                    0, 0, 0, 1
                );
            }
            private void GetAxisAngleFromMatrix(MatrixD matrix, out Vector3D axis, out double angle)
            {
                // 计算旋转角度
                double trace = matrix.M11 + matrix.M22 + matrix.M33;
                trace = Math.Max(Math.Min(trace, 3.0), -1.0);
                angle = Math.Acos((trace - 1) / 2);
                // 避免除以零
                if (angle < 1e-10)
                {
                    axis = Vector3D.Forward;
                    return;
                }

                // 计算旋转轴
                axis = new Vector3D(
                    matrix.M23 - matrix.M32,
                    matrix.M31 - matrix.M13,
                    matrix.M12 - matrix.M21
                );
                axis = Vector3D.Normalize(axis);
            }
        }

        public class VehicleController : FlightController
        {
            private enum ControllingType
            {
                SpaceJet,   // 太空舰艇
                Spaceship,  // 太空飞船
                VTOL,   // 垂直起降多用途飞行器
                Auxiliary,  // 辅助驾驶
            }
            public VehicleController(DependencyContainer dependencies) : base(dependencies)
            {
            }
        }
        public class MissileController : FlightController
        {
            private enum ControllingType
            {
                BeamRider,  // 激光制导
                Inertial,   // 惯性制导
                ActiveHoming,   // 主动寻的
                SemiHoming, // 半主动寻的
                PassiveHoming,  // 被动寻的
            }
            public MissileController(DependencyContainer dependencies) : base(dependencies)
            {
            }
        }
        // 通讯
        public class IGCIO : IIGCIO
        {
            private readonly IMyIntergridCommunicationSystem _IGC;
            private readonly List<IMyBroadcastListener> ChannelListeners = new List<IMyBroadcastListener>();
            private readonly List<IMyBroadcastListener> ChannelHosts = new List<IMyBroadcastListener>();
            private MyIGCMessage news;
            public IGCIO(DependencyContainer dependencies, string pri_key = "")
            {
                news = new MyIGCMessage();
                _IGC = dependencies.Resolve<IMyIntergridCommunicationSystem>();
            }

            public void CreateChannel(string channel_id)
            {
            }

            public void DropChannel(IMyBroadcastListener listener)
            {
            }

            public List<IMyBroadcastListener> GetChannels()
            {
                return ChannelHosts;
            }
            public List<IMyBroadcastListener> GetSubscriptions()
            {
                return ChannelListeners;
            }
            public void SendMessage<T>(string host, T msg)
            {
                _IGC.SendBroadcastMessage(host, msg);
            }
            public MyIGCMessage GetMessage(IMyBroadcastListener listener)
            {

                if (listener.HasPendingMessage) news = listener.AcceptMessage();
                return news;
            }

            public void Subscribe(string channel_id)
            {
                IMyBroadcastListener ch = _IGC.RegisterBroadcastListener(channel_id);
                ChannelListeners.Add(ch);
                ch.SetMessageCallback();
            }

            public void Unsubscribe(IMyBroadcastListener listener) { ChannelListeners.Remove(listener); }
        }

        // 显示器控制器
        public class ScreenController : IScreenController
        {
            public ScreenController()
            {
            }
            public void InitializeUI()
            {
            }

            public void Update()
            {
            }

            public void Write()
            {
            }

            public void WriteLine()
            {
            }
            public void Clear()
            {
            }
            // 图形库

            public static void Cube()
            {

            }
        }
        // 日志工具方法
        private class Logger : ILogger
        {
            private List<string> _logs = new List<string>();
            private readonly int max_logs = 16;
            private List<string> levels = new List<string> { "debug", "info", "warn", "error"};
            public void Log(string msg, short level = 0) { _logs.Add($"[{levels[level]}]: { msg }"); Wrap(); }
            public List<string> GetLogs() { return _logs; }
            public void Wrap() { if (_logs.Count > 2 * max_logs) _logs = _logs.Skip(50).ToList(); }
            public void Purge() { _logs = new List<string>(); }
        }
        // ==================== 数据结构 ====================
        // 飞控配置
        public struct FlightControllerConfig
        {
            int controllingType;
            public IPIDController PIDController;
            public double MaxAngularVelocity { get; set; }
            public double BrakeThreshold { get; set; }
            public double PredictiveBrakeFactor { get; set; }
        }
        // 安全交换协议
        //public struct SafeSwappingProtocol
        //{
        //    byte[] pub_key;
        //    byte[] content;

        //}
    }
}
