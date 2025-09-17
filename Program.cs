using Sandbox.Game.AI.Navigation;
using Sandbox.Game.EntityComponents;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using SpaceEngineers.Game.ModAPI.Ingame;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Linq;
using System.Security.Cryptography.X509Certificates;
using System.Text;
using VRage;
using VRage.Collections;
using VRage.Game;
using VRage.Game.Components;
using VRage.Game.GUI.TextPanel;
using VRage.Game.ModAPI.Ingame;
using VRage.Game.ModAPI.Ingame.Utilities;
using VRage.Game.ObjectBuilders.Definitions;
using VRageMath;
using VRageRender;
using static VRageMath.Base6Directions;

namespace AerOS
{
    partial class Program : MyGridProgram
    {
        private readonly DependencyContainer _dependencies = new DependencyContainer();

        private FlightController FC;
        private ChannelIO CH;
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
            FC.Update(deltaTime);

            foreach (IMyBroadcastListener _ch in CH.GetSubscriptions())
            {
                FC.SetTargetAttitude((MatrixD)CH.GetMessage(_ch).Data);
            }

            DisplayLogger();
        }

        private void InitializeDependencies()
        {
            // 初始化硬件
            string loc = "Initialization";
            var ctr = GridTerminalSystem.GetBlockWithName("ctr") as IMyShipController;
            if (ctr == null) { logger.Log(loc, "错误: 未找到控制器 'ctr'"); return; }
            var gyros = GetBlocksOfType<IMyGyro>();
            var thrusts = GetBlocksOfType<IMyThrust>();
            var lcds = GetBlocksOfType<IMyTextPanel>();

            // 注册依赖
            _dependencies.Register<IMyShipController>(ctr);
            _dependencies.Register<List<IMyGyro>>(gyros);
            _dependencies.Register<List<IMyThrust>>(thrusts);
            _dependencies.Register<List<IMyTextPanel>>(lcds);

            _dependencies.Register<ITimeProvider>(new TimeProvider());
            _dependencies.Register<FlightControllerConfig>(new FlightControllerConfig { PIDController = new FilteredPIDController(6.5, 0.1, 2.1, 5.5), });
            _dependencies.Register<IMyIntergridCommunicationSystem>(IGC);
            _dependencies.Register<ILogger>(logger);

            // 创建飞行控制器
            FC = new FlightController(_dependencies);
            //  创建通信IO控制器
            CH = new ChannelIO(_dependencies);
        }
        private int logc = 1000;
        private void DisplayLogger()
        {
            if (logc < 20) logc++;
            else
            {
                foreach (IMyBroadcastListener _ch in CH.GetSubscriptions())
                {
                    //logger.Log("MSG", CH.GetMessage(_ch).Data.ToString());
                    logger.Log("MSG", "信息更新");
                }

                foreach (string log in logger.GetLogs())
                {
                    Echo(log);
                }
                logc = 0;
            }
        }
        private List<T> GetBlocksOfType<T>() where T : class
        {
            string loc = "InitializeDevice";
            var blocks = new List<T>();
            GridTerminalSystem.GetBlocksOfType(blocks);
            blocks = blocks.Where(b => (b as IMyTerminalBlock)?.IsFunctional == true).ToList();
            logger.Log(loc, $"找到 {blocks.Count} 个{blocks.FirstOrDefault()}");
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

        // ==================== 接口 ====================
        public interface ITimeProvider
        {
            double DeltaTime { get; set; }
        }

        public interface IPIDController
        {
            Vector3D Update(Vector3D error, double deltaTime);
            void Reset();
        }

        public interface IFlightController
        {
            void Update(double deltaTime);
            void SetTargetAttitude(MatrixD targetAttitude);
        }
        public interface IChannelIO
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
            void Log(string msg_from, string message);
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

            private Vector3D _integral = Vector3D.Zero;
            private Vector3D _previousError = Vector3D.Zero;
            private Vector3D _previousDerivative = Vector3D.Zero;
            private double _derivativeSmoothingFactor = 0.5; // 微分项平滑因子

            public PIDController(double kp, double ki, double kd)
            {
                Kp = kp;
                Ki = ki;
                Kd = kd;
            }

            public Vector3D Update(Vector3D error, double deltaTime)
            {
                if (deltaTime < 1e-10) return Vector3D.Zero;

                // 抗饱和积分项
                _integral += error * deltaTime;

                // 限制积分项大小
                double maxIntegral = 2.0;
                _integral.X = Math.Max(Math.Min(_integral.X, maxIntegral), -maxIntegral);
                _integral.Y = Math.Max(Math.Min(_integral.Y, maxIntegral), -maxIntegral);
                _integral.Z = Math.Max(Math.Min(_integral.Z, maxIntegral), -maxIntegral);

                // 微分项平滑导数计算
                Vector3D derivative;
                if (_previousError != Vector3D.Zero)
                {
                    derivative = (error - _previousError) / deltaTime;
                    // 应用平滑滤波
                    derivative = _previousDerivative * _derivativeSmoothingFactor +
                                derivative * (1 - _derivativeSmoothingFactor);
                }
                else
                {
                    derivative = Vector3D.Zero;
                }

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

        // PID 控制器 带低通滤波
        public class FilteredPIDController : IPIDController
        {
            public double Kp { get; set; }
            public double Ki { get; set; }
            public double Kd { get; set; }

            private Vector3D _integral = Vector3D.Zero;
            private Vector3D _previousError = Vector3D.Zero;
            private LowPassFilter _outputFilter;

            public FilteredPIDController(double kp, double ki, double kd, double filterCutoff = 0.5)
            {
                Kp = kp;
                Ki = ki;
                Kd = kd;
                _outputFilter = new LowPassFilter(filterCutoff);
            }

            public Vector3D Update(Vector3D error, double deltaTime)
            {
                if (deltaTime < 1e-10) return Vector3D.Zero;

                // 积分项
                _integral += error * deltaTime;

                // 限制积分项大小
                double maxIntegral = 2.0;
                _integral.X = Math.Max(Math.Min(_integral.X, maxIntegral), -maxIntegral);
                _integral.Y = Math.Max(Math.Min(_integral.Y, maxIntegral), -maxIntegral);
                _integral.Z = Math.Max(Math.Min(_integral.Z, maxIntegral), -maxIntegral);

                // 微分项
                Vector3D derivative = (_previousError != Vector3D.Zero) ?
                    (error - _previousError) / deltaTime : Vector3D.Zero;
                _previousError = error;

                // PID输出
                Vector3D rawOutput = (Kp * error) + (Ki * _integral) + (Kd * derivative);

                // 应用低通滤波
                return _outputFilter.Apply(rawOutput, deltaTime);
            }

            public void Reset()
            {
                _integral = Vector3D.Zero;
                _previousError = Vector3D.Zero;
                _outputFilter.Reset();
            }

            public void SetFilterCutoff(double cutoffFrequency)
            {
                _outputFilter.SetCutoffFrequency(cutoffFrequency);
            }
            // 低通滤波器
            public class LowPassFilter
            {
                private Vector3D _previousOutput = Vector3D.Zero;
                private double _cutoffFrequency;
                private double _alpha;

                public LowPassFilter(double cutoffFrequency)
                {
                    _cutoffFrequency = cutoffFrequency;
                }

                public Vector3D Apply(Vector3D input, double deltaTime)
                {
                    // 计算滤波系数
                    double rc = 1.0 / (2 * Math.PI * _cutoffFrequency);
                    _alpha = deltaTime / (rc + deltaTime);

                    // 应用一阶低通滤波
                    _previousOutput = _previousOutput * (1 - _alpha) + input * _alpha;
                    return _previousOutput;
                }

                public void Reset()
                {
                    _previousOutput = Vector3D.Zero;
                }

                public void SetCutoffFrequency(double cutoffFrequency)
                {
                    _cutoffFrequency = cutoffFrequency;
                }
            }
        }

        // 飞控
        public class FlightController : IFlightController
        {
            private readonly IMyShipController _controller;
            private readonly List<IMyGyro> _gyros;
            private readonly IPIDController _pidController;
            private Vector3D _previousAngularVelocity = Vector3D.Zero;
            private MatrixD _targetAttitude;
            private readonly ILogger _logger;

            public FlightController(DependencyContainer dependencies)
            {
                _controller = dependencies.Resolve<IMyShipController>();
                _gyros = dependencies.Resolve<List<IMyGyro>>();
                _pidController = dependencies.Resolve<FlightControllerConfig>().PIDController;
                _logger = dependencies.Resolve<ILogger>();

                // 默认保持当前姿态
                _targetAttitude = _controller.WorldMatrix;
            }

            public void Update(double deltaTime)
            {
                if (_controller == null || _gyros == null || _gyros.Count == 0) return;

                // 计算当前姿态与目标姿态之间的误差
                var error = CalculateAttitudeError(_controller.WorldMatrix, _targetAttitude);
                _logger.Log("Update", $"误差: { error.ToString()}");
                // 计算角速度命令
                var desiredAngularVelocity = _pidController.Update(error, deltaTime);

                _previousAngularVelocity = desiredAngularVelocity;
                // 应用
                ApplyAngularVelocity(desiredAngularVelocity);
            }

            public void SetTargetAttitude(MatrixD targetAttitude)
            {
                _targetAttitude = targetAttitude;
                _pidController.Reset(); // 重置PID控制器积分项
                _previousAngularVelocity = Vector3D.Zero; // 重置角速度历史
            }

            private void ApplyAngularVelocity(Vector3D angularVelocity)
            {

                foreach (var gyro in _gyros)
                {
                    if (gyro == null) continue;

                    gyro.GyroOverride = true;

                    // 将全局角速度命令转换为陀螺仪的局部坐标系
                    var localAxis = Vector3D.TransformNormal(
                        angularVelocity,
                        MatrixD.Transpose(gyro.WorldMatrix)
                    );

                    // 设置陀螺仪转速
                    gyro.Pitch = (float)localAxis.X;
                    gyro.Yaw = (float)localAxis.Y;
                    gyro.Roll = (float)localAxis.Z;
                }
            }

            // ==================== 工具函数 ====================
            private static Vector3D CalculateAttitudeError(MatrixD current, MatrixD target)
            {
                // 提取3x3旋转部分
                MatrixD currentRot = ExtractRotation(current);
                MatrixD targetRot = ExtractRotation(target);

                // 计算姿态误差矩阵
                var errorMatrix = MatrixD.Transpose(current) * target;

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

            private static void GetAxisAngleFromMatrix(MatrixD matrix, out Vector3D axis, out double angle)
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

        // 通讯
        public class ChannelIO : IChannelIO
        {
            private readonly IMyIntergridCommunicationSystem _IGC;
            private readonly List<IMyBroadcastListener> ChannelListeners = new List<IMyBroadcastListener>();
            private readonly List<IMyBroadcastListener> ChannelHosts = new List<IMyBroadcastListener>();
            private MyIGCMessage news;
            public ChannelIO(DependencyContainer dependencies)
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
            private readonly int max_logs = 50;
            public void Log(string msg_from, string msg) { _logs.Add($"[{msg_from}]: {msg}"); Wrap(); }
            public List<string> GetLogs() { return _logs; }
            public void Wrap() { if (_logs.Count > 2 * max_logs) _logs = _logs.Skip(50).ToList(); }
            public void Purge() { _logs = new List<string>(); }
        }
        // ==================== 数据结构 ====================
        public struct FlightControllerConfig
        {
            public IPIDController PIDController;
        }
        public struct SafeSwappingProtocol
        {
            byte[] pub_key;
            byte[] content;

        }
    }
}
