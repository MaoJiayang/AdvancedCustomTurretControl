using Sandbox.ModAPI.Ingame;
using System;
using System.Collections.Generic;
using System.Linq;
using VRageMath;
using System.Text;
using VRage.Game;
// TODO: 防友伤系统，俯仰归位系统，没驾驶舱时不工作
// TODO：索敌逻辑要考虑多炮塔多目标的选择，是否位于我的可达范围内
// TODO：轮射耗时优化
namespace IngameScript
{
    public class Program : MyGridProgram
    {
        #region 配置

        // 武器与转子配置
        private const string 自定义炮塔编组前缀 = "自定义炮塔"; // 编组前缀
        private const string 方向机名称 = "方向机";     // 方位转子名称标签
        private const string 高低机名称 = "高低机";     // 俯仰转子名称标签
        private const string 基线武器标记 = "基线";       // 基线武器标记
        private const double 角度容差 = 3.14 / 180 * 1;            // 角度容差(弧度)
        private const double 转子比例系数 = 30.0;         // 转子PID比例系数
        private const double 转子积分系数 = 0.0;          // 转子PID积分系数
        private const double 转子微分系数 = 0.0;          // 转子PID微分系数
        private const double 转子最大速度 = 6.28;          // 转子最大速度 (rad/s)  
        private const double PID时间间隔 = 1.0 / 60.0;         // PID控制时间间隔


        // 弹道与预测配置
        private const double 默认弹速 = 500.0;           // 默认武器弹速 (m/s)
        private const float 俯仰下限 = -20.0f;           // 俯仰下限（度）
        private const float 俯仰上限 = 75.0f;            // 俯仰上限（度）
        private const int 预测迭代次数 = 5;               // 预测迭代次数
        private const int 默认预测窗口时长 = 750;          // 预测窗口时长（750毫秒）

        // 性能与运行配置
        private const int 默认跳帧数 = 6;                // 默认跳帧数
        private const int 备用跳帧数 = 1;                 // 备选跳帧数
        private const int 方块更新跳帧数 = 3600;          // 更新方块跳帧数

        // 射击控制配置
        private const bool 默认使用轮射 = true;          // 默认是否使用轮射
        private const int 默认轮射间隔 = 6;               // 默认轮射间隔帧数

        #endregion

        #region 字段

        // 转子武器系统
        private IMyMotorStator 方位转子;
        private IMyMotorStator 俯仰转子;
        private IMyTerminalBlock 默认基线武器;                // 基线武器，用于瞄准计算
        private List<IMyUserControllableGun> 武器列表;    // 所有武器列表
        private IMyBlockGroup 炮塔编组;                   // 当前使用的炮塔编组
        private PID 方向机PID控制器; // 方位转子PID控制器
        private PID 高低机PID控制器; // 俯仰转子PID控制器
        private bool 弹药受重力影响 = true; // 是否考虑重力影响


        // 索敌系统
        private List<IMyLargeTurretBase> 索敌炮塔列表;    // 用于索敌的炮塔
        private List<IMyShipController> 驾驶舱列表;      // 控制器列表

        // 目标跟踪系统
        private TargetTracker 目标跟踪器;
        private int 预测窗口时长 = 默认预测窗口时长; // 预测窗口时长（毫秒）
        private MyDetectedEntityInfo 当前目标;
        private bool 存在有效目标 = false;
        private double 目标距离 = 0;
        private Vector3D 目标位置_显示用 = Vector3D.Zero;
        private Vector2D? 角度误差_显示用 = Vector2D.Zero; // 用于存储计算的角度误差
        private Vector3D 缓存预测位置 = Vector3D.Zero;
        // 运行控制
        private int 跳帧计数 = 默认跳帧数;
        private int 运行计时 = 0;
        private double 武器弹速 = 默认弹速;
        private bool 使用轮射 = 默认使用轮射;
        private bool 正在齐射 = false; // 是否正在执行齐射
        private int 轮射间隔 = 默认轮射间隔;
        private Dictionary<int, int> 武器激活时间 = new Dictionary<int, int>(); // 武器索引 -> 激活时间帧
        private int 射击关闭延迟 = 20; // 延迟关闭射击功能的帧数
        private IMyUserControllableGun 当前参考基线武器 = null;
        private int 轮射武器索引 = 0;
        private int 轮射窗口长度 = 0;
        private int 最大射程 = 0;

        // 性能统计
        private double 总运行时间ms = 0;
        private double 最大运行时间ms = 0;
        private int 性能统计运行次数 = 0;
        private int 当前最大迭代次数 = 0;
        private StringBuilder 性能统计 = new StringBuilder();

        #endregion

        #region 初始化

        public Program()
        {
            // 初始化目标跟踪器
            按预测窗口时长设置目标跟踪器(默认预测窗口时长);
            // 初始化武器列表
            武器列表 = new List<IMyUserControllableGun>();
            索敌炮塔列表 = new List<IMyLargeTurretBase>();
            驾驶舱列表 = new List<IMyShipController>();

            方向机PID控制器 = new PID(转子比例系数, 转子积分系数, 转子微分系数, PID时间间隔);
            高低机PID控制器 = new PID(转子比例系数, 转子积分系数, 转子微分系数, PID时间间隔);
            // 配置PID控制器
            方向机PID控制器.SetOutputLimits(-转子最大速度, 转子最大速度);
            高低机PID控制器.SetOutputLimits(-转子最大速度, 转子最大速度);
            方向机PID控制器.SetIntegralLimits(-2.0, 2.0); // 合理的积分限制值
            高低机PID控制器.SetIntegralLimits(-2.0, 2.0);
            方向机PID控制器.SetBackCalculationFactor(0.1); // 抗饱和系数
            高低机PID控制器.SetBackCalculationFactor(0.1);
            // 设置运行频率
            Runtime.UpdateFrequency = UpdateFrequency.Update1;

            // 获取方块
            获取方块();
            Vector4D 武器基础信息 = 获取武器基础信息(默认基线武器 as IMyUserControllableGun, 武器列表.Count);
            轮射间隔 = (int)武器基础信息.X;
            轮射窗口长度 = (int)武器基础信息.Y;
            当前参考基线武器 = 默认基线武器 as IMyUserControllableGun;
            Echo("自定义炮塔控制系统已启动");
            Echo("使用命令：目标 X Y Z 设置目标坐标");
        }

        private void 按预测窗口时长设置目标跟踪器(int 预测窗口时长ms)
        {
            int 历史记录大小 = 预测窗口时长ms / (跳帧计数 * 17);
            历史记录大小 = Math.Max(历史记录大小, 4); // 确保至少有4个历史记录
            目标跟踪器 = new TargetTracker(历史记录大小);
        }

        private void 获取方块()
        {
            // 重置引用
            方位转子 = null;
            俯仰转子 = null;
            默认基线武器 = null;
            炮塔编组 = null;
            武器列表.Clear();
            索敌炮塔列表.Clear();
            驾驶舱列表.Clear();

            // 查找符合命名规则的编组
            var 编组列表 = new List<IMyBlockGroup>();
            GridTerminalSystem.GetBlockGroups(编组列表);

            foreach (var 编组 in 编组列表)
            {
                if (编组.Name.StartsWith(自定义炮塔编组前缀))
                {
                    // 检查本编程方块是否在该编组中
                    var 编程方块列表 = new List<IMyProgrammableBlock>();
                    编组.GetBlocksOfType(编程方块列表, 方块 => 方块.EntityId == Me.EntityId);

                    if (编程方块列表.Count > 0)
                    {
                        炮塔编组 = 编组;
                        Echo($"找到炮塔编组: {编组.Name}");
                        break;
                    }
                }
            }

            if (炮塔编组 == null)
            {
                Echo($"错误: 未找到以'{自定义炮塔编组前缀}'开头的编组");
                return;
            }

            // 在编组中查找方位转子
            var 转子列表 = new List<IMyMotorStator>();
            炮塔编组.GetBlocksOfType(转子列表);

            foreach (var 转子 in 转子列表)
            {
                if (转子.CustomName.Contains(方向机名称) && 转子.IsFunctional)
                {
                    方位转子 = 转子;
                    Echo($"找到方位转子: {方位转子.CustomName}");
                    break;
                }
            }

            if (方位转子 == null)
            {
                Echo($"错误: 编组内未找到包含'{方向机名称}'的转子");
            }

            // 在编组中查找俯仰转子
            foreach (var 转子 in 转子列表)
            {
                if (转子.CustomName.Contains(高低机名称) && 转子.IsFunctional)
                {
                    俯仰转子 = 转子;
                    Echo($"找到俯仰转子: {俯仰转子.CustomName}");
                    break;
                }
            }

            if (俯仰转子 == null)
            {
                Echo($"错误: 编组内未找到包含'{高低机名称}'的转子");
            }

            // 获取编组中的所有武器和索敌炮塔
            var 所有武器 = new List<IMyUserControllableGun>();
            炮塔编组.GetBlocksOfType(所有武器, 武器 => 武器.IsFunctional);

            // 分类武器和索敌炮塔
            foreach (var 武器 in 所有武器)
            {
                // 如果是大型炮塔，加入索敌列表
                if (武器 is IMyLargeTurretBase)
                {
                    索敌炮塔列表.Add(武器 as IMyLargeTurretBase);
                }
                else
                {
                    // 非大型炮塔的武器加入武器列表
                    武器列表.Add(武器);
                }
            }

            // 如果武器列表为空，提示错误
            if (武器列表.Count == 0)
            {
                Echo($"警告: 编组内未找到非炮塔类型的武器");
            }
            else
            {
                Echo($"找到 {武器列表.Count} 个武器");

                // 查找含"基线"的武器作为瞄准基准
                foreach (var 武器 in 武器列表)
                {
                    if (武器.CustomName.Contains(基线武器标记))
                    {
                        默认基线武器 = 武器;
                        Echo($"找到基线武器: {默认基线武器.CustomName}");
                        break;
                    }
                }

                // 如果没找到基线武器，使用第一个武器作为基准
                if (默认基线武器 == null)
                {
                    默认基线武器 = 武器列表[0];
                    Echo($"未找到基线武器，使用第一个武器作为基准: {默认基线武器.CustomName}");
                }
            }

            // 获取控制舱
            GridTerminalSystem.GetBlocksOfType(驾驶舱列表, ctrl => ctrl.IsFunctional && ctrl.CanControlShip);
            // 搜索编组内的所有武器方块，设置更新间隔为1
            var 战斗AI块 = new List<IMyOffensiveCombatBlock>();
            GridTerminalSystem.GetBlocksOfType(战斗AI块);
            foreach (var 块 in 战斗AI块)
            {
                if (块.IsFunctional)
                {
                    块.UpdateTargetInterval = 1;
                }
            }
            Echo($"找到 {索敌炮塔列表.Count} 个索敌炮塔");
            Echo($"找到 {驾驶舱列表.Count} 个控制器");

            // 获取武器基础信息
            Vector4D 武器基础信息 = 获取武器基础信息(默认基线武器 as IMyUserControllableGun, 武器列表.Count);
            武器弹速 = 武器基础信息.Z;
            最大射程 = (int)武器基础信息.W;
            重置全部武器状态(enabled: false, shoot: true);
        }

        #endregion

        #region 主 循 环

        public void Main(string argument, UpdateType updateSource)
        {
            try
            {
                // 处理命令
                if (!string.IsNullOrEmpty(argument))
                {
                    处理命令(argument);
                }

                // 高级控制和索敌（跳帧执行）
                if (运行计时 % 跳帧计数 == 0 || updateSource != UpdateType.Update1)
                {
                    更新索敌();
                }

                // 转子武器控制（每帧执行）
                if (系统就绪())
                {
                    更新转子炮台控制();
                }

                // 定期更新方块
                if (运行计时 % 方块更新跳帧数 == 0)
                {
                    获取方块();
                }

                // 更新计数和显示状态
                运行计时++;
                显示状态();
                更新性能统计();
            }
            catch (Exception e)
            {
                Echo($"错误: {e.Message}");
            }
        }

        private void 更新索敌()
        {
            // 获取当前控制器
            var 当前控制器 = 驾驶舱列表.FirstOrDefault(x => x.IsUnderControl);
            if (当前控制器 == null && 驾驶舱列表.Any())
                当前控制器 = 驾驶舱列表[0];

            if (当前控制器 == null) return;


            // 扫描目标
            MyDetectedEntityInfo 最佳目标 = 扫描目标();

            // 更新目标状态
            更新目标状态(最佳目标, 当前控制器);

            // // 如果存在有效目标，计算并缓存预测位置
            if (存在有效目标 && 系统就绪())
            {
                缓存预测位置 = 计算预测位置();
            }
            else
            {
                // 如果没有有效目标，将缓存预测位置设为1000m外，方向为前方
                Vector3D 前方方向 = 当前控制器.WorldMatrix.Forward;
                缓存预测位置 = 当前控制器.GetPosition() + 前方方向 * 1000.0;
            }
        }

        private void 更新转子炮台控制()
        {
            // 如果使用轮射
            if (使用轮射 && 武器列表.Count > 0)
            {
                当前参考基线武器 = 武器列表[轮射武器索引];
            }
            else
            {
                当前参考基线武器 = 默认基线武器 as IMyUserControllableGun;
            }
            // 使用缓存的预测位置进行瞄准
            var 角度误差 = 计算旋转角度(缓存预测位置, 当前参考基线武器);
            角度误差_显示用 = 角度误差;
            if (!角度误差.HasValue)
                return;
            double 方位误差 = 角度误差.Value.X;
            double 俯仰误差 = 角度误差.Value.Y;

            // PID控制 - 使用PID控制器获取输出
            double 方位输出 = 方向机PID控制器.GetOutput(方位误差);
            double 俯仰输出 = 高低机PID控制器.GetOutput(俯仰误差);

            // 应用到转子
            方位转子.TargetVelocityRad = (float)方位输出;
            俯仰转子.TargetVelocityRad = (float)俯仰输出;

            // 检查是否到达目标
            bool 已瞄准 = Math.Abs(方位误差) < 角度容差 &&
                        Math.Abs(俯仰误差) < 角度容差;


            bool 目标在范围内 = 目标距离 <= 最大射程 && 存在有效目标;

            检查并关闭超时武器();

            // if (已瞄准)
            // {
            //     方位转子.TargetVelocityRad = 0;
            //     俯仰转子.TargetVelocityRad = 0;
            // }
            if (目标在范围内 && 已瞄准)
            {
                发射武器();
            }
            if (正在齐射 && (!已瞄准 || !目标在范围内))
            {
                // 如果正在齐射但未瞄准或目标不在范围内，重置武器状态
                重置全部武器状态(enabled: false, shoot: true);
                正在齐射 = false; // 停止齐射
            }
        }

        #endregion

        #region 目标扫描与跟踪

        private MyDetectedEntityInfo 扫描目标()
        {
            List<MyDetectedEntityInfo> 目标列表 = new List<MyDetectedEntityInfo>();

            foreach (var 炮塔 in 索敌炮塔列表)
            {
                if (炮塔 == null || !炮塔.IsFunctional) continue;

                var 探测目标 = 炮塔.GetTargetedEntity();
                if (!探测目标.IsEmpty())
                {
                    目标列表.Add(探测目标);
                }
            }

            // 选择最大的目标
            return 目标列表.Any() ?
                目标列表.Aggregate((a, b) => (a.BoundingBox.Volume > b.BoundingBox.Volume) ? a : b) :
                new MyDetectedEntityInfo();
        }

        private void 更新目标状态(MyDetectedEntityInfo 目标, IMyShipController 控制器)
        {
            if (!目标.IsEmpty())
            {
                // 检查是否为新目标
                if (!存在有效目标 || 当前目标.EntityId != 目标.EntityId)
                {
                    // 新目标，清除历史记录
                    目标跟踪器.ClearHistory();
                }

                // 更新当前目标
                当前目标 = 目标;
                存在有效目标 = true;

                // 计算距离
                目标距离 = Vector3D.Distance(目标.Position, 控制器.GetPosition());
                目标位置_显示用 = 目标.Position;

                // 更新目标跟踪器
                目标跟踪器.UpdateTarget(目标,hasVelocityAvailable: false);
            }
            else
            {
                // 没有目标
                if (存在有效目标)
                {
                    存在有效目标 = false;
                    目标跟踪器.ClearHistory();
                }
                目标距离 = 0;
            }
        }

        #endregion

        #region 弹道计算

        /// <summary>
        /// 计算预测位置，使用线性解为初值进行迭代优化
        /// /// </summary>
        /// <returns>预测的拦截位置</returns>
        private Vector3D 计算预测位置()
        {
            if (!存在有效目标) return Vector3D.Zero;

            // 获取炮台和目标的位置
            Vector3D 武器位置 = 当前参考基线武器.GetPosition();
            Vector3D 目标当前位置 = 当前目标.Position;
            Vector3D 目标速度 = 当前目标.Velocity;

            // 获取重力信息
            Vector3D 重力向量 = Vector3D.Zero;
            bool 存在重力 = false;
            // 获取舰船速度
            Vector3D 舰船速度 = Vector3D.Zero;
            if (驾驶舱列表.Any())
            {
                舰船速度 = 驾驶舱列表[0].GetShipVelocities().LinearVelocity;
                重力向量 = 驾驶舱列表[0].GetNaturalGravity();
                存在重力 = 重力向量.LengthSquared() > 0.05;
            }

            // 计算线性解作为初始拦截点
            Vector3D 拦截点;
            Vector3D 相对位置 = 目标当前位置 - 武器位置;
            Vector3D 相对速度 = 目标速度 - 舰船速度;

            // 解二次方程计算拦截时间
            double a = 相对速度.LengthSquared() - Math.Pow(武器弹速, 2);

            // a≈0时表示目标速度接近武器弹速，会导致解不稳定
            if (Math.Abs(a) > 0.01)
            {
                double b = 2 * Vector3D.Dot(相对位置, 相对速度);
                double c = 相对位置.LengthSquared();
                double 判别式 = b * b - 4 * a * c;

                if (判别式 >= 0)
                {
                    // 有实数解，计算拦截时间
                    double t1 = (-b + Math.Sqrt(判别式)) / (2 * a);
                    double t2 = (-b - Math.Sqrt(判别式)) / (2 * a);

                    // 选择有效的正解（如果有）
                    double 拦截时间 = double.NaN;
                    if (t1 > 0 && t2 > 0)
                        拦截时间 = Math.Min(t1, t2);
                    else if (t1 > 0)
                        拦截时间 = t1;
                    else if (t2 > 0)
                        拦截时间 = t2;

                    if (!double.IsNaN(拦截时间) && 拦截时间 > 0)
                    {
                        // 使用线性解作为迭代起点
                        拦截点 = 目标当前位置 + 目标速度 * 拦截时间 - 舰船速度 * 拦截时间;
                    }
                    else
                    {
                        // 拦截时间无效，使用当前位置
                        拦截点 = 目标当前位置;
                    }
                }
                else
                {
                    // 无实数解，使用当前位置
                    拦截点 = 目标当前位置;
                }
            }
            else
            {
                // 特殊情况，使用当前位置
                拦截点 = 目标当前位置;
            }

            // 迭代求解最佳拦截点，使用预定义的迭代次数
            for (int i = 0; i < 预测迭代次数; i++)
            {
                // 计算预判位置需要的时间
                double 距离 = Vector3D.Distance(武器位置, 拦截点);
                double 飞行时间 = 距离 / 武器弹速;

                // 预测目标在未来位置
                long 预测时间ms = (long)(飞行时间 * 1000) + 跳帧计数 * 17;
                var 目标信息 = 目标跟踪器.PredictFutureTargetInfo(预测时间ms, false);

                // 参考系变换：从绝对位置转换到相对位置
                // 计算舰船在飞行时间内的位移
                Vector3D 舰船位移 = 舰船速度 * 飞行时间;

                // 对自身采用匀速模型进行修正
                Vector3D 新拦截点 = 目标信息.Position - 舰船位移;

                // 检查预测收敛条件
                if (Vector3D.Distance(拦截点, 新拦截点) < 0.5)
                    break;

                拦截点 = 新拦截点;

                当前最大迭代次数 = Math.Max(当前最大迭代次数, i + 1);
            }
            // 简单重力补偿-忽略抛物线轨迹的时间影响
            if (存在重力 && 弹药受重力影响)
            {
                // 计算预判位置需要的时间
                double 距离 = Vector3D.Distance(武器位置, 拦截点);
                double 飞行时间 = 距离 / 武器弹速;
                // 重力导致的弹道下降量: 1/2 * g * t²
                Vector3D 重力补偿量 = 0.5 * 重力向量 * 飞行时间 * 飞行时间;
                // 向重力反方向补偿，使弹道恰好命中目标
                拦截点 -= 重力补偿量;
            }
            return 拦截点;
        }

        #endregion

        #region 转子控制

        private Vector2D? 计算旋转角度(Vector3D 目标位置, IMyUserControllableGun 当前基线)
        {
            if (默认基线武器 == null || 方位转子 == null || 俯仰转子 == null)
                return null;

            // 获取武器和目标的位置关系
            Vector3D 武器位置 = 当前基线.GetPosition();
            Vector3D 目标方向 = Vector3D.Normalize(目标位置 - 武器位置);
            Vector3D 武器前方 = 当前基线.WorldMatrix.Forward;

            // 获取方位转子的旋转轴
            Vector3D 方位轴 = 方位转子.WorldMatrix.Up;

            // 水平平面投影
            Vector3D 目标水平 = Vector3D.Reject(目标方向, 方位轴);
            Vector3D 武器水平 = Vector3D.Reject(武器前方, 方位轴);

            // 规范化向量
            武器水平 = Vector3D.Normalize(武器水平);
            目标水平 = Vector3D.Normalize(目标水平);

            // 计算水平角度差（方位角）
            double 余弦值 = Vector3D.Dot(武器水平, 目标水平);
            Vector3D 叉积 = Vector3D.Cross(武器水平, 目标水平);
            double 角度符号 = -Math.Sign(Vector3D.Dot(叉积, 方位轴));
            double 方位误差 = 角度符号 * Math.Acos(MathHelper.Clamp(余弦值, -1.0, 1.0));

            // // 计算垂直角度差（俯仰角）
            // double 武器仰角 = Math.Asin(MathHelper.Clamp(Vector3D.Dot(武器前方, 方位轴), -1.0, 1.0));
            // double 目标仰角 = Math.Asin(MathHelper.Clamp(Vector3D.Dot(目标方向, 方位轴), -1.0, 1.0));
            // double 俯仰误差 = 目标仰角 - 武器仰角;

            // 获取俯仰转子轴
            Vector3D 俯仰轴 = 俯仰转子.WorldMatrix.Up;
            Vector3D 目标垂直 = Vector3D.Reject(目标方向, 俯仰轴);
            Vector3D 武器垂直 = Vector3D.Reject(武器前方, 俯仰轴);
            // 归一化
            目标垂直 = Vector3D.Normalize(目标垂直);
            武器垂直 = Vector3D.Normalize(武器垂直);
            余弦值 = Vector3D.Dot(武器垂直, 目标垂直);
            叉积 = Vector3D.Cross(武器垂直, 目标垂直);
            角度符号 = -Math.Sign(Vector3D.Dot(叉积, 俯仰轴));
            double 俯仰误差 = 角度符号 * Math.Acos(MathHelper.Clamp(余弦值, -1.0, 1.0));
            // 直接返回需要旋转的角度差
            return new Vector2D(方位误差, 俯仰误差);
        }

        #endregion

        #region 武器控制

        private void 发射武器()
        {
            if (武器列表.Count == 0) return;

            if (使用轮射)
            {
                执行轮射();
            }
            else
            {
                执行齐射();// 备注：齐射不参与武器计时，超时关闭等功能
            }
        }
        private void 重置全部武器状态(bool enabled = true,bool shoot = false)
        {
            if(武器列表.Count == 0) return;
            if ((武器列表[0].BlockDefinition.SubtypeId == "LargeRailgun" ||
                武器列表[0].BlockDefinition.SubtypeId == "SmallRailgun" ) &&
                enabled == false && shoot == true)// 当尝试使用常规模式关闭磁轨炮射击时
            {
                enabled = true;
                shoot = false;
            }
            for (int i = 0; i < 武器列表.Count; i++)
                {
                    武器列表[i].Enabled = enabled;
                    武器列表[i].Shoot = shoot;
                }
        }
        
        /// <summary>
        /// 检查并关闭达到延迟时间的武器
        /// </summary>
        private void 检查并关闭超时武器()
        {
            if (武器列表.Count == 0 || 武器激活时间.Count == 0) return;

            // 检查需要关闭的武器
            List<int> 待关闭武器 = new List<int>();
            foreach (var 项 in 武器激活时间)
            {
                int 索引 = 项.Key;
                int 激活时间 = 项.Value;

                // 检查是否达到延迟关闭时间
                if (运行计时 - 激活时间 >= 射击关闭延迟)
                {
                    if (索引 < 武器列表.Count && 武器列表[索引].IsFunctional)
                    {
                        if(武器列表[索引].BlockDefinition.SubtypeId == "LargeRailgun" || 
                           武器列表[索引].BlockDefinition.SubtypeId == "SmallRailgun")
                        {
                            // 对于磁轨类武器，关闭射击功能
                            武器列表[索引].Shoot = false;
                        }
                        else 武器列表[索引].Enabled = false;
                    }
                    待关闭武器.Add(索引);
                }
            }

            // 从激活时间字典中移除已关闭的武器
            foreach (int 索引 in 待关闭武器)
            {
                武器激活时间.Remove(索引);
            }
        }

        private void 执行齐射()
        {
            if (武器列表.Count == 0 || 正在齐射) return;
            // 如果未激活，激活所有可用武器
            重置全部武器状态(enabled: true, shoot: true);
            正在齐射 = true;
        }

        private void 执行轮射()
        {
            if (武器列表.Count == 0) return;
            // 激活新一批武器
            if (运行计时 % 轮射间隔 == 0)
            {
                for (int i = 0; i < 轮射窗口长度; i++)
                {
                    if (轮射武器索引 < 武器列表.Count && 武器列表[轮射武器索引].IsFunctional)
                    {
                        if(武器列表[轮射武器索引].BlockDefinition.SubtypeId == "LargeRailgun" || 
                           武器列表[轮射武器索引].BlockDefinition.SubtypeId == "SmallRailgun")
                        {
                            // 对于磁轨类武器，开启射击功能
                            武器列表[轮射武器索引].Shoot = true;
                        }
                        else 武器列表[轮射武器索引].Enabled = true;
                        武器激活时间[轮射武器索引] = 运行计时; // 记录激活时间
                    }         
                    // 更新索引
                    轮射武器索引 = (轮射武器索引 + 1) % 武器列表.Count;
                }
            }
        }

        #endregion

        #region 命令处理

        private void 处理命令(string argument)
        {
            string[] 参数 = argument.Split(' ');

            // 目标坐标设置
            if (参数.Length >= 4 && (参数[0].ToLower() == "target" || 参数[0] == "目标"))
            {
                try
                {
                    double x = Convert.ToDouble(参数[1]);
                    double y = Convert.ToDouble(参数[2]);
                    double z = Convert.ToDouble(参数[3]);

                    缓存预测位置 = new Vector3D(x, y, z);
                    Echo($"目标设置为: ({x:F1}, {y:F1}, {z:F1})");
                }
                catch
                {
                    Echo("错误: 无效的坐标格式。使用: 目标 X Y Z");
                }
            }
            // 停止命令
            else if (参数[0].ToLower() == "stop" || 参数[0] == "停止")
            {
                清除目标();
                Echo("目标已清除");
            }
            // 重置命令
            else if (参数[0].ToLower() == "reset" || 参数[0] == "重置")
            {
                获取方块();
                目标跟踪器.ClearHistory();
                Echo("系统已重置，重新扫描炮塔编组");
            }
            // 跳帧设置
            else if (参数[0] == "跳帧切换")
            {
                跳帧计数 = 跳帧计数 == 默认跳帧数 ? 备用跳帧数 : 默认跳帧数;
                按预测窗口时长设置目标跟踪器(预测窗口时长);
                Echo($"跳帧设置为: {跳帧计数}");
            }
            else if (参数.Length > 1 && 参数[0] == "设置跳帧")
            {
                int 值;
                if (int.TryParse(参数[1], out 值) && 值 > 0)
                {
                    跳帧计数 = 值;
                    Echo($"跳帧设置为: {值}");
                }
                按预测窗口时长设置目标跟踪器(预测窗口时长);
            }
            else if (参数.Length > 1 && 参数[0] == "设置窗口时长")
            {
                int 值;
                if (int.TryParse(参数[1], out 值) && 值 > 0)
                {
                    预测窗口时长 = 值;
                    按预测窗口时长设置目标跟踪器(预测窗口时长);
                    Echo($"预测窗口时长设置为: {值} ms");
                }
            }
            // 弹速设置
            else if (参数.Length > 1 && 参数[0] == "设置弹速")
            {
                double 值;
                if (double.TryParse(参数[1], out 值) && 值 > 0)
                {
                    武器弹速 = 值;
                    Echo($"弹速设置为: {值} m/s");
                }
            }
            // 射击模式设置
            else if (参数[0] == "切换射击模式")
            {
                使用轮射 = !使用轮射;
                Echo($"射击模式: {(使用轮射 ? "轮射" : "齐射")}");
            }
            else if (参数.Length > 1 && 参数[0] == "设置轮射间隔")
            {
                int 值;
                if (int.TryParse(参数[1], out 值) && 值 > 0)
                {
                    轮射间隔 = 值;
                    Echo($"轮射间隔设置为: {值}帧");
                }
            }
        }

        private void 清除目标()
        {
            存在有效目标 = false;
            if (方位转子 != null) 方位转子.TargetVelocityRad = 0;
            if (俯仰转子 != null) 俯仰转子.TargetVelocityRad = 0;
            // 重置PID控制器状态
            方向机PID控制器.Reset();
            高低机PID控制器.Reset();
        }

        #endregion

        #region 辅助方法

        private bool 系统就绪()
        {
            return 方位转子 != null && 方位转子.IsFunctional &&
                   俯仰转子 != null && 俯仰转子.IsFunctional &&
                   默认基线武器 != null && 默认基线武器.IsFunctional;
        }
        /// <summary>
        /// 获取默认轮射窗口 - 根据武器类型和数量计算轮射参数
        /// </summary>
        /// <param name="武器">基线武器</param>
        /// <param name="武器数量">武器总数</param>
        /// <returns>返回轮射间隔、窗口大小和弹速</returns>
        private Vector4D 获取武器基础信息(IMyUserControllableGun 武器, int 武器数量)
        {
            if (武器 == null || 武器数量 <= 0)
                return new Vector4D(默认轮射间隔, 0, 500, 800); // 默认间隔和单个武器窗口

            // 根据武器类型获取基础射速(RPM)
            float 基础射速RPM;
            float 弹速;
            float 射程;


            string 子类型ID = 武器.BlockDefinition.SubtypeId;

            switch (子类型ID)
            {
                case "SmallBlockAutocannon":
                    基础射速RPM = 150f;   // 机炮: 150 RPM
                    弹速 = 400f;
                    射程 = 800f;
                    break;
                case "SmallBlockMediumCalibreGun":
                    基础射速RPM = 10f;    // 突击炮: 10 RPM
                    弹速 = 500f;
                    射程 = 1400f;
                    break;
                case "LargeBlockLargeCalibreGun":
                    基础射速RPM = 5f;     // 火炮: 5 RPM
                    弹速 = 500f;
                    射程 = 2000f;
                    break;
                case "LargeRailgun":
                    基础射速RPM = 1.0133f; // 大型轨道炮: 1.0133 RPM
                    弹速 = 2000f;
                    射程 = 2000f;
                    break;
                case "SmallRailgun":
                    基础射速RPM = 3f;     // 小型轨道炮: 3 RPM
                    弹速 = 1000f;
                    射程 = 1400f;
                    break;
                default:
                    if (武器 is IMySmallGatlingGun)
                    {
                        基础射速RPM = 700f; // 机枪: 700 RPM
                        弹速 = 400f;
                        射程 = 800f;
                    }
                    else if (武器 is IMySmallMissileLauncher)
                    {
                        // 导弹发射器根据网格大小区分
                        基础射速RPM = 武器.CubeGrid.GridSizeEnum == MyCubeSize.Large ? 120f : 60f;
                        弹速 = 200f;
                        射程 = 800f;
                    }
                    else
                    {
                        基础射速RPM = 60f;    // 默认射速: 60 RPM
                        弹速 = 500f; // 默认弹速
                        射程 = 800f; // 默认射程
                    }
                    弹药受重力影响 = false;
                    break;
            }

            // 将RPM转换为帧间隔 (60fps)
            int 单武器间隔 = (int)Math.Ceiling(3600f / 基础射速RPM);

            // 计算理论上每个武器的发射间隔
            float 理论间隔 = (float)单武器间隔 / 武器数量;

            // 窗口大小与间隔计算
            int 窗口大小;
            int 轮射间隔;

            if (理论间隔 >= 1f)
            {
                // 射速适中，每个tick发射一个武器
                窗口大小 = 1;
                轮射间隔 = Math.Max(1, (int)Math.Ceiling(理论间隔));
            }
            else
            {
                // 射速太快，需要多个武器一起发射
                窗口大小 = (int)Math.Floor(1f / 理论间隔);
                轮射间隔 = 1; // 最小间隔1tick

                // 确保窗口大小不超过总武器数
                窗口大小 = Math.Min(窗口大小, 武器数量);
            }

            return new Vector4D(轮射间隔, 窗口大小, 弹速, 射程);
        }
        private void 显示状态()
        {
            Echo("=== 自定义炮塔火控系统 v1.3.2 ===");
            Echo($"炮塔编组: {炮塔编组?.Name ?? "未找到"}");
            Echo($"跳帧设置: {跳帧计数} | 弹速: {武器弹速:F0} m/s");
            Echo($"武器名称: {默认基线武器?.BlockDefinition.SubtypeId ?? "未找到"}");
            Echo($"射击模式: {(使用轮射 ? "轮射" : "齐射")} | 间隔: {轮射间隔}帧");
            Echo($"轮射窗口: {轮射窗口长度} | 最大射程: {最大射程}m | 当前武器: {轮射武器索引}");
            Echo("");

            // 系统状态
            Echo("系统状态:");
            Echo($"方位转子: {(方位转子?.IsFunctional == true ? "正常" : "未找到/故障")}");
            Echo($"俯仰转子: {(俯仰转子?.IsFunctional == true ? "正常" : "未找到/故障")}");
            Echo($"基线武器: {(默认基线武器?.IsFunctional == true ? 默认基线武器.CustomName : "未找到/故障")}");
            Echo($"武器数量: {武器列表.Count} | 索敌炮塔: {索敌炮塔列表.Count}");
            Echo("");
            if (驾驶舱列表.Count == 0)
            {
                Echo($"未找到飞船控制 - 当飞船运动时，可能无法正确计算目标位置");
            }

            if (存在有效目标)
            {
                if (系统就绪())
                {
                    // 显示转子角度 部署后可以注释掉以节省资源
                    var 角度误差 = 角度误差_显示用;
                    if (角度误差.HasValue)
                    {
                        double 方位误差 = 角度误差.Value.X;
                        double 俯仰误差 = 角度误差.Value.Y;

                        Echo($"方位角误差: {方位误差 * 180 / Math.PI:F2}°");
                        Echo($"俯仰角误差: {俯仰误差 * 180 / Math.PI:F2}°");
                        Echo($"方位速度: {方位转子.TargetVelocityRad:F2} rad/s");
                        Echo($"俯仰速度: {俯仰转子.TargetVelocityRad:F2} rad/s");

                        bool 已瞄准 = Math.Abs(方位误差) < 角度容差 &&
                                    Math.Abs(俯仰误差) < 角度容差;
                        Echo($"瞄准状态: {(已瞄准 ? "已锁定" : "调整中")}");
                    }

                    Echo("目标信息:");
                    if (当前目标.EntityId != 0)
                    {
                        Echo($"目标ID: {当前目标.EntityId % 1e12:F0}");
                    }
                    // Echo($"目标坐标: ({目标位置.X:F1}, {目标位置.Y:F1}, {目标位置.Z:F1})");
                    Echo($"目标距离: {目标距离:F1}m");

                    // 显示跟踪预测信息
                    Echo($"历史记录: {目标跟踪器.GetHistoryCount()}");
                    Echo($"预测误差: {目标跟踪器.combinationError:F3}m/s");
                    Echo($"多项式权重: {目标跟踪器.linearWeight:F2}");
                    Echo($"环绕权重: {目标跟踪器.circularWeight:F2}");
                    // Echo($"线性权重: {目标跟踪器.linearWeight:F2}");
                }
            }
            else
            {
                Echo("无目标 - 使用 '目标 X Y Z' 设置目标坐标");
            }

            // 显示性能统计
            Echo(性能统计.ToString());
        }

        private void 更新性能统计()
        {
            // 更新运行时间统计
            double 最后运行时间 = Runtime.LastRunTimeMs;

            总运行时间ms += 最后运行时间;
            性能统计运行次数++;

            if (最后运行时间 > 最大运行时间ms)
                最大运行时间ms = 最后运行时间;

            if (性能统计运行次数 > 600)
            {
                // 重置统计信息以避免溢出
                总运行时间ms = 最后运行时间;
                最大运行时间ms = 最后运行时间;
                性能统计运行次数 = 1;
            }

            // 更新统计显示
            性能统计.Clear();
            性能统计.AppendLine("=== 性能统计 ===");
            性能统计.AppendLine($"上次运行: {最后运行时间:F3} ms");
            性能统计.AppendLine($"平均运行: {总运行时间ms / 性能统计运行次数:F3} ms");
            性能统计.AppendLine($"最大运行: {最大运行时间ms:F3} ms");
            性能统计.AppendLine($"最大迭代: {当前最大迭代次数}");
            性能统计.AppendLine($"指令使用: {Runtime.CurrentInstructionCount}/{Runtime.MaxInstructionCount}");
        }

        #endregion
    }
}