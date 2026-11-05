# 实验室机器人项目

本科期间在实验室做的一些机器人小项目，主要是 STM32 小车的循迹避障和四足机器人的步态控制。

> 代码里出现 `TODO(verify-on-hardware)` 的地方，都是必须上机才能确认的取值
> （引脚、晶振、舵机行程、机身尺寸）。这些位置按"最小且可推导"的原则给出默认值，
> 上机前请逐条核对。

---

## 1. STM32 循迹避障小车

**硬件**：STM32F103C8T6 + 5路红外传感器 + HC-SR04超声波 + L298N电机驱动

**实现的功能**：
- 红外循迹：5路传感器加权平均算偏移，PID控制电机差速
- 超声波避障：检测到15cm内有障碍物就停车右转（停车200ms → 原地右转400ms，全部由
  非阻塞状态机计时，主循环不卡在 `HAL_Delay` 里）
- 有符号电机控制：`Motor_SetSpeed(-300, 300)` 是真正的原地右转（方向引脚 + `abs()` 占空比）

**时钟**：HSE 8MHz → PLL×9 → 72MHz；AHB /1（HCLK 72MHz）、APB1 /2（PCLK1 36MHz）、
APB2 /1，Flash 2 wait states。APB1 分频后定时器时钟仍为 72MHz，所以 TIM2
`Prescaler=71, Period=999` 正好是 **1kHz PWM**，占空比分辨率 0~999。

### 引脚分配

| 功能 | 引脚 | 备注 |
|---|---|---|
| 5 路红外 | PB0 / PB1 / PB2 / PB3 / PB4 | 原来是 PA0–PA4，见下方说明 |
| L298N ENA / ENB（PWM） | PA0 / PA1 | TIM2_CH1 / CH2，默认映射，配成复用推挽 |
| L298N IN1 / IN2（左） | PB12 / PB13 | `TODO(verify-on-hardware)` |
| L298N IN3 / IN4（右） | PB14 / PB15 | `TODO(verify-on-hardware)` |
| HC-SR04 TRIG | PB5 | `TODO(verify-on-hardware)` |
| HC-SR04 ECHO | PB6 | `TODO(verify-on-hardware)`，5V 输出建议加 1k/2k 分压 |
| SWD 调试 | PA13 / PA14 | 关 JTAG、保 SWD 后仍可下载 |

方向引脚和超声波引脚都集中在 `inc/motor.h`、`inc/sensor.h` 的宏里，换线只改这两个文件。
L298N 的 IN1/IN2 极性取决于电机出线方向：如果某个轮子反转，交换对应的一组 IN 宏即可。

### 为什么红外从 PA0–PA4 挪到了 PB0–PB4

PA0/PA1 同时是 TIM2_CH1/CH2 的默认引脚，也就是 ENA/ENB 的 PWM 输出。原来的
`Sensor_Init()` 把 PA0/PA1 配成上拉输入，等 `HAL_TIM_PWM_Start()` 启动时引脚功能已经
被覆盖成输入，PWM 根本到不了 L298N（当时也还没有方向引脚），电机拿不到任何有效驱动。

现在的分工：
- `MX_TIM2_Init()` 负责把 PA0/PA1 配成 `GPIO_MODE_AF_PP`（复用推挽）并初始化 TIM2；
- `Sensor_Init()` 只初始化 PB0–PB4，不再碰 PA 口；
- PB3/PB4 复位后是 JTAG 的 JTDO/NJTRST，`Sensor_Init()` 调用
  `__HAL_AFIO_REMAP_SWJ_NOJTAG()` 释放这两个脚，同时保留 SWD（PA13/PA14）用于下载调试。

如果板子必须把红外留在 PA 口，另一条路是给 TIM2 做重映射
（`__HAL_AFIO_REMAP_TIM2_PARTIAL_1` 或 `FULL`，CH1/CH2 → PA15/PB3）或换 TIM3/TIM4 的通道；
两种做法都要同步改 `motor.h` 里的 PWM 通道宏、`MX_TIM2_Init()` 和 CubeMX 配置。

### 编译 / 运行

**本仓库只含应用层源码（`src/` + `inc/`），不含 `.ioc`、Makefile、startup 文件和链接脚本，
不能单独编译。** 需要配合 STM32CubeMX 生成的工程使用：

1. CubeMX 新建 STM32F103C8T6 工程，时钟源选 HSE 8MHz 晶振，SYSCLK 72MHz；
2. 使能 TIM2 的 CH1/CH2 为 PWM Generation（Prescaler 71、Period 999）；
3. 需要的 HAL 模块：
   - `stm32f1xx_hal.c` / `stm32f1xx_hal_cortex.c`（`HAL_Init`、`HAL_GetTick`、`HAL_Delay`）
   - `stm32f1xx_hal_rcc.c` + `rcc_ex.c`（时钟树）
   - `stm32f1xx_hal_gpio.c` + `gpio_ex.c`（GPIO 与 AFIO 的 JTAG 重映射）
   - `stm32f1xx_hal_tim.c` + `tim_ex.c`（PWM）
   - `stm32f1xx_hal_flash.c` + `flash_ex.c`（等待周期与预取）
   - `stm32f1xx_hal_uart.c`：**当前代码没有用到**，只有自己加 `printf` 调试时才需要
     （用 USART1 的 PA9/PA10，注意别和 PB6 的 ECHO 抢重映射）
4. 把 `src/`、`inc/` 加进工程。`main.c` 里已经自带了 `SystemClock_Config()` 和
   `MX_TIM2_Init()`，如果直接用 CubeMX 生成的 `main.c`，要么保留 CubeMX 的版本并删掉本仓库
   的同名函数，要么删掉 CubeMX 的版本——**不要两个都留，否则重复定义**；
5. 编译选项建议 C99（代码里用了 `for (int i = ...)` 和语句后声明）。

### 踩坑记录

- PID参数一开始Kp=50，小车在弯道抖得厉害，后来降到25并加了积分限幅，直线误差从5cm降到2cm左右
- PID 的积分/微分原来按"每次循环"累加，主循环周期一变（避障、超声波超时）等效增益就跟着变；
  现在 `PID_Update()` 多一个 `dt` 参数，用 `HAL_GetTick()` 的实测间隔做归一化。
  归一化后 Ki/Kd 的量纲从"每次"变成"每秒"，`main.c` 里按 10ms 标称周期换算了等效增益
  （Ki 0.5 → 50、Kd 8 → 0.08），**上机还需要重新整定**；
  积分限幅由 `PID_SetOutputLimit()` 取输出上限的一半，限制的是积分**项**（`Ki*integral`），
  所以积分单独不可能打满转向
- 红外加权平均原来是整数除法，`-3/2` 被截断成 `-1`，PID 在偏差较小时会偏；改成浮点后
  是 `-1.5`
- 红外传感器原接 PA0–PA4，和 TIM2_CH1/CH2 冲突（见上节）
- `Motor_SetSpeed()` 原来把负速度钳成 0，`Motor_SetSpeed(-300, 300)` 实际上只是左轮停转，
  原地右转从来没生效过；现在用符号控制 IN1/IN2、IN3/IN4
- 超声波原来是个恒返回 `30.0` 的桩函数，避障分支永远为真；现在补了真实的
  TRIG 10µs 脉冲 + ECHO 高电平计时（`duration * 0.034 / 2`），20ms 超时，
  两次测距之间至少间隔 60ms（HC-SR04 要求），超时或低于 2cm 死区时返回
  `US_NO_ECHO_DISTANCE_CM`（400cm，视为前方无障碍）
- 超声波HC-SR04在快速移动时测距不稳定，有时候误触发，加了延时滤波后好了一些
- 红外传感器在阳光下会受干扰，后来加了黑色遮光罩

---

## 2. ROS 四足步态控制

在实验室用 Python + ROS 写的四足机器人步态控制，实现了Trot步态。

**运动学**：3自由度单腿（髋偏航 + 大腿 + 小腿），解析法求逆解，带关节限幅

**步态**：Trot（对角腿同步），占空比0.5，摆相用Cycloid曲线做足端轨迹

**软件架构**：
```
    ROS Master
    ┌──────────────────────┐
    │ trot_gait_controller │
    │  ┌───────┐ ┌───────┐ │
    │  │ Trot  │ │ 逆解  │ │
    │  │ 步态  │→│ (IK) │ │
    │  └───────┘ └───────┘ │
    │         ↓            │
    │    /joint_states     │
    └──────────────────────┘
```

**代码结构**：
```
quadruped-ros/
└── src/quadruped_gait/
    ├── scripts/
    │   ├── kinematics.py      # 运动学正逆解（库文件，不作为节点安装）
    │   └── gait_controller.py # 步态生成节点
    ├── launch/
    │   └── trot_gait.launch
    ├── CMakeLists.txt
    └── package.xml
```

**坐标系**：机身系 x 前、y 左、z 上；每腿的髋偏航以"该腿朝外的中立方向"为零位
（左腿 +90°、右腿 −90°），`leg_id` 同时决定髋安装位置和左右镜像。
中立站姿足端在髋安装点外侧 `L1 + stance_radius` 处、机身下方 `body_height`。

**一个步态周期**：
- 支撑相：足端在机身系里从 `+step_length/2` 扫到 `-step_length/2`（脚踩在地上不动，
  机身相对脚向前走），
- 摆动相：抬脚后用 Cycloid 曲线从 `-step_length/2` 回到 `+step_length/2`，
- 两相在 `±step_length/2` 处衔接，位置连续，不会跳变。

**编译 / 运行**：
```bash
cd quadruped-ros && catkin_make
rosrun quadruped_gait gait_controller.py     # 或 roslaunch quadruped_gait trot_gait.launch
```
运行依赖 `python3-numpy`（已写进 `package.xml` 的 `exec_depend`）。

**踩坑记录**：
- 逆解一开始没加角度限幅，算出来有时候关节角度超出物理范围。现在的做法**不是 clip**：
  `kinematics.py` 对髋/大腿/小腿三组角度按 `JOINT_LIMITS` 检查，超限直接返回 `None`
  并向 stderr 打印 leg_id 和具体角度；`gait_controller.py` 拿到 `None` 时保持上一组
  有效角度并计数，连续 5 次失败就 `rospy.logerr` 并停止发布（不再用固定角度
  `(0, -0.3, 0.6)` 顶着继续发）
- 髋角原来直接对机身坐标做 `atan2(y, x)`，而调用方恒传 `y=0`、`fx` 半周期为负，
  于是髋角在 0 和 π 之间来回跳。现在先把足端坐标减去髋安装偏移、再按 `leg_id` 的
  镜像方向旋转到腿的局部坐标系，`atan2` 只在这个局部系里用，髋角在 0 附近小幅变化
- 足端落进髋部圆内时 `r = sqrt(x²+y²) - L1 < 0`，原来的 2 连杆公式会算出一个镜像解
  却不报错；现在直接判为无解并打印，不再继续算
- 中立站姿参数是有几何约束的：`stance_radius=0.055`、`body_height=0.18`、半步长 `0.03`，
  最远点 `sqrt(0.085² + 0.18²) = 0.199 m`，刚好在 `L2 + L3 = 0.2 m` 工作空间内；
  改步长/机身高之前先算一下这个不等式，否则 IK 会持续失败并停节点
- Trot步态相位一开始没调好，对角腿不同步，调试了很久才稳定
- 没有实体机器狗，目前只在RViz里可视化验证

---

## 上机验证清单（按优先级）

1. **先不要接电机**：上电测 PA0/PA1（TIM2_CH1/CH2）是否有 1kHz、占空比随速度变化的 PWM，
   确认时钟树和 GPIO 复用正确。
2. 测 PB0–PB4 的红外电平，确认 PB3/PB4 已从 JTAG 释放（SWD 还能连上）。
3. 接电机，临时在 `main()` 里发 `Motor_SetSpeed(300, 300)` / `(-300, -300)` / `(-300, 300)`，
   确认方向引脚与轮子转向一致（不一致就交换对应的 IN1/IN2、IN3/IN4 宏）。
4. 测超声波：串口打印距离，用手在 10/20/30cm 处对比卷尺，确认 ECHO 分压和 20ms 超时。
5. 整车循迹：先低速直道，再弯道，观察 `line_position` 是否随偏差单调变化、并且能出现
   `-1.5` / `0.5` 这类半整数值（整数除法时只可能是整数）。
6. 避障：15cm 内应"停 200ms → 原地右转 400ms → 冷却 300ms 后继续循迹"。
7. 四足：先在 RViz 里看 `joint_states` 是否连续（髋角不应在 0/π 跳变），
   再上舵机、拆掉负载单独标定每个关节的 `JOINT_LIMITS`。

---

**技术栈**：C、Python、STM32 HAL、ROS、NumPy
