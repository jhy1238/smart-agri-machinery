# 核心文献 10 篇 · ROS 履带式智能农机

> **选文标准**：① 主题聚焦"智能农机"；② 优先覆盖**履带式底盘**与 **ROS 系统**这两条主线；③ 全部为本地已下载的开放获取全文，可直接打开阅读（文件在 `literature/pdf/`）。
> 年份与卷期以检索数据库记录为准，每篇均给出 DOI 供复核。

---

## 一、选文逻辑

一台 ROS 履带式智能农机的技术栈可以拆成四层，本清单按这四层配比：

| 层 | 需要解决的问题 | 入选篇数 |
|---|---|---|
| **A 履带底盘与运动控制** | 履带滑移转向的运动学建模、转向执行、牵引力不确定性 | 5 篇（№1–5） |
| **B ROS 系统与仿真** | 节点划分、仿真验证、多机协同、数字孪生 | 3 篇（№6–8） |
| **C 定位与建图** | GNSS/RTK 与 LiDAR/SLAM 的选型与融合 | 1 篇（№9） |
| **D 路径跟踪控制** | 控制器选型（Pure Pursuit / Stanley / MPC / 滑模） | 1 篇（№10） |

> **先说一个诚实的结论**：本地库 26 篇履带类文献中，**同时命中"履带 + ROS + 农业"三个条件的只有 3 篇**，其中 2 篇属于感知/规划侧。也就是说，**"ROS 履带式农机"目前没有一篇能一站式覆盖的标杆文献**——这正是本清单必须"组合覆盖"的原因，也说明该方向仍有明确的选题空间。

---

## 二、十篇核心文献

### №1 ★ 最贴合本课题的一篇

**Liu F., Wu L., Wu Z., Cai G., Wang M., et al. (2026). _Simulation-Based Multi-Factor Noise-Aware Adaptive Pure Pursuit with Causal EKF-SG Pose Preprocessing for Tracked Agricultural Robots_. Sensors.**
DOI: [10.3390/s26175673](https://doi.org/10.3390/s26175673) ｜ 本地：`literature/pdf/L0103_F_2026_Simulation-Based_Multi-Factor_Noise-Aware_Adaptive_Pure_P.pdf`

- **为什么重要**：这是库里唯一一篇**同时**做"履带式农业机器人 + 自适应纯跟踪 + EKF 位姿预处理"的工作，与你"ROS 履带车"的定位几乎重合。它把"多因素噪声感知"显式建模进前视距离自适应律，而不是用固定前视距离——履带车在农田软土上横向滑移大，这一点尤其关键。
- **怎么用**：直接对照本项目 `src/control/pure_pursuit.mjs` 的自适应前视距离实现（`L_d = k_v·v + L_min`，再按曲率收缩）；论文的 EKF-SG 位姿预处理可以接到 `src/perception/gnss_fusion.mjs` 的输出之后作为二级滤波。

### №2 履带农机自动驾驶控制的性能评估范式

**Han J.-h., Park C.-h., Kwon J.H., Lee J., Kim T.S., Jang Y.Y. (2020). _Performance Evaluation of Autonomous Driving Control Algorithm for a Crawler-Type Agricultural Vehicle Based on Low-Cost Multi-Sensor Fusion Positioning_. Applied Sciences, 10(13):4667.**
DOI: [10.3390/app10134667](https://doi.org/10.3390/app10134667) ｜ 本地：`literature/pdf/L1630_Han_2020_Performance_Evaluation_of_Autonomous_Driving_Control_Al.pdf`

- **为什么重要**：**低成本**多传感器融合定位 + 履带车控制算法的完整性能评估，给出了横向误差、航向误差的实测口径与试验设计。对"预算有限的履带农机"选题，它的传感器配置（低端 GNSS/INS）比高端方案更有参考价值。
- **怎么用**：把它的评价指标体系（直线段/曲线段分开统计、稳态误差与超调）套到你的 ROS 履带车试验方案上；本项目 `src/perception/gnss_fusion.mjs` 的 RTK 固定解/浮点解分段统计即参考此类范式。

### №3 履带式农机"导航 + 作业"一体化设计

**Yue B., Zhang Z., Zhang W., Luo X., Zhang G., Huang H., et al. (2024). _Design of an Automatic Navigation and Operation System for a Crawler-Based Orchard Sprayer Using GNSS Positioning_. Agronomy, 14(2):271.**
DOI: [10.3390/agronomy14020271](https://doi.org/10.3390/agronomy14020271) ｜ 本地：`literature/pdf/L0798_Yue_2024_Design_of_an_Automatic_Navigation_and_Operation_System_.pdf`

- **为什么重要**：国产团队（华南农大罗锡文团队方向）的**履带式果园喷药机**整机方案，覆盖底盘、GNSS 导航、喷头作业控制与系统集成。果园是履带底盘最有优势的场景（窄行距、坡地、需要低接地比压），这篇给出了完整的机电一体化设计链路。
- **怎么用**：作为你 ROS 履带车的**整机需求与接口设计蓝本**；其作业控制（行进-喷施联动）对应本项目 `src/planning/coverage_path.mjs` 输出的"连续作业/关喷头"两种工况区分。

### №4 履带拖拉机 + LiDAR 的自主导航

**Guo H., Li Y., Wang H., Wang T., Rong L., Wang H., et al. (2024). _Research on autonomous navigation system of greenhouse electric crawler tractor based on LiDAR_. Frontiers in Plant Science, 15:1377269.**
DOI: [10.3389/fpls.2024.1377269](https://doi.org/10.3389/fpls.2024.1377269) ｜ 本地：`literature/pdf/L0831_Guo_2024_Research_on_autonomous_navigation_system_of_greenhouse_.pdf`

- **为什么重要**：**温室**场景下 GNSS 不可用，改用 **LiDAR** 做履带拖拉机导航。这正好回答 ROS 履带车的一个关键问题：**无 GNSS 环境（大棚、果园树冠下）如何退化为激光/视觉导航**。
- **怎么用**：与 №9 的定位建图综述配合，决定你的 ROS 节点里 `gnss_fusion` 与 `lidar_slam`（建议直接用 `slam_toolbox` / `LIO-SAM`）的切换与融合策略。

### №5 静液压传动履带拖拉机的多模态转向控制

**Wang L., Fu W., Wang H., Meng Z., Wei Y., Qin W., et al. (2026). _Autonomous driving system for single hydrostatic transmission crawler tractor based on multi-modal steering control_. Frontiers in Plant Science.**
DOI: [10.3389/fpls.2026.1887292](https://doi.org/10.3389/fpls.2026.1887292) ｜ 本地：`literature/pdf/L0101_L_2026_Autonomous_driving_system_for_single_hydrostatic_transmis.pdf`

- **为什么重要**：**单静液压传动（HST）履带底盘**的转向控制——这是国产履带农机最主流的传动形式。论文的"多模态转向"（差速转向 / 单边制动转向的组合）直接对应履带车最难的执行层问题：**低速大半径与小半径原地转向的平滑切换**。
- **怎么用**：你的 ROS 履带车如果要接真实底盘，这篇的执行层建模（静液压泵排量 ↔ 履带线速度）是绕不开的参考；本项目 `src/control/pure_pursuit.mjs` 目前的转向执行模型是通用一阶滞后 + 速率限制，接实车时需按此替换为履带差速模型。

### №6 ROS 农业机器人的仿真与建模框架

**Tsolakis N., Bechtsis D., Bochtis D. (2019). _AgROS: A Robot Operating System Based Emulation Tool for Agricultural Robotics_. Agronomy, 9(7):403.**
DOI: [10.3390/agronomy9070403](https://doi.org/10.3390/agronomy9070403) ｜ 本地：`literature/pdf/L1732_Tsolakis_2019_AgROS_A_Robot_Operating_System_Based_Emulation_Too.pdf`

- **为什么重要**：**专门为农业机器人做的 ROS 仿真/建模工具**，是本次清单里 ROS 侧最核心的一篇（被引 101）。它解决的是"农机 ROS 仿真里地块、作物、机具怎么建模"的问题——通用 Gazebo 世界文件无法表达农艺约束。
- **怎么用**：作为你 ROS 履带车仿真环境的设计依据；与本项目 `src/sim/field_simulator.mjs`（地块多边形、作物行、传感器噪声与丢星注入）的思路可直接对照，便于把纯 JS 原型迁移到 ROS/Gazebo。

### №7 ROS + ArduPilot + EKF 的真实落地案例

**Zhu X., Zhao X., Liu J., Feng W., Fan X. (2025). _Autonomous Navigation and Obstacle Avoidance for Orchard Spraying Robots: A Sensor-Fusion Approach with ArduPilot, ROS, and EKF_. Agronomy, 15(6):1373.**
DOI: [10.3390/agronomy15061373](https://doi.org/10.3390/agronomy15061373) ｜ 本地：`literature/pdf/L0463_Zhu_2025_Autonomous_Navigation_and_Obstacle_Avoidance_for_Orchar.pdf`

- **为什么重要**：**同时命中"履带 + ROS + 农业"的三篇之一**，而且是唯一给出完整工程栈（ArduPilot 做底层飞控/车控 + ROS 做上层感知决策 + EKF 做状态估计）的论文。对"ROS 履带车"这种**分层架构**（底层实时控制与上层 ROS 解耦）是最直接的模板。
- **怎么用**：直接借用它的分层：ArduPilot/PX4 跑 100 Hz 姿态与速度环，ROS 跑导航与避障；本项目 `src/cloud/telemetry.mjs` 的 CAN/J1939 链路可作为该架构中"车控 ↔ 云"的补充。

### №8 基于 Autoware 的农机仿真与数字孪生

**Zhao X.Y., Wang W., Wen L., Chen Z., Wu S., Zhou K., et al. (2023). _Digital twins in smart farming: An autoware-based simulator for autonomous agricultural vehicles_. International Journal of Agricultural and Biological Engineering, 16(4).**
DOI: [10.25165/j.ijabe.20231604.8039](https://doi.org/10.25165/j.ijabe.20231604.8039) ｜ 本地：`literature/pdf/L1073_Zhao_2023_Digital_twins_in_smart_farming_An_autoware-based_simul.pdf`

- **为什么重要**：把**自动驾驶汽车领域的 Autoware 栈迁移到农机**，并做成数字孪生仿真器。它证明了一件事：**ROS 生态的自动驾驶中间件（Autoware/Nav2）可以直接复用**，不必从零搭农机专用的规划控制栈——这能大幅压缩你的开发周期。
- **怎么用**：如果你打算用 ROS 2 + Nav2，这篇是"如何改造成农机适用"的直接参考；与本项目 `src/planning/coverage_path.mjs`（农艺全覆盖）互补——Nav2 的通用规划器不解决覆盖率与地头转弯。

### №9 农业与林业机器人定位建图综述（选型依据）

**Aguiar A.S., dos Santos F.N., Boaventura-Cunha J., Sobreira H., Sousa A.J. (2020). _Localization and Mapping for Robots in Agriculture and Forestry: A Survey_. Robotics, 9(4):97.**
DOI: [10.3390/robotics9040097](https://doi.org/10.3390/robotics9040097) ｜ 本地：`literature/pdf/L1595_Aguiar_2020_Localization_and_Mapping_for_Robots_in_Agriculture_a.pdf`

- **为什么重要**：被引 137，系统梳理了农业/林业场景下 GNSS、视觉、LiDAR、UWB 等定位手段的适用边界与失效条件。果园树冠遮挡、温室无 GNSS、坡地多路径——这些正是履带车会遇到的场景。
- **怎么用**：作为你做 **ROS 传感器选型与融合架构决策**的第一篇；本项目 `src/perception/gnss_fusion.mjs` 的"RTK 固定解 / 浮点解 / 丢星航位推算"三态降级设计即对应此文所述的失效模式。

### №10 农机路径跟踪控制的系统性综述（控制选型依据）

**Sun C., Sun J., Ding S., Li Q., Ma L. (2025). _Path Tracking Control in Autonomous Agricultural Vehicles: A Systematic Survey of Models, Methods, and Challenges_. Agriculture, 15(23):2522.**
DOI: [10.3390/agriculture15232522](https://doi.org/10.3390/agriculture15232522) ｜ 本地：`literature/pdf/L0430_Sun_2025_Path_Tracking_Control_in_Autonomous_Agricultural_Vehicl.pdf`

- **为什么重要**：2025 年最新的路径跟踪控制系统综述，把 Pure Pursuit、Stanley、LQR、MPC、滑模等方法在**农机工况**（低速、大惯性、滑移、地头频繁转向）下的适用性做了横向比较。选控制器前读这一篇，能避免走弯路。
- **怎么用**：本项目 `src/control/pure_pursuit.mjs` 选 Pure Pursuit 的理由应当由这篇支撑；若要升级到 MPC/Stanley，此文给出参数整定与失效场景的直接参考。（另可对照 `L1031` 与 `L1265`，见下节。）

---

## 三、建议阅读路线（约 4 周）

| 阶段 | 目标 | 阅读 |
|---|---|---|
| 第 1 周 | 建立履带底盘与整机概念 | №3 → №2 → №5 |
| 第 2 周 | 确定 ROS 架构与仿真方案 | №6 → №8 → №7 |
| 第 3 周 | 定位/感知方案选型 | №9 → №4（无 GNSS 退化） |
| 第 4 周 | 控制算法落地 | №10 → №1（履带专用纯跟踪 + EKF） |

**先读顺序的例外**：如果你已确定要用 ROS 2 + 真实履带底盘，建议**第 1 篇就读 №1**，它离你的目标最近，能最快建立"履带车控制到底难在哪"的直觉。

---

## 四、额外推荐（本库有全文，按需补读）

| 文献 | 本地文件 | 适用场景 |
|---|---|---|
| Lv P. 等 (2022). _Multi-Objective Association Detection of Farmland Obstacles Based on Information Fusion of Millimeter Wave Radar and Camera_. Sensors. DOI: 10.3390/s23010230 | `literature/pdf/L1318_Lv_2022_Multi-Objective_Association_Detection_of_Farmland_Obstac.pdf` | **履带∧ROS 三篇命中之一**；履带车低矮障碍物感知，毫米波+视觉融合 |
| Wang Q. 等 (2023). _Modelling and Control Methods in Path Tracking Control for Autonomous Agricultural Vehicles: A Review_. Applied Sciences. DOI: 10.3390/app13127155 | `literature/pdf/L1031_Wang_2023_Modelling_and_Control_Methods_in_Path_Tracking_Control.pdf` | 与 №10 互补的路径跟踪综述，含建模方法对比 |
| Wang L. 等 (2022). _Path Tracking Control of an Autonomous Tractor Using Improved Stanley Controller Optimized with Multiple-Population Genetic Algorithm_. Actuators. DOI: 10.3390/act11010022 | `literature/pdf/L1265_Wang_2022_Path_Tracking_Control_of_an_Autonomous_Tractor_Using_I.pdf` | 若要从 Pure Pursuit 升级到 Stanley 控制器 |
| Fue K.G. 等 (2020). _Autonomous Navigation of a Center-Articulated and Hydrostatic Transmission Rover using a Modified Pure Pursuit Algorithm in a Cotton Field_. Sensors. DOI: 10.3390/s20164412 | `literature/pdf/L1633_Fue_2020_Autonomous_Navigation_of_a_Center-Articulated_and_Hydro.pdf` | 静液压传动 + 改进纯跟踪的田间实测（轮式铰接，可与 №5 履带对照） |
| Wei J. 等 (2026). _Complete Coverage Path Planning for Agricultural Sowing Machine Using Improved Biological Neural Network_. Agriculture. DOI: 10.3390/agriculture16181968 | `literature/pdf/L0275_Wei_2026_Complete_Coverage_Path_Planning_for_Agricultural_Sowing.pdf` | 全覆盖路径规划的神经网络方法，对照本项目牛耕式方案 |

## 五、补充文献的获取状态

| 文献 | DOI | 状态 | 说明 |
|---|---|---|---|
| Kayacan E. 等 (2018). _High-precision control of tracked field robots in the presence of unknown traction coefficients_. **J. Field Robotics**. | [10.1002/rob.21794](https://doi.org/10.1002/rob.21794) | **已获取** | 出版商版闭源，已取 arXiv 预印本（2103.11294，同作者同标题）：pdf/L1817_Kayacan_2018_High_precision_control_of_tracked_field_r.pdf |
| Jiang Y. 等 (2025). _Research Review of Agricultural Machinery Power Chassis in Hilly and Mountainous Areas_. **Agriculture**. | [10.3390/agriculture15111158](https://doi.org/10.3390/agriculture15111158) | **已获取** | MDPI 开放获取 |
| Gentilini L. 等 (2021). _Trajectory Planning ROS Service for an Autonomous Agricultural Robot_. IEEE MetroAgriFor. | [10.1109/metroagrifor52389.2021.9628620](https://doi.org/10.1109/metroagrifor52389.2021.9628620) | **未获取** | Unpaywall / OpenAlex 均标记 closed，无开放版本 |
| Tang Z. 等 (2026). _From compaction to coexistence: chassis engineering review_. **Science Progress**. | [10.1177/00368504261420946](https://doi.org/10.1177/00368504261420946) | **未获取** | 名义 gold OA，但 SAGE 与 PMC 对本网络均返回 403 |

> 截至本次更新：核心 10 篇 **10/10 全部本地**；补充 9 篇中 7 篇本地、2 篇确认无法通过合法开放渠道获取。

---

## 六、与本地项目的对应关系

| 文献 | 对应本项目模块 |
|---|---|
| №1, №10 | `src/control/pure_pursuit.mjs` |
| №2, №4, №9 | `src/perception/gnss_fusion.mjs` |
| №3 | `src/planning/coverage_path.mjs`（作业工况区分） |
| №6, №8 | `src/sim/field_simulator.mjs`（迁移到 ROS/Gazebo 的蓝本） |
| №7 | 分层架构：底层车控 + ROS 上层；配合 `src/cloud/telemetry.mjs` |

> 说明：本项目 `src/` 是**零依赖的 Node.js 算法原型**，用于验证算法与指标口径；迁移到 ROS 2 时，上述模块是逐一对位的移植源。
