# 候选筛选报告 · ROS 履带式智能农机

**筛选口径**：四条硬约束必须同时满足（履带底盘 / ROS 或 ROS2 / GPS-GNSS 含 RTK + IMU + 视觉 / 农业田间作业）。
**取证方法**：不依赖摘要关键词，而是对本地 PDF 做**全文文本抽取**（`scripts/pdf_text.mjs`，FlateDecode 解压 + Tj/TJ 文本算子），统计术语出现次数，再对边界样本**逐条上下文人工核验**。仓库类用 GitHub API 校验 `pushed_at` 与最近 commit。

> **关键词计数会被"引言/相关工作/参考文献"污染**：例如某温室履带拖拉机论文中 GNSS 出现 10 次，但全部在引言综述里，系统本身并无 GNSS。故下表的关键数值均以人工核验后的判定为准。

---

## 结论先行

在 **19 个候选**（15 篇文献 + 4 个仓库）中：

| 判定 | 数量 | 条目 |
|---|---|---|
| **四条硬约束全部满足** | **1** | L0463 |
| 满足 3 条（差分 1 条） | 4 | L1298、L1630、L0798、L0619 |
| 满足 2 条 | 4 | L0831、L0101、L1318、L1496 |
| 满足 ≤1 条 | 6 | L0103、L1817、L1633、L0534、L1619、L1039 + 4 仓库 |

**核心判断**：**"履带 + ROS + GNSS/IMU/视觉三融合 + 农业"这个组合目前没有成熟范式**——不是我没找到，而是这个交叉点本身是空的。L0463 是唯一实证存在的完整样本，且它仍有明显局限（非 RTK、ROS 1、改装小底盘）。这对你的项目既是风险也是机会。

---

## T1 · 全通过（唯一）

### 1. L0463 — Autonomous Navigation and Obstacle Avoidance for Orchard Spraying Robots: A Sensor-Fusion Approach with ArduPilot, ROS, and EKF

- **来源**：Zhu X., Zhao X., Liu J., Feng W., Fan X. *Agronomy* 2025, 15(6):1373
- **链接**：https://doi.org/10.3390/agronomy15061373
- **本地全文**：`literature/pdf/L0463_Zhu_2025_Autonomous_Navigation_and_Obstacle_Avoidance_for_Orchar.pdf`（23 页）

**2. 硬约束逐项**

| 约束 | 判定 | 证据 |
|---|---|---|
| 履带式底盘 | ✅ | 3WDZ-200D 遥控履带喷药机改装，"tracked chassis"、"rotate in place" |
| ROS / ROS2 | ✅ | Ubuntu 20.04 + **ROS Noetic**，Raspberry Pi 4B 上跑感知与决策节点 |
| GPS/GNSS + IMU + 视觉 | ✅ | **GPS**（Quectel EC200A）+ **IMU** + **T265 视觉里程计**（双目鱼眼），EKF 三者融合 |
| 农业田间作业 | ✅ | 果园（orchard）植保喷施，河北农业大学，果园实地试验 |

**3. 与"GPS+IMU+视觉融合"的关联点**
论文的核心创新就是把 GPS、T265 视觉里程计、IMU 三路数据送进**同一个 EKF**，专门解决果园两类典型失效：GNSS 被树冠遮挡、纯视觉在弱光下退化。这正是一条"三融合"的完整工程实现。

**4. 对你的项目的直接可用价值**
- **架构**：ArduPilot（底层 100 Hz 姿态/速度环）+ ROS（上层导航与避障）的分层解耦，是履带车最稳妥的落地形态。
- **算法**：EKF 观测模型的构造方式（三路异构观测如何定权、如何门控）可直接对照本项目 `src/perception/gnss_fusion.mjs`。
- **工程经验**：它把"遥控履带机 → 自主履带机"的改造路径写得很细（舵机/油门信号如何被 ArduPilot 接管），这是接实车时最省时间的一段。

**5. 局限性**
- 用的是**普通 GPS 而非 RTK**，定位精度上限受限（论文未给厘米级指标）。
- **ROS 1（Noetic）**，非 ROS 2；迁移需重写 launch 与部分消息接口。
- 底盘是 200 L 药箱的小型履带喷药机，**载荷与农具接口**与你可能的作业机具不同。
- 视觉仅作里程计（VO），**不做作物行识别或语义感知**；若你要行间对准，需另加视觉检测。

---

## T2 · 差一条（按可用价值排序）

### 2. L1298 — RoboNav: An Affordable Yet Highly Accurate Navigation System for Autonomous Agricultural Robots

- *Robotics* 2022, 11(5):99 ｜ https://doi.org/10.3390/robotics11050099
- 本地：`literature/pdf/L1298_Galati_2022_RoboNav_An_Affordable_Yet_Highly_Accurate_Navigation.pdf`

| 履带 | ROS | GNSS+IMU+视觉 | 农业 |
|---|---|---|---|
| ✅ 全地形履带、橡胶履带、滑移转向 | ✅ 但**用法很浅** | GNSS✅ IMU✅ **视觉❌** | ✅ 商业葡萄园实测 |

**3. 关联点**：GPS + IMU 经 EKF/滤波融合输出航向与位置，**但系统里没有相机**——视觉只在相关工作里被讨论（"front-facing camera"用于行检测是他人方案）。
**4. 价值**：**低成本 RTK/GPS + IMU 的标定与误差分解做得极扎实**，葡萄园实测横向误差约 0.2 m；对"预算敏感的履带车"是最好的基线参照。滑移转向的电机混控逻辑可直接借鉴。
**5. 局限**：无视觉；ROS 仅用于取传感器数据，核心算法是 C++ 底层（想学 ROS 架构会失望）；定位精度 0.2 m 量级，不满足行间精细作业。

### 3. L1630 — Performance Evaluation of Autonomous Driving Control Algorithm for a Crawler-Type Agricultural Vehicle Based on Low-Cost Multi-Sensor Fusion Positioning

- *Applied Sciences* 2020, 10(13):4667 ｜ https://doi.org/10.3390/app10134667

| 履带 | ROS | GNSS+IMU+视觉 | 农业 |
|---|---|---|---|
| ✅ 履带式农机 | ❌ 全文 0 次 | GNSS✅ IMU✅ 视觉❌ | ✅ 田间 |

**3. 关联点**：低成本 GNSS/INS 组合定位 + 履带车路径跟踪，**无视觉**。
**4. 价值**：**履带车自动驾驶的评测方法论**——直线段/曲线段分开统计、稳态误差与超调的定义，可直接搬进你的试验大纲。
**5. 局限**：无 ROS、无视觉；传感器是低端 GNSS/INS，精度有限。

### 4. L0798 — Design of an Automatic Navigation and Operation System for a Crawler-Based Orchard Sprayer Using GNSS Positioning

- *Agronomy* 2024, 14(2):271 ｜ https://doi.org/10.3390/agronomy14020271

| 履带 | ROS | GNSS+IMU+视觉 | 农业 |
|---|---|---|---|
| ✅ 履带果园喷药机 | ❌ | GNSS✅ IMU✅(MTi-30) 视觉❌ | ✅ 果园 |

**3. 关联点**：GNSS + MTi-30 惯性传感器做姿态与位置，**无视觉融合**。
**4. 价值**：国产整机方案（华南农大团队方向），**导航与喷施作业的联动控制**（行进-喷头开关）写得很完整，是"导航如何接农艺"的范本。
**5. 局限**：无 ROS、无视觉；GNSS 单点+惯导，树冠下会退化。

### 5. L0619 — Research on EV Crawler-Type Soil Sample Robot Using GNSS Information

- *Sensors* 2025, 25(3):604 ｜ https://doi.org/10.3390/s25030604

| 履带 | ROS | GNSS+IMU+视觉 | 农业 |
|---|---|---|---|
| ✅ 电动履带取样机器人 | ❌ 全文 0 次 | GNSS✅ IMU✅ 视觉⚠️(弱) | ✅ 田间取样 |

**3. 关联点**：GNSS 主导定位 + IMU 姿态补偿；视觉出现 6 次但非融合主体。
**4. 价值**：**履带底盘 + 电动化 + 定点作业**的完整机电方案，适合作为"小型履带作业机"的参考。
**5. 局限**：无 ROS；定位以 GNSS 为主，视觉不参与融合；作业类型（取土）与你可能不同。

---

## T3 · 差两条及以上（仅作背景与零件来源）

### 6. L0831 — Research on autonomous navigation system of greenhouse electric crawler tractor based on LiDAR
*Frontiers in Plant Science* 2024, 15:1377269 ｜ https://doi.org/10.3389/fpls.2024.1377269

| 履带 | ROS | GNSS+IMU+视觉 | 农业 |
|---|---|---|---|
| ✅ 电动履带拖拉机 | ✅ **真 ROS**（Kinetic + Gmapping） | **GNSS❌** IMU✅ **视觉❌** | ✅ 温室 |

**3. 关联点**：系统实为 **LiDAR + IMU + 轮速**的 ROS 建图导航；GNSS 与视觉的 10/11 次出现**全部在引言综述**，系统本身没有。
**4. 价值**：**"无 GNSS 环境如何用 ROS 跑履带车"的最佳样本**——ROS 节点划分、Gmapping 建图、USB 传感器接入都写得很实。
**5. 局限**：无 GNSS、无视觉融合（这恰是硬约束要的）；温室场景，田间适应性未知。

>  methodological 提醒：这篇是"关键词计数 6/6 全过、人工核验后只过 2 条"的典型，说明必须读上下文。

### 7. L0101 — Autonomous driving system for single hydrostatic transmission crawler tractor based on multi-modal steering control
*Frontiers in Plant Science* 2026 ｜ https://doi.org/10.3389/fpls.2026.1887292

| 履带 | ROS | GNSS+IMU+视觉 | 农业 |
|---|---|---|---|
| ✅ 单 HST 履带拖拉机 | ❌（仅参考文献出现 1 次） | GNSS✅ IMU❌ 视觉❌ | ✅ |

**3. 关联点**：GNSS 导航 + 多模态转向控制，无 IMU 融合、无视觉。
**4. 价值**：**履带差速/单边制动转向的执行层建模**（静液压泵排量 ↔ 履带线速度）——接实车时绕不开。
**5. 局限**：无 ROS、无视觉；以转向控制为主，感知很薄。

### 8. L0103 — Simulation-Based Multi-Factor Noise-Aware Adaptive Pure Pursuit with Causal EKF-SG Pose Preprocessing for Tracked Agricultural Robots
*Sensors* 2026 ｜ https://doi.org/10.3390/s26175673

| 履带 | ROS | GNSS+IMU+视觉 | 农业 |
|---|---|---|---|
| ✅ 履带农业机器人 | ❌ | GNSS❌ IMU❌ 视觉❌（**纯仿真**） | ✅ |

**3. 关联点**：**没有真实传感器**，是多因素噪声注入的仿真研究；EKF-SG 是位姿预处理算法而非多传感器融合。
**4. 价值**：**履带机器人自适应纯跟踪的噪声敏感性分析**，对你的控制器鲁棒性设计与仿真试验设计很有用。
**5. 局限**：纯仿真、无 ROS、无真实传感器——只能当算法参考，不能当系统参考。

### 9. L1318 — Multi-Objective Association Detection of Farmland Obstacles Based on Information Fusion of Millimeter Wave Radar and Camera
*Sensors* 2023, 23(1):230 ｜ https://doi.org/10.3390/s23010230

| 履带 | ROS | GNSS+IMU+视觉 | 农业 |
|---|---|---|---|
| ⚠️（拖拉机平台，含履带讨论） | ✅ Ubuntu 16.04 + ROS + Rviz | **GNSS❌ IMU❌** 视觉✅ | ✅ 农田障碍 |

**3. 关联点**：毫米波雷达 + 相机的前融合，**不含 GNSS/IMU**。
**4. 价值**：ROS 环境下的**雷达-相机外参标定与融合检测**工程细节完整，可直接补你履带车的低矮障碍感知短板。
**5. 局限**：缺 GNSS/IMU；感知专用，不含定位与导航闭环。

### 10. L1496 — Trajectory Planning ROS Service for an Autonomous Agricultural Robot
*IEEE MetroAgriFor* 2021 ｜ https://doi.org/10.1109/metroagrifor52389.2021.9628620

| 履带 | ROS | GNSS+IMU+视觉 | 农业 |
|---|---|---|---|
| ⚠️ 未明 | ✅ | GNSS✅ IMU❌ 视觉❌ | ✅ |

**3. 关联点**：GNSS 提供全局位姿，**无 IMU/视觉融合**。
**4. 价值**：把轨迹规划封装成 **ROS Service** 的接口范式，架构层面可抄。
**5. 局限**：本库无全文（仅有题录），且传感器组合不全。

### 11. L1817 — High-precision control of tracked field robots in the presence of unknown traction coefficients
*Journal of Field Robotics* 2018 ｜ https://doi.org/10.1002/rob.21794

| 履带 | ROS | GNSS+IMU+视觉 | 农业 |
|---|---|---|---|
| ✅ 履带田间机器人 | ❌ | ❌ | ✅ |

**3. 关联点**：纯控制理论（自适应鲁棒控制），**不涉及传感器融合**。
**4. 价值**：**履带滑移/牵引系数未知**下的高精度控制——这是履带车区别于轮式车的核心难点，理论价值高（被引 49）。
**5. 局限**：无 ROS、无融合、无视觉；本库无全文（Wiley 订阅），需机构账号补下。

---

## T4 · 若把"履带"放宽为"可迁移的农机移动平台"

> 这三篇**只差"履带"一条**，但因为含完整的 ROS + GNSS + IMU + 视觉 四件套，架构迁移价值可能高于 T2 的多数条目。

### 12. L1633 — Autonomous Navigation of a Center-Articulated and Hydrostatic Transmission Rover using a Modified Pure Pursuit Algorithm in a Cotton Field
*Sensors* 2020, 20(16):4412 ｜ https://doi.org/10.3390/s20164412

| 履带 | ROS | GNSS+IMU+视觉 | 农业 |
|---|---|---|---|
| ❌ 铰接轮式 | ✅ | GNSS✅ IMU✅ 视觉⚠️ | ✅ 棉田实测 |

**3. 关联点**：GNSS + IMU 融合定位，改进纯跟踪；视觉不是融合主体。
**4. 价值**：**静液压传动 + 铰接底盘的纯跟踪整定**，与履带车的低速大惯性特性最接近，可直接对照 `src/control/pure_pursuit.mjs`。
**5. 局限**：铰接轮式而非履带，转向几何不同。

### 13. L0534 — Development of an Orchard Inspection Robot: A ROS-Based LiDAR-SLAM System with Hybrid A*-DWA Navigation
*Sensors* 2025, 25(21):6662 ｜ https://doi.org/10.3390/s25216662

| 履带 | ROS | GNSS+IMU+视觉 | 农业 |
|---|---|---|---|
| ❌ 轮式 | ✅ **ROS 系统完整** | LiDAR-SLAM 主导，GNSS/视觉弱 | ✅ 果园 |

**3. 关联点**：以 LiDAR-SLAM 为主，**GNSS + 视觉融合不是重点**。
**4. 价值**：**ROS 下 Hybrid A* + DWA 的完整果园导航栈**，节点组织与参数配置可直接迁移。
**5. 局限**：传感器组合不满足硬约束；轮式平台。

### 14. L1619 — Development of a Multi-Purpose Autonomous Differential Drive Mobile Robot for Plant Phenotyping and Soil Sensing
*Electronics* 2020, 9(9):1550 ｜ https://doi.org/10.3390/electronics9091550

| 履带 | ROS | GNSS+IMU+视觉 | 农业 |
|---|---|---|---|
| ❌ 差速轮式 | ✅ | GNSS✅ IMU✅ 视觉✅ | ✅ 表型/土壤 |

**3. 关联点**：GPS + IMU + 相机（表型成像）同车共存，但**融合深度有限**（视觉更多用于成像而非定位）。
**4. 价值**：**多用途农机模块化设计**（可换传感器载荷）的工程组织方式。
**5. 局限**：差速轮式；视觉未深度参与定位融合。

---

## T5 · GitHub 仓库（按你的排除项核查）

| 仓库 | ★ | 最近 commit | 硬约束判定 | 结论 |
|---|---|---|---|---|
| [javadibrahimli/tomato_agribot_ros2](https://github.com/javadibrahimli/tomato_agribot_ros2) | 17 | 2026-01-08 | 履带❌ 差速轮式 / ROS2✅ / GPS✅ IMU? 视觉⚠️(ArUco) / 农业✅ | **排除**：平台不符，且无 IMU+视觉融合定位 |
| [mertcetinturk/autonomous-orchard-robot](https://github.com/mertcetinturk/autonomous-orchard-robot) | 7 | 2026-10-18 | 履带❌ 未声明 / ROS2✅ / 传感器未声明 / 农业✅(仿真) | **排除**：仅仿真展示，无代码主体（lang=null，105 MB 资源文件） |
| [Fontainebleau2021/Agri_ROS](https://github.com/Fontainebleau2021/Agri_ROS) | 5 | 2026-11-06 | 履带❌ / ROS✅ / 定位+SLAM+感知 / 农业✅ | **观察**：定位/SLAM/感知框架，但无履带与 GNSS+IMU+视觉 三融合证据 |
| [nicolapace/agromatic](https://github.com/nicolapace/agromatic) | 11 | 2025-04-08（>1.5 年） | 履带❌ / ROS✅ / 施肥仿真 / 农业✅ | **排除**：平台不符 + 更新停滞 |
| [ros-agriculture/ros-a-site.github.io](https://github.com/ros-agriculture/ros-a-site.github.io) | 15 | 2020-03-25（>2 年） | — | **排除**：仅为网站，无实质代码 |
| [kartik-nighania/ROS_Based_Farm_Robot](https://github.com/kartik-nighania/ROS_Based_Farm_Robot) | 11 | 2018-03-20（>2 年） | — | **排除**：长期未更新 |
| [MrCerealKiller/agros_paths](https://github.com/MrCerealKiller/agros_paths) | 8 | 2020-02-28（>2 年） | — | **排除**：长期未更新 |

**仓库侧结论**：GitHub 上没有满足"履带 + ROS + GNSS/IMU/视觉 + 农业"四条件的活跃仓库。最接近的农业 ROS 仓库都是**差速轮式**，且多数只做单一模态（GPS 或 SLAM）。

---

## 排序建议（软偏好加权后）

| 排名 | 条目 | 软偏好命中 | 建议用法 |
|---|---|---|---|
| 1 | **L0463** | 融合✅ 田间✅ 履带控制✅ 近年✅ | **主参考**：整体架构直接对标 |
| 2 | L1298 | 融合(部分)✅ 田间✅ 履带控制✅ 近年✅ | 低成本 GNSS/IMU 标定与误差分解 |
| 3 | L1630 | 田间✅ 履带控制✅ | 评测方法论 / 试验大纲 |
| 4 | L0798 | 田间✅ 履带✅ 近年✅ | 导航↔作业联动 |
| 5 | L0831 | ROS✅ 履带✅ 田间✅ 近年✅ | **无 GNSS 场景的 ROS 退化方案**（补短板用） |
| 6 | L0619 | 履带✅ 近年✅ | 小型电动履带底盘机电方案 |
| 7 | L0103 | 履带控制✅ 近年✅ | 控制器鲁棒性 / 仿真设计 |
| 8 | L1318 | ROS✅ 近年✅ | 雷达-相机融合感知模块 |

---

## 给你的三条判断

1. **不要试图找"五合一"的现成方案**——它不存在。合理路线是：**L0463 做架构主干**（它已经解决了 ArduPilot/ROS 分层与三融合 EKF），**L1630 + L1298 做定位与评测**，**L0831 做无 GNSS 退化**，**L0103 + L1633 做控制器整定**。

2. **最可能的真实短板是"视觉"而不是"ROS"**。清单里 ROS 做得好的（L0831、L1318、L0534）几乎都不做视觉-惯性-卫星三融合；做三融合的（L0463）视觉只当里程计。**你的差异化空间在"视觉用于农艺任务"（作物行/垄线/障碍）与"视觉用于定位"的结合**。

3. **RTK 是明确空白**。L0463 用普通 GPS，L1298 是 0.2 m 级。若你做 RTK + IMU + 视觉，在**精度指标**上就有可发表的增量；这也是本项目 `src/perception/gnss_fusion.mjs` 已经把 RTK 固定解/浮点解/丢星三态做进去的原因。

---

## 附：被排除的典型（供你复核口径）

| 条目 | 排除理由 |
|---|---|
| L1496 | 无全文且传感器组合不全 |
| L1817 | 无 ROS、无融合（但控制理论价值高，建议补下） |
| L0420（丘陵山地底盘综述） | 纯综述，无导航/定位实现 |
| L0430 / L1031（路径跟踪综述） | 综述，无具体平台与传感器组合 |
| L1039（AgROS 类） | ROS 框架/资源调度，非履带平台 |
| 纯视觉杂草识别类（大量） | 单传感器，不涉及融合与平台 |

> 说明：本报告的"硬约束判定"基于论文全文的自述；未做复现实验验证。论文未明确写出的传感器，一律按"未使用"处理（保守判定）。
