# 智能农机无人作业系统（Intelligent Agricultural Machinery）· 软件原型

面向"无人农场 / 智能农机"课题的可运行软件原型：从**感知定位 → 全覆盖路径规划 → 多机协同调度 → 闭环跟踪控制 → 云端遥测运维**的完整闭环，
全部算法均为真实实现（无占位、无伪代码），端到端主线一条命令跑通并输出结构化结果与中文作业报告。

- 运行环境：**Node.js ≥ 18（实测 v24.19.0）**，**零第三方依赖**（仅使用 Node 内置模块）
- 全部源码为 ESM（`.mjs`），无 Python、无需编译、无原生依赖
- 端到端主线：`node src/cli.mjs`　自检：`node src/selftest.mjs`（103 项断言全绿）
- 配套仓库：[`weed-vision-prescription`](https://github.com/jhy1238/weed-vision-prescription) —— 视觉除草模块的独立版本（可单独复现 IoU 与省药率）
- 指标溯源：[`docs/12_指标复现与数据溯源.md`](docs/12_指标复现与数据溯源.md) —— 报告里每个数字对应哪条命令、哪条断言、哪个源文件

---

## 1. 目录结构

```text
smart-agri-machinery/
├─ package.json                 # name/version/type=module/scripts(start, test)
├─ README.md                    # 本文件
├─ src/
│  ├─ cli.mjs                   # 端到端主线（生成地块→规划→调度→跟踪→视觉→遥测→报告）
│  ├─ selftest.mjs              # 自检（103 项数值断言，失败非零退出）
│  ├─ sim/field_simulator.mjs   # 地块/作物行/农机库/传感器/合成农田影像 仿真环境
│  ├─ perception/
│  │  ├─ gnss_fusion.mjs        # GNSS/RTK + IMU 扩展卡尔曼滤波（EKF）定位融合
│  │  └─ vision_weed.mjs        # ExG 植被分割 + 连通域标记 + 作物/杂草分类 + 密度栅格与处方图
│  ├─ planning/
│  │  ├─ coverage_path.mjs      # 牛耕式全覆盖 + 地头 Π/Ω 转弯（Dubins）+ 重叠率/覆盖率栅格化
│  │  └─ dispatch.mjs           # 多机协同调度（贪心构造 + 局部搜索）与朴素基线对比
│  ├─ control/pure_pursuit.mjs  # 纯跟踪控制 + 曲率/转向速率速度规划 + 闭环仿真
│  └─ cloud/telemetry.mjs       # J1939/ISOBUS 编解码 + 边缘网关 + 云端时序库 + 异常检测告警
├─ docs/                        # 12 篇中文工程文档（见第 5 节索引）
├─ data/                        # 运行产物：run_result.json（结构化）、run_report.txt（文本报告）
└─ .github/workflows/ci.yml     # CI：每次推送跑 103 项自检（Node 18 / 20 / 22）
```

> 仓库只保留可运行的算法与文档：文献全文库（3 GB+）、采集脚本、运行日志均不入库，
> 原因是体积、版权与凭据安全。文献检索方法与统计见 `docs/09_文献综述与检索报告.md`。

---

## 2. 快速开始

```bash
git clone https://github.com/jhy1238/smart-agri-machinery.git
cd smart-agri-machinery

# 端到端主线（约 6~10 s，输出中文作业报告并写 data/run_result.json + data/run_report.txt）
node src/cli.mjs

# 快速模式（缩短遥测时长、单一样区）
node src/cli.mjs --quick --seed=20240501

# 自检（103 项断言，全部通过时退出码 0）
node src/selftest.mjs
```

> 无需 `npm install`：项目零第三方依赖。Windows / Linux / macOS 均可直接运行。

### 2.1 `node src\cli.mjs` 真实输出（节选，完整见 `data/run_report.txt`）

```text
==============================================================================
        智能农机无人作业系统 · 端到端仿真运行报告
==============================================================================
随机种子: 20240501    运行模式: 完整    Node.js: v24.19.0

【一】地块与作物
F1  东风农场 1 号大田   大田  5.40 ha  玉米  播种      F6  设施园艺示范棚区  设施  0.96 ha  番茄  耕整
F2  东风农场 2 号 L 形地 大田  3.00 ha  大豆  播种      F7  东岗 7 号麦田     大田  4.76 ha  小麦  收获
F3  北岭不规则地块      大田  3.64 ha  小麦  植保      F8  三里屯 8 号大田   大田  3.08 ha  玉米  播种
F4  南坡果园            果园  2.40 ha  苹果  植保      F9  北坡梨园         果园  1.98 ha  梨    植保
F5  西河湾 5 号大田     大田  3.90 ha  水稻  收获      F10 南洼 10 号地      大田  2.60 ha  大豆  耕整
合计作业面积: 31.72 公顷 (475.7 亩)；作物行理论株数合计 4,115,366 株

【二】全覆盖路径规划（牛耕式 + 地头转弯）
地块  农机              幅宽   作业方向 行程数 作业里程km 转弯里程m 转弯Π/Ω  覆盖率%  重叠率%  时长h
F1    东方红 LX2204     4.5m   0°       40     13.11      1246      38/3     99.86    9.81     1.83
F2    东方红 LX2204     4.5m   0°       35     7.23       851       33/3     99.65    10.04    1.03
F3    大疆 T60 植保无人  6.0m   6°       34     6.41       579       47/0     99.27    9.85     0.52
F5    雷沃谷神 GM100    5.0m   0°       29     8.50       737       27/3     99.81    9.98     1.61
F7    雷沃谷神 GM100    5.0m   0°       33     10.28      834       31/3     99.85    9.05     1.94
平均覆盖率 99.72%    平均重叠率(实测) 9.92%    平均非作业里程占比 8.97%

【三】多机协同调度（贪心构造 + 局部搜索 vs 朴素轮询基线）
农机  机型                  作业地块序列               班次(h) 作业(h) 成本(元)
M2    东方红 LX2204 无人     T1 → T2 → T6 → T8 → T10    5.82    5.16    931
M3    雷沃谷神 GM100 联合    T5 → T7                     5.97    3.56    1314
M4    大疆 T60 植保无人机    T3 → T9 → T4                4.39    1.09    791
指标                优化方案   朴素基线   改进
目标函数 J          10.40      11.79      -11.7%
完工时间            6.32 h     7.48 h     -15.5%
空驶里程            6.87 km    7.71 km    -10.9%
局部搜索改进: 9.37%（共 4 次改进移动：Or-opt 迁移 / 机间交换 / 机内 2-opt）

【四】路径跟踪闭环仿真（纯跟踪控制 + EKF 融合定位）
被控对象: 东方红 LX2204 无人驾驶拖拉机   轴距 2.60 m   最大前轮转角 35°   转向速率上限 25°/s
仿真路段: 东风农场 1 号大田   长度 473.2 m   仿真时长 219.5 s   行驶里程 472.9 m
横向跟踪误差: RMS = 2.15 cm   平均绝对误差 = 0.88 cm   最大 = 15.56 cm
航向误差 RMS = 0.626°   平均车速 = 2.15 m/s   最高车速 = 2.20 m/s

【五】视觉杂草识别（ExG = 2G - R - B，连通域特征分类）
样区  真值密度株/m² 检出簇数 精确率  召回率  F1     行间召回 行内召回 分割IoU
W2    0.80          38       1.000   0.570   0.726  0.687    0.070    0.879
作物行自动检测: 检出 16 行，行距 0.600 m，与真值平均偏差 0.46 cm
植被分割: Otsu 自动阈值 ExG > 61，形态学去噪剔除 4635 px，分割 IoU 0.877
变量喷药处方: 变量作业总药量 0.021 L，均一喷施 0.039 L，节省 45.9%

【六】GNSS/RTK + IMU 扩展卡尔曼融合定位
定位误差: 全程 RMS = 1.98 cm，最大 10.31 cm
  · RTK 固定解区间 RMS = 0.68 cm      · 丢星区间（航位推算）最大 3.03 cm
航向误差 RMS = 0.068°    GNSS 观测接受率 = 98.59%
NIS 卡方门控: 检出并剔除野值 8 个（注入多路径野值 3 个）
丢星降级: 检测到连续丢星 6.2 s，>2.0 s 门限即进入航位推算并膨胀过程噪声 4 倍

【七】农机物联网遥测与异常检测
链路: 边缘网关 5403 次采样 -> 编码 CAN 报文 32418 帧 -> 上行 1068 批（丢包 13 批 / 390 帧）-> 云端时序点 48042 个
报警统计: 共 72 条（严重 6 条，警告 66 条），故障检出率 100.0%，平均检出时延 2.3 s
M1  节温器卡滞导致冷却液温度持续爬升   检出 是  首报 727s  时延 7s   报警 25
M2  发动机转速异常尖峰                检出 是  首报 1080s 时延 0s   报警 17
M3  燃油消耗率突降至近零              检出 是  首报 1500s 时延 0s   报警 8

【八】关键指标达成情况
平均覆盖率        99.72 %   > 98 %      ✔   横向跟踪误差 RMS  2.15 cm   < 10 cm  ✔
平均重叠率(实测)  9.92 %    5 % ~ 15 %  ✔   RTK 固定解定位 RMS 0.68 cm  < 5 cm   ✔
调度目标函数改进  11.7 %    > 10 %      ✔   杂草识别 F1        0.726    > 0.6    ✔
遥测故障检出率    100 %     = 100 %     ✔   平均故障检出时延   2.3 s    < 60 s   ✔
```

### 2.2 运行产物

| 文件 | 内容 |
|---|---|
| `data/run_result.json` | 结构化全量结果（约 400 KiB）：地块几何、路径采样、调度甘特、跟踪曲线、识别统计与处方图、融合精度、遥测报警 |
| `data/run_report.txt` | 与上面等价的纯文本中文作业报告（便于归档/打印/diff） |

---

## 3. 模块说明

| 模块 | 关键算法 | 主要导出 |
|---|---|---|
| `src/sim/field_simulator.mjs` | 地块多边形（矩形/L 形/凸多边形）、作物行生成、农机装备库、GNSS/IMU 传感器仿真（固定解/浮点解/丢星/多路径野值）、合成农田影像（行播作物 + 随机杂草 + 像素级真值） | `createRng` `makeRectangleField` `makeLShapeField` `makeConvexPolygonField` `generateCropRows` `buildLoopTruthTrack` `simulateGnssTrack` `simulateImu` `makeSyntheticFieldImage` `MACHINE_TYPES` `buildFleet` |
| `src/perception/gnss_fusion.mjs` | 5 维状态 EKF（x, y, θ, v, ω）+ 多速率观测（RTK 位置 / 双天线定向 / 轮速 / 陀螺）；NIS 卡方门控剔除野值；连续丢星 >2 s 进入航位推算并膨胀过程噪声；观测质量突变自适应膨胀协方差 | `ExtendedKalmanFilter` `GnssImuFusion` `runFusionScenario` `wrapAngle` `matInverse` `CHI2_99` |
| `src/perception/vision_weed.mjs` | ExG = 2G − R − B + Otsu 自动阈值分割；3×3 形态学开闭运算去噪；两遍扫描 + 并查集连通域标记；行投影峰值法自动检测作物行；按"行带外像素占比 + 面积 + 长宽比"分类作物/杂草；1 m 密度栅格与变量喷药处方图 | `exgIndex` `otsuThreshold` `segmentVegetation` `morphOpen` `morphClose` `connectedComponents` `estimateCropRows` `classifyComponents` `weedDensityGrid` `prescriptionMap` `runWeedDetection` |
| `src/planning/coverage_path.mjs` | 多边形等距内缩（地头预留）；牛耕式扫描线分解 + 作业方向自动寻优；Dubins 六词最短路径（CSC/CCC 精确构造）；Π 形平头转弯闭式几何与 Ω 形灯泡转弯；跳行作业策略；0.25 m 栅格化覆盖率与重叠率 | `CoveragePlanner` `offsetPolygon` `boustrophedonPath` `dubinsPath` `cccCandidates` `flatTurn` `bulbTurn` `skipRowOrder` `turnCostModel` `rasterizeCoverage` `filletLoop` |
| `src/planning/dispatch.mjs` | 带时间窗与机具能力约束的并行机调度；构造式贪心（J 增量最小插入）+ VND 局部搜索（Or-opt 迁移 / 机间交换 / 机内 2-opt）；目标函数 J = w1·完工 + w2·空驶 + w3·等待 + w4·成本 + w5·超窗；与朴素轮询基线对比 | `DispatchOptimizer` `baselineNaiveAssignment` `evaluateSchedule` `extractGantt` `objectiveFunction`（权重常量）` |
| `src/control/pure_pursuit.mjs` | 纯跟踪 δ = atan2(2L·sinα, L_d)；自适应前视距离（速度增益 + 曲率收缩）；速度规划（曲率限速 + 转向速率限速 + 地头转弯限速 + 双向加减速斜坡）；转向执行一阶滞后与速率限制；闭环仿真（控制器仅使用 EKF 融合位姿） | `PurePursuitController` `planSpeedProfile` `simulateTracking` `crossTrackError` `resamplePath` `pathCurvature` |
| `src/cloud/telemetry.mjs` | J1939/ISO 11783 29 位 CAN 标识符与 SPN 编解码（冷却液温度/转速/燃油率/车速/电压/专有报文）、DM1 故障码；边缘网关批量上行（丢包+时延）；云端内存时序库；异常检测（绝对阈值 + 60 s 滑动窗口 3σ + EWMA 控制图 λ=0.2 + 变化率）与作业状态门控去抖 | `encodeCanFrame` `decodeCanFrame` `encodeDm1` `decodeDm1` `encodeDtc` `EdgeTelemetryGateway` `TelemetryCloud` `AnomalyDetector` `runTelemetryScenario` |
| `src/cli.mjs` | 端到端主线：10 地块 × 4 农机，串起上述全部模块，打印中文作业报告并落盘 JSON + TXT | — |
| `src/selftest.mjs` | 103 项数值断言（几何/Dubins/内缩/规划/调度/跟踪/视觉/融合/遥测/产物结构） | — |

---

## 4. 关键指标（`node src\cli.mjs` 实测）

| 指标 | 实测值 | 判据 | 结论 |
|---|---|---|---|
| 平均覆盖率（0.25 m 栅格） | 99.72 % | > 98 % | ✔ |
| 相邻行程重叠率（栅格） | 9.92 %（设定 8 %） | 5 % ~ 15 % | ✔ |
| 横向跟踪误差 RMS | 2.15 cm（最大 15.56 cm） | < 10 cm | ✔ |
| RTK 固定解定位 RMS | 0.68 cm（全程 1.98 cm） | < 5 cm | ✔ |
| 调度目标函数 J 改进 | 11.7 %（完工时间 −15.5 %，空驶 −10.9 %） | > 10 % | ✔ |
| 杂草识别精确率 / 召回率 / F1 | 1.000 / 0.570 / 0.726 | F1 > 0.6 | ✔ |
| 遥测故障检出率 / 时延 | 100 % / 2.3 s | = 100 %；< 60 s | ✔ |
| 自检断言 | 103 / 103 通过 | 全绿 | ✔ |

---

## 5. 文档索引（`docs/`）

| 文档 | 内容 |
|---|---|
| `00_项目总览与执行摘要.md` | 目标、量化指标、交付物清单、技术路线图、里程碑、风险 |
| `01_需求与场景分析.md` | 大田/果园/设施三类场景、痛点、功能(FR)与非功能(NFR)需求、KPI、竞品对比 |
| `02_总体架构设计.md` | 端-边-云三层架构、模块划分、数据流、接口与 JSON 示例、时序图、部署视图 |
| `03_感知与定位子系统.md` | 北斗/GNSS 双天线定向、IMU、视觉 ExG、LiDAR、EKF 推导、标定、精度指标 |
| `04_决策与规划子系统.md` | 全覆盖路径规划、地头转弯、机群调度优化、处方图与 AI 模型选型、复杂度与实测指标 |
| `05_控制与执行子系统.md` | 纯跟踪推导与整定、速度规划、MPC 对比、电液转向、CAN/ISOBUS、安全冗余与降级 |
| `06_云平台与数据子系统.md` | 遥测链路、PGN/SPN 表、数据模型与建表 SQL、边缘-云协同、告警、数字孪生 |
| `07_实施路线与效益分析.md` | 三期实施路线、硬件选型与成本表、单亩成本、投资回收期、敏感性分析与风险 |
| `08_安全合规与标准.md` | ISO 25119 / ISO 18497 / ISO 11783 / GB/T 35381 等合规要求与检查清单 |
| `09_文献综述与检索报告.md` | 检索策略（7 个数据库）、去重与相关性过滤规则、研究现状综述（含 26 篇核验过的精选文献）、研究空白与项目对应关系、全文获取与合规说明 |
| `10_核心文献十篇_ROS履带式智能农机.md` | 10 篇核心文献的逐篇精读要点（方法、实验、可复用结论） |
| `11_候选筛选报告_ROS履带式农机.md` | 候选文献筛选过程与判据（为何保留/剔除） |
| `12_指标复现与数据溯源.md` | 报告指标 → 命令 → 断言 → 源文件的对照表，用于核对结论可复现性 |

---

## 6. 文献综述工作（未随仓库分发）

本课题另完成了智能农机方向的文献检索与综述：7 个数据库（OpenAlex / Crossref / Europe PMC / PubMed /
arXiv / Semantic Scholar / CNKI）检索，去重后 4,491 条，相关性过滤后 1,958 条，其中 CNKI 中文题录 749 条；
检索策略、筛选判据、研究现状与研究空白分析见 `docs/09`～`docs/11`。

全文库（开放获取 PDF，3 GB+）与采集脚本因体积、版权与凭据安全**不入库**。
`src/` 下所有模块与文献库完全解耦，不读写任何文献数据——克隆本仓库即可独立运行全部算法。

---

## 7. 实现约定与边界

1. **零依赖**：仅使用 `node:fs` `node:path` `node:url`，无 npm 依赖、无原生编译，任意平台可跑。
2. **可复现**：所有随机过程使用固定种子的 mulberry32 随机数发生器，`--seed=<n>` 可切换场景。
3. **不做"作弊式"真值利用**：视觉模块的作物行中心线由影像自身检测（行投影峰值法），
   真值标签仅用于评估精确率/召回率；行内杂草与作物行带粘连，单目 RGB 下本不可分，
   原型如实统计其漏检（行间召回 ≈ 0.63、行内召回 ≈ 0.12），并说明需多光谱或深度学习进一步区分。
4. **物理约束入模**：转向速率上限 25°/s、最小转弯半径 4.0/5.0/5.5 m、横向加速度上限 1.5 m/s²、
   地头转弯限速 1.6 m/s，均参与速度规划与转弯代价计算，而非仅作注释。
5. **未实现部分**（文档中已明确标注为"文献量级/设计值"）：Stanley / LQR / MPC 控制器、
   LiDAR 点云处理、深度学习杂草模型、真实 CAN 硬件驱动与云端微服务部署。
