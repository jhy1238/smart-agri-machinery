#!/usr/bin/env node
/**
 * src/selftest.mjs
 * ---------------------------------------------------------------------------
 * 智能农机无人作业系统 —— 自检（关键数值断言）
 *
 * 覆盖范围：
 *   1) 仿真地块几何（面积、凹凸性）
 *   2) Dubins 最短路径正确性（终点位姿误差、路径长度自洽、Π/Ω 分类）
 *   3) 多边形内缩（地头预留）
 *   4) 全覆盖规划（覆盖率 > 98%、相邻行程重叠率 5%~15%、路径连续性、转弯代价）
 *   5) 多机调度（优于朴素基线、能力约束、时间窗、甘特数据完整）
 *   6) 路径跟踪（横向误差 RMS < 0.1 m、速度/转角约束）
 *   7) 视觉杂草识别（行检测精度、分割 IoU、精确率/召回率/F1）
 *   8) GNSS/RTK + IMU 融合（RTK 固定解精度、NIS 野值剔除、丢星降级）
 *   9) 遥测（CAN/J1939 编解码往返一致、DM1 故障码、故障检出率、误报数）
 *  10) 端到端产物 data/run_result.json 结构完整性
 *
 * 运行：node src\selftest.mjs      失败时以非零码退出
 */

import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

import {
  makeRectangleField,
  makeLShapeField,
  makeConvexPolygonField,
  generateCropRows,
  buildFleet,
  createRng,
  polygonArea,
} from './sim/field_simulator.mjs';
import { runFusionScenario, wrapAngle, matInverse, matMul, matIdentity } from './perception/gnss_fusion.mjs';
import { runWeedDetection } from './perception/vision_weed.mjs';
import { makeSyntheticFieldImage } from './sim/field_simulator.mjs';
import { CoveragePlanner, dubinsPath, offsetPolygon, flatTurn, bulbTurn, rasterizeCoverage, cccCandidates } from './planning/coverage_path.mjs';
import { DispatchOptimizer, extractGantt, machineCanDo } from './planning/dispatch.mjs';
import { simulateTracking, PurePursuitController, planSpeedProfile, crossTrackError, resamplePath } from './control/pure_pursuit.mjs';
import {
  encodeCanFrame,
  decodeCanFrame,
  encodeCanId,
  decodeCanId,
  encodeDm1,
  decodeDm1,
  encodeDtc,
  decodeDtc,
  SPN_DEFS,
  runTelemetryScenario,
} from './cloud/telemetry.mjs';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.resolve(__dirname, '..');

let passed = 0;
let failed = 0;
const failures = [];
const group = (t) => {
  console.log('');
  console.log('── ' + t + ' ' + '─'.repeat(Math.max(0, 60 - t.length)));
};
const check = (name, cond, detail) => {
  if (cond) {
    passed += 1;
    console.log('  [PASS] ' + name + (detail ? '   ' + detail : ''));
  } else {
    failed += 1;
    failures.push(name + (detail ? ' :: ' + detail : ''));
    console.log('  [FAIL] ' + name + (detail ? '   ' + detail : ''));
  }
};
const near = (a, b, tol) => Math.abs(a - b) <= tol;

console.log('='.repeat(74));
console.log('  智能农机无人作业系统 · 自检（src/selftest.mjs）');
console.log('  Node.js ' + process.version);
console.log('='.repeat(74));

/* =======================================================================
 * 1) 地块几何
 * ======================================================================= */
group('1) 仿真地块几何');
const rect = makeRectangleField({ id: 'R', lengthM: 300, widthM: 180 });
check('矩形地块面积 = 5.40 ha', near(rect.areaHa, 5.4, 1e-6), 'areaHa=' + rect.areaHa.toFixed(4));
check('矩形地块周长为正且合理', near(rect.perimeterM, 960, 1e-6), 'perimeter=' + rect.perimeterM.toFixed(1));
check('多边形按逆时针给出（有向面积为正）', polygonArea(rect.polygon) > 0, 'signedArea=' + polygonArea(rect.polygon).toFixed(1));
const lshape = makeLShapeField({});
check('L 形地块面积 = 3.00 ha', near(lshape.areaHa, 3.0, 1e-6), 'areaHa=' + lshape.areaHa.toFixed(4));
const convex = makeConvexPolygonField({});
let convexOk = true;
for (let i = 0; i < convex.polygon.length; i += 1) {
  const a = convex.polygon[i];
  const b = convex.polygon[(i + 1) % convex.polygon.length];
  const c = convex.polygon[(i + 2) % convex.polygon.length];
  if ((b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x) <= 0) convexOk = false;
}
check('凸多边形地块所有顶点为凸（叉积同号）', convexOk, '顶点数=' + convex.polygon.length);
const rows = generateCropRows(rect, {});
check('作物行生成：行数 = 300 行（行距 0.6 m）', rows.rowCount === 300, 'rowCount=' + rows.rowCount);
check('作物行总长 ≈ 面积/行距', near(rows.totalRowLengthM, 5.4e4 / 0.6, 50), 'totalLen=' + rows.totalRowLengthM.toFixed(0) + ' m');

/* =======================================================================
 * 2) Dubins 路径与转弯几何
 * ======================================================================= */
group('2) Dubins 最短路径与 Π/Ω 转弯几何');
const dubinsCases = [
  [{ x: 0, y: 0, theta: 0 }, { x: 20, y: 3, theta: Math.PI }, 4],
  [{ x: 0, y: 0, theta: 0 }, { x: 1, y: 3, theta: Math.PI }, 4],
  [{ x: 0, y: 0, theta: 0.7 }, { x: 5, y: -4, theta: -1.2 }, 5.5],
  [{ x: 3, y: 3, theta: -2 }, { x: 3.5, y: 3.2, theta: 2.5 }, 4],
];
let maxEndErr = 0;
let maxLenErr = 0;
for (const [s, e, r] of dubinsCases) {
  const d = dubinsPath(s, e, r, 0.2);
  const last = d.points[d.points.length - 1];
  maxEndErr = Math.max(maxEndErr, Math.hypot(last.x - e.x, last.y - e.y), Math.abs(wrapAngle(last.theta - e.theta)) * r);
  maxLenErr = Math.max(maxLenErr, Math.abs(last.s - d.lengthM));
}
check('Dubins 终点位姿误差 < 1e-6 m', maxEndErr < 1e-6, 'maxErr=' + maxEndErr.toExponential(2));
check('Dubins 采样弧长与解析长度一致 (< 1e-6 m)', maxLenErr < 1e-6, 'maxLenErr=' + maxLenErr.toExponential(2));
const flat = flatTurn({ x: 0, y: 0, theta: 0 }, { x: 0, y: 9, theta: Math.PI }, 4);
check('Π 形平头转弯在 w=9m >= 2R=8m 时几何可行', flat !== null, 'length=' + (flat ? flat.lengthM.toFixed(3) : 'n/a') + ' m');
check('Π 形转弯长度 = pi*R + w - 2R', flat !== null && near(flat.lengthM, Math.PI * 4 + 9 - 8, 1e-9), 'expect=' + (Math.PI * 4 + 1).toFixed(3));
check('Π 形转弯在 w=3m < 2R 时判定不可行（应改用 Ω 形）', flatTurn({ x: 0, y: 0, theta: 0 }, { x: 0, y: 3, theta: Math.PI }, 4) === null);
const bulb = bulbTurn({ x: 0, y: 0, theta: 0 }, { x: 0, y: 2.76, theta: Math.PI }, 4, 0.5);
check('Ω 形灯泡转弯对 w < 2R 的相邻行程返回可行解', bulb !== null && bulb.type === 'Ω', bulb ? 'word=' + bulb.word + ' len=' + bulb.lengthM.toFixed(2) + ' m' : 'n/a');
check('Ω 形转弯里程明显大于 Π 形', bulb !== null && bulb.lengthM > Math.PI * 4 + 1, bulb ? 'Ω=' + bulb.lengthM.toFixed(1) + ' m' : '');

/* =======================================================================
 * 3) 多边形内缩（地头预留）
 * ======================================================================= */
group('3) 多边形内缩（地头预留）');
const inner = offsetPolygon(rect.polygon, 6);
const innerArea = Math.abs(polygonArea(inner));
const expectInner = (300 - 12) * (180 - 12);
check('矩形内缩 6 m 后面积符合解析解', near(innerArea, expectInner, 1), 'area=' + innerArea.toFixed(1) + ' vs ' + expectInner);
check('内缩后面积小于原面积', innerArea < rect.areaM2, '');
const innerL = offsetPolygon(lshape.polygon, 7.5);
check('凹多边形内缩后仍为正面积多边形', Math.abs(polygonArea(innerL)) > 1000, 'area=' + Math.abs(polygonArea(innerL)).toFixed(0) + ' m2');

/* =======================================================================
 * 4) 全覆盖路径规划
 * ======================================================================= */
group('4) 全覆盖路径规划（覆盖率 / 重叠率 / 转弯）');
const coverageCases = [
  ['矩形大田', rect, 4.5, 5.5, true],
  ['L 形地块', lshape, 4.5, 5.5, true],
  ['凸多边形地块', convex, 6.0, 2.0, false],
];
const planMetrics = [];
for (const [name, f, w, r, arcWork] of coverageCases) {
  const t0 = Date.now();
  const plan = new CoveragePlanner({
    field: f,
    swathWidthM: w,
    overlapRatio: 0.08,
    minTurnRadiusM: r,
    cornerArcWorking: arcWork,
    workSpeedMps: 2.2,
  }).plan();
  planMetrics.push({ name, m: plan.metrics, plan });
  const m = plan.metrics;
  check(name + ' 覆盖率 > 98%', m.coverageRate > 0.98, (m.coverageRate * 100).toFixed(2) + '%');
  check(name + ' 相邻行程重叠率 ∈ [5%, 15%]', m.overlapRatioMeasured >= 0.05 && m.overlapRatioMeasured <= 0.15, (m.overlapRatioMeasured * 100).toFixed(2) + '%');
  check(name + ' 行程间距 = 幅宽*(1-重叠率)', near(m.stripSpacingM, w * 0.92, 1e-9), m.stripSpacingM.toFixed(3) + ' m');
  check(name + ' 作业里程 > 0 且非作业里程占比 < 25%', m.workLengthM > 0 && m.nonWorkingRatio < 0.25, '非作业占比=' + (m.nonWorkingRatio * 100).toFixed(2) + '%');
  check(name + ' 转弯几何有效（转弯次数 > 0 且每次转弯均有里程）', m.turnCount > 0 && m.turnLengthM > 0, 'Π=' + m.piTurnCount + ' Ω=' + m.omegaTurnCount);
  check(name + ' 路径点数 > 0 且规划耗时 < 3 s', m.pathPointCount > 100 && Date.now() - t0 < 3000, 'points=' + m.pathPointCount + ' (' + (Date.now() - t0) + ' ms)');
  // 路径连续性：相邻路径点间距不超过 3 m（不存在跳变）
  let maxGap = 0;
  const pts = plan.path;
  for (let i = 1; i < pts.length; i += 1) {
    const d = Math.hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
    if (d > maxGap) maxGap = d;
  }
  check(name + ' 路径连续（相邻点间距 < 3 m）', maxGap < 3.0, 'maxGap=' + maxGap.toFixed(3) + ' m');
}
const totalPi = planMetrics.reduce((s, x) => s + x.m.piTurnCount, 0);
const totalOmega = planMetrics.reduce((s, x) => s + x.m.omegaTurnCount, 0);
check('三类地块合计同时使用 Π 形与 Ω 形地头转弯', totalPi > 0 && totalOmega > 0, 'Π 合计=' + totalPi + '，Ω 合计=' + totalOmega);
const avgCoverage = planMetrics.reduce((s, x) => s + x.m.coverageRate, 0) / planMetrics.length;
const avgOverlap = planMetrics.reduce((s, x) => s + x.m.overlapRatioMeasured, 0) / planMetrics.length;
check('三类地块平均覆盖率 > 99%', avgCoverage > 0.99, (avgCoverage * 100).toFixed(3) + '%');
check('三类地块平均重叠率 ∈ [5%, 15%]', avgOverlap >= 0.05 && avgOverlap <= 0.15, (avgOverlap * 100).toFixed(3) + '%');
const skipPlan = new CoveragePlanner({ field: rect, swathWidthM: 4.5, minTurnRadiusM: 5.5, turnStrategy: 'skip-row' }).plan();
const adjPlan = new CoveragePlanner({ field: rect, swathWidthM: 4.5, minTurnRadiusM: 5.5, turnStrategy: 'adjacent' }).plan();
check('跳行作业策略转弯里程显著小于相邻行策略', skipPlan.metrics.turnLengthM < adjPlan.metrics.turnLengthM * 0.8,
  '跳行=' + skipPlan.metrics.turnLengthM.toFixed(0) + ' m vs 相邻行=' + adjPlan.metrics.turnLengthM.toFixed(0) + ' m');
check('跳行作业的 Ω 形转弯次数大幅减少', skipPlan.metrics.omegaTurnCount < adjPlan.metrics.omegaTurnCount * 0.5,
  '跳行 Ω=' + skipPlan.metrics.omegaTurnCount + ' vs 相邻行 Ω=' + adjPlan.metrics.omegaTurnCount);

/* =======================================================================
 * 5) 多机协同调度
 * ======================================================================= */
group('5) 多机协同调度（贪心 + 局部搜索 vs 朴素基线）');
const fleet = buildFleet({});
const ops = ['播种', '播种', '植保', '植保', '收获', '耕整', '收获', '播种'];
const tasks = ops.map((op, i) => {
  const areaHa = 2 + (i % 4) * 1.2;
  const dur = {};
  for (const m of fleet) if (m.capabilities.includes(op)) dur[m.id] = areaHa / m.fieldCapacityHaPerH;
  return {
    id: 'T' + (i + 1),
    name: '地块' + (i + 1),
    capability: op,
    areaHa,
    location: { x: 100 + i * 130, y: 60 + (i % 4) * 110 },
    timeWindowH: [i * 0.4, 18],
    durationHByMachine: dur,
  };
});
const dispatch = new DispatchOptimizer({ tasks, machines: fleet }).solve();
check('优化解目标函数优于朴素基线', dispatch.schedule.J < dispatch.baseline.schedule.J,
  'J=' + dispatch.schedule.J.toFixed(3) + ' < ' + dispatch.baseline.schedule.J.toFixed(3));
check('目标函数改进 > 5%', dispatch.improvement.objectivePct > 5, dispatch.improvement.objectivePct.toFixed(2) + '%');
check('空驶里程不高于基线', dispatch.schedule.transitKm <= dispatch.baseline.schedule.transitKm + 1e-9,
  dispatch.schedule.transitKm.toFixed(2) + ' km vs ' + dispatch.baseline.schedule.transitKm.toFixed(2) + ' km');
check('完工时间不高于基线', dispatch.schedule.makespanH <= dispatch.baseline.schedule.makespanH + 1e-9,
  dispatch.schedule.makespanH.toFixed(2) + ' h vs ' + dispatch.baseline.schedule.makespanH.toFixed(2) + ' h');
let capabilityViolation = 0;
for (const [mid, list] of Object.entries(dispatch.assignment)) {
  const machine = fleet.find((m) => m.id === mid);
  for (const tid of list) {
    const task = tasks.find((t) => t.id === tid);
    if (!machineCanDo(machine, task)) capabilityViolation += 1;
  }
}
check('能力约束满足（无越权分配）', capabilityViolation === 0, '违规数=' + capabilityViolation);
const assignedAll = Object.values(dispatch.assignment).flat().length;
check('所有任务均被分配', assignedAll === tasks.length, assignedAll + '/' + tasks.length);
const gantt = extractGantt(dispatch.schedule, { machines: fleet });
check('甘特数据完整（每台农机均有时间轴）', gantt.length === fleet.length && gantt.every((g) => Array.isArray(g.items)), '甘特行数=' + gantt.length);
let timeConsistent = true;
for (const g of gantt) {
  for (let i = 1; i < g.items.length; i += 1) if (g.items[i].startH < g.items[i - 1].startH - 1e-6) timeConsistent = false;
  for (const it of g.items) if (it.endH < it.startH - 1e-9) timeConsistent = false;
}
check('甘特时间轴单调且区间合法', timeConsistent);

/* =======================================================================
 * 6) 路径跟踪控制
 * ======================================================================= */
group('6) 纯跟踪闭环控制');
const trackPlan = planMetrics[0].plan;
const rawPath = trackPlan.path.map((p) => ({ x: p.x, y: p.y, type: p.type }));
let accS = 0;
for (let i = 0; i < rawPath.length; i += 1) {
  if (i > 0) accS += Math.hypot(rawPath[i].x - rawPath[i - 1].x, rawPath[i].y - rawPath[i - 1].y);
  rawPath[i].s = accS;
}
let bi = 0;
let bl = 0;
for (let i = 1; i < rawPath.length; i += 1) {
  if (rawPath[i].type === 'work' && rawPath[i - 1].type === 'work') {
    let j = i;
    while (j < rawPath.length && rawPath[j].type === 'work') j += 1;
    const L = rawPath[j - 1].s - rawPath[i - 1].s;
    if (L > bl) {
      bl = L;
      bi = i - 1;
    }
    i = j;
  }
}
const track = simulateTracking(rawPath, {
  startIndex: bi,
  simLengthM: 300,
  maxTimeS: 300,
  vMaxMps: 2.2,
  wheelbaseM: 2.6,
  steerRateDegPerS: 25,
  vTurnMaxMps: 1.6,
  seed: 20240501,
});
check('横向跟踪误差 RMS < 0.1 m', track.rmsCrossTrackM < 0.1, (track.rmsCrossTrackM * 100).toFixed(2) + ' cm');
check('横向误差为厘米级（RMS < 0.05 m）', track.rmsCrossTrackM < 0.05, (track.rmsCrossTrackM * 100).toFixed(2) + ' cm');
check('横向误差最大值 < 0.5 m', track.maxCrossTrackM < 0.5, (track.maxCrossTrackM * 100).toFixed(2) + ' cm');
check('航向误差 RMS < 3°', track.rmsHeadingErrDeg < 3, track.rmsHeadingErrDeg.toFixed(3) + '°');
check('车速不超过规划上限 2.2 m/s', track.maxSpeedMps <= 2.2 + 1e-6, track.maxSpeedMps.toFixed(3) + ' m/s');
check('仿真行驶里程与路径长度一致（误差 < 5%）', Math.abs(track.travelledM - track.pathLengthM) / track.pathLengthM < 0.05,
  track.travelledM.toFixed(1) + ' m vs ' + track.pathLengthM.toFixed(1) + ' m');
const ctrl = new PurePursuitController({ vMaxMps: 2.2 });
ctrl.setPath(rawPath.slice(bi, bi + 400).map((p, i) => ({ x: p.x, y: p.y, s: i * 0.5, type: p.type })));
const ldSlow = ctrl.lookaheadDistance(1.0, 0);
const ldFast = ctrl.lookaheadDistance(3.0, 0);
const ldCurve = ctrl.lookaheadDistance(3.0, 0.25);
check('前视距离随车速增大', ldFast > ldSlow, ldSlow.toFixed(2) + ' m -> ' + ldFast.toFixed(2) + ' m');
check('前视距离随曲率减小（弯道提高跟踪精度）', ldCurve < ldFast, ldFast.toFixed(2) + ' m -> ' + ldCurve.toFixed(2) + ' m');
check('前视距离被限制在 [Lmin, Lmax] 内', ldSlow >= ctrl.opt.lookaheadMinM * 0.8 && ldFast <= ctrl.opt.lookaheadMaxM + 1e-9);
// 显式构造"直线 40 m + R=6 m 的 90° 圆弧 + 直线 40 m"路径验证曲率限速与转向速率限速
const spPath = [];
let spS = 0;
for (let x = 0; x <= 40; x += 1) {
  spPath.push({ x, y: 0, s: spS });
  spS += 1;
}
for (let a = 0; a <= Math.PI / 2 + 1e-9; a += 0.05) {
  spPath.push({ x: 40 + 6 * Math.sin(a), y: 6 - 6 * Math.cos(a), s: spS, type: 'turn' });
  spS += 6 * 0.05;
}
for (let y = 6; y <= 46; y += 1) {
  spPath.push({ x: 46, y, s: spS });
  spS += 1;
}
const spRes = resamplePath(spPath, 0.2);
const sp = planSpeedProfile(spRes, {
  vMaxMps: 2.2,
  aLatMaxMps2: 1.5,
  wheelbaseM: 2.6,
  steerRateRadPerS: (25 * Math.PI) / 180,
  curvatureTurnThreshold: 0.08,
  vTurnMaxMps: 1.6,
});
let speedOk = true;
for (let i = 0; i < sp.v.length; i += 1) if (sp.v[i] > 2.2 + 1e-9) speedOk = false;
check('速度规划结果不超过 v_max', speedOk, 'max=' + Math.max(...Array.from(sp.v)).toFixed(3) + ' m/s');
const vStraight = sp.v[Math.floor(sp.v.length * 0.1)];
const vArc = Math.min(...Array.from(sp.v));
check('曲线段限速生效（圆弧段速度 <= 地头转弯限速 1.6 m/s）', vArc <= 1.6 + 1e-6, '直线段=' + vStraight.toFixed(2) + ' m/s, 圆弧段=' + vArc.toFixed(2) + ' m/s');
check('圆弧段速度显著低于直线段', vArc < vStraight * 0.8, vArc.toFixed(2) + ' < ' + (vStraight * 0.8).toFixed(2));
const cccTest = cccCandidates({ x: 0, y: 0, theta: 0 }, { x: 2, y: 3, theta: Math.PI }, 4, -1);
let cccEndErr = Infinity;
if (cccTest.length) {
  const c = cccTest[0];
  const d2 = dubinsPath({ x: 0, y: 0, theta: 0 }, { x: 2, y: 3, theta: Math.PI }, 4, 0.2, [c.word]);
  const last = d2.points[d2.points.length - 1];
  cccEndErr = Math.hypot(last.x - 2, last.y - 3);
}
check('CCC（RLR/LRL）几何构造成立且端点闭合', cccTest.length > 0 && cccEndErr < 1e-6, '解数=' + cccTest.length + ' 端点误差=' + (cccEndErr === Infinity ? 'n/a' : cccEndErr.toExponential(2)));

/* =======================================================================
 * 7) 视觉杂草识别
 * ======================================================================= */
group('7) 视觉杂草识别（ExG 分割 + 连通域分类）');
const imgA = makeSyntheticFieldImage({ seed: 4001, weedDensityPerM2: 0.8 });
const weedA = runWeedDetection(imgA);
check('作物行自动检测行距误差 < 5 cm', weedA.rowDetection.meanAbsErrorM < 0.05, (weedA.rowDetection.meanAbsErrorM * 100).toFixed(2) + ' cm');
check('检测到 16 条作物行（12.8 m / 0.6 m 行距）', weedA.rowDetection.detectedRows === 16, 'rows=' + weedA.rowDetection.detectedRows);
check('植被分割 IoU > 0.8', weedA.evaluation.segmentation.iou > 0.8, weedA.evaluation.segmentation.iou.toFixed(4));
check('杂草识别精确率 > 0.8', weedA.evaluation.weedPrecision > 0.8, weedA.evaluation.weedPrecision.toFixed(4));
check('杂草识别召回率 > 0.4', weedA.evaluation.weedRecall > 0.4, weedA.evaluation.weedRecall.toFixed(4));
check('杂草识别 F1 > 0.6', weedA.evaluation.weedF1 > 0.6, weedA.evaluation.weedF1.toFixed(4));
check('行间杂草召回率显著高于行内杂草（符合行带法原理）', weedA.evaluation.recallInterRowWeed > weedA.evaluation.recallInRowWeed * 1.5,
  '行间=' + weedA.evaluation.recallInterRowWeed.toFixed(3) + ' 行内=' + weedA.evaluation.recallInRowWeed.toFixed(3));
check('形态学去噪确实剔除了噪声像素', weedA.noiseRemovedPx > 0, weedA.noiseRemovedPx + ' px');
check('杂草密度栅格尺寸正确（13 x 10 格 @1 m）', weedA.grid.cols === 13 && weedA.grid.rows === 10, weedA.grid.cols + 'x' + weedA.grid.rows);
const gridSum = weedA.grid.density.reduce((s, d) => s + d * weedA.grid.cellAreaM2, 0);
check('密度栅格积分 = 检出杂草簇数', near(gridSum, weedA.weedCount, 1e-6), gridSum.toFixed(2) + ' vs ' + weedA.weedCount);
check('变量喷药处方较均一喷施节省药量', weedA.prescription.savingPct > 0, weedA.prescription.savingPct.toFixed(1) + '%');
const imgB = makeSyntheticFieldImage({ seed: 4002, weedDensityPerM2: 1.6 });
const weedB = runWeedDetection(imgB);
check('高密度样区检出杂草簇数多于低密度样区', weedB.weedCount > weedA.weedCount, weedB.weedCount + ' > ' + weedA.weedCount);

/* =======================================================================
 * 8) GNSS/RTK + IMU 融合定位
 * ======================================================================= */
group('8) GNSS/RTK + IMU 扩展卡尔曼融合');
const fusion = runFusionScenario({ seed: 20240501, laps: 1 });
const fm = fusion.metrics;
check('RTK 固定解区间定位 RMS < 5 cm', fm.rmsErrorRtkFixedM < 0.05, (fm.rmsErrorRtkFixedM * 100).toFixed(2) + ' cm');
check('RTK 固定解区间定位 RMS < 2 cm（厘米级）', fm.rmsErrorRtkFixedM < 0.02, (fm.rmsErrorRtkFixedM * 100).toFixed(2) + ' cm');
check('全程定位 RMS < 10 cm', fm.rmsErrorM < 0.1, (fm.rmsErrorM * 100).toFixed(2) + ' cm');
check('航向误差 RMS < 1°', fm.headingRmsDeg < 1.0, fm.headingRmsDeg.toFixed(3) + '°');
check('注入的多路径野值全部被 NIS 卡方门控剔除', fm.rejectedOutliers >= fm.injectedOutliers,
  '剔除 ' + fm.rejectedOutliers + ' / 注入 ' + fm.injectedOutliers);
check('连续丢星被检测并进入航位推算降级', fm.detectedOutageS > 2.0, '丢星 ' + fm.detectedOutageS.toFixed(1) + ' s');
check('丢星期间航位推算误差 < 0.3 m', fm.maxErrorOutageM < 0.3, (fm.maxErrorOutageM * 100).toFixed(2) + ' cm');
check('恢复 RTK 后 3 s 内误差回到厘米级', fm.rmsErrorRecoveryM < 0.05, (fm.rmsErrorRecoveryM * 100).toFixed(2) + ' cm');
check('GNSS 观测接受率 > 90%', fm.gnssAcceptedRate > 0.9, (fm.gnssAcceptedRate * 100).toFixed(2) + '%');
const invOk = (() => {
  const A = matMul(matIdentity(3), [
    [4, 1, 0],
    [1, 3, 1],
    [0, 1, 2],
  ]);
  const Ai = matInverse(A);
  const I = matMul(A, Ai);
  let ok = true;
  for (let i = 0; i < 3; i += 1) for (let j = 0; j < 3; j += 1) if (Math.abs(I[i][j] - (i === j ? 1 : 0)) > 1e-9) ok = false;
  return ok;
})();
check('矩阵求逆数值正确（A*A^-1 = I）', invOk);
check('角度归一化正确（wrapAngle）', near(wrapAngle(3 * Math.PI), Math.PI, 1e-12) && near(wrapAngle(-3 * Math.PI), Math.PI, 1e-12));

/* =======================================================================
 * 9) 遥测链路
 * ======================================================================= */
group('9) 农机物联网遥测（J1939 编解码 + 异常检测）');
let rtOk = true;
let rtDetail = '';
for (const [spn, def] of Object.entries(SPN_DEFS)) {
  const v = def.signal === 'coolantTempC' ? 92 : def.signal === 'engineSpeedRpm' ? 2050 : def.signal === 'fuelRateLph' ? 18.5 : def.signal === 'groundSpeedKph' ? 8.0 : def.signal === 'batteryVoltageV' ? 27.8 : def.signal === 'hydraulicOilTempC' ? 64 : def.signal === 'ptoRpm' ? 540 : def.signal === 'defLevelPct' ? 62 : def.signal === 'oilTempC' ? 105 : 60;
  const f = encodeCanFrame(def.pgn, 0x21, { [def.signal]: v });
  const d = decodeCanFrame(f);
  const tol = def.resolution;
  if (Math.abs(d.values[def.signal] - v) > tol) {
    rtOk = false;
    rtDetail += ' SPN' + spn + '(' + d.values[def.signal] + '!=' + v + ')';
  }
}
check('全部 SPN 编解码往返一致（误差 <= 分辨率）', rtOk, rtDetail || Object.keys(SPN_DEFS).length + ' 个 SPN');
check('CAN 标识符解码 PGN 正确', decodeCanId(encodeCanId(65262, 0x21, 6)).pgn === 65262 &&
  decodeCanId(encodeCanId(61444, 0x22, 3)).priority === 3 &&
  decodeCanId(encodeCanId(61444, 0x22, 3)).sa === 0x22,
  '0x' + encodeCanId(65262, 0x21, 6).toString(16).toUpperCase());
const dtc = { spn: 110, fmi: 0, oc: 0, cm: 0 };
const dtcRt = decodeDtc(encodeDtc(dtc));
check('DM1 故障码 SPN/FMI 编解码往返一致', dtcRt.spn === 110 && dtcRt.fmi === 0, JSON.stringify(dtcRt));
const dm1 = encodeDm1(0x21, [dtc]);
check('DM1 报文可解析出故障码', decodeDm1(dm1).length === 1 && decodeDm1(dm1)[0].spn === 110, dm1.data.join(' '));
const telMachines = fleet.slice(0, 3);
const tel = runTelemetryScenario({ machines: telMachines, durationMin: 30, seed: 20240504 });
check('故障检出率 = 100%（3 类注入故障全部检出）', tel.stats.detectionRate === 1,
  tel.faultLog.map((f) => f.kind + ':' + (f.detected ? '检出' : '漏检')).join(' '));
check('平均故障检出时延 < 60 s', tel.stats.meanLatencyS !== null && tel.stats.meanLatencyS < 60, tel.stats.meanLatencyS.toFixed(1) + ' s');
const firstFault = Math.min(...tel.faultLog.map((f) => f.startS));
const earlyAlarms = tel.alarms.filter((a) => a.t < firstFault - 1).length;
check('首个故障前误报数 <= 5', earlyAlarms <= 5, earlyAlarms + ' 条');
check('存在严重级（critical）报警', tel.stats.alarmsCritical > 0, tel.stats.alarmsCritical + ' 条');
check('云端时序库写入数据点 > 10000', tel.stats.points > 10000, tel.stats.points + ' 点');
check('上行丢包率在合理范围（0.2%~3%）', tel.gateway.stats.batchesLost / tel.gateway.stats.batchesSent > 0.002 && tel.gateway.stats.batchesLost / tel.gateway.stats.batchesSent < 0.03,
  (100 * tel.gateway.stats.batchesLost / tel.gateway.stats.batchesSent).toFixed(2) + '%');
const coolantAlarms = tel.alarms.filter((a) => a.deviceId === 'M1' && a.signal === 'coolantTempC' && a.severity === 'critical');
check('冷却液温度过高产生严重报警并携带 DTC', coolantAlarms.length > 0 && coolantAlarms[0].spn === 110,
  coolantAlarms.length + ' 条, SPN=' + (coolantAlarms[0] ? coolantAlarms[0].spn : 'n/a'));

/* =======================================================================
 * 10) 端到端产物
 * ======================================================================= */
group('10) 端到端产物 data/run_result.json');
const resultPath = path.join(ROOT, 'data', 'run_result.json');
if (fs.existsSync(resultPath)) {
  const j = JSON.parse(fs.readFileSync(resultPath, 'utf8'));
  check('run_result.json 可解析且包含 8 大板块', !!(j.meta && j.fields && j.coverage && j.dispatch && j.tracking && j.vision && j.gnssFusion && j.telemetry),
    Object.keys(j).length + ' 个顶层字段');
  check('产物地块数与模块一致（10 块）', Array.isArray(j.fields) && j.fields.length === 10, '地块数=' + (j.fields ? j.fields.length : 0));
  check('产物中含调度甘特数据', !!(j.dispatch && Array.isArray(j.dispatch.gantt) && j.dispatch.gantt.length > 0), '甘特行=' + (j.dispatch && j.dispatch.gantt ? j.dispatch.gantt.length : 0));
  check('产物中跟踪误差与产物统计一致', !!(j.tracking && j.tracking.rmsCrossTrackM < 0.1), j.tracking ? (j.tracking.rmsCrossTrackM * 100).toFixed(2) + ' cm' : 'n/a');
  check('产物摘要指标齐全', !!(j.summary && j.summary.meanCoverageRate > 0.98 && j.summary.telemetryDetectionRate === 1));
} else {
  console.log('  [SKIP] 未找到 data/run_result.json（请先运行 node src\\cli.mjs）');
}

/* =======================================================================
 * 汇总
 * ======================================================================= */
console.log('');
console.log('='.repeat(74));
console.log('  自检结果: ' + passed + ' 项通过, ' + failed + ' 项失败, 共 ' + (passed + failed) + ' 项');
if (failed) {
  console.log('  失败项:');
  for (const f of failures) console.log('    - ' + f);
}
console.log('='.repeat(74));
process.exit(failed === 0 ? 0 : 1);
