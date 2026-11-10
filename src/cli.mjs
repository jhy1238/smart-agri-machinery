#!/usr/bin/env node
/**
 * src/cli.mjs
 * ---------------------------------------------------------------------------
 * 智能农机无人作业系统 —— 端到端主线（软件原型）
 *
 * 执行链路：
 *   1) 生成仿真地块（矩形 / L 形 / 凸多边形，含作物行参数）
 *   2) 全覆盖路径规划（牛耕式 + 地头 Π/Ω 转弯 + 重叠率与覆盖率栅格化）
 *   3) 多机协同调度（贪心构造 + 局部搜索，与朴素轮询基线对比）
 *   4) 闭环路径跟踪仿真（纯跟踪 + 曲率/转向速率限速 + EKF 融合定位）
 *   5) 视觉杂草识别（ExG 分割 + 连通域 + 行带特征分类 + 密度栅格与处方图）
 *   6) GNSS/RTK + IMU 扩展卡尔曼融合定位精度评估
 *   7) 农机物联网遥测与异常检测（J1939 编码 + 3-sigma/EWMA/变化率/阈值 + 故障注入）
 *   8) 输出结构化 JSON（data/run_result.json）与控制台中文作业报告
 *
 * 运行：node src/cli.mjs  [--seed=2024] [--quick]
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
  MACHINE_TYPES,
} from './sim/field_simulator.mjs';
import { runFusionScenario } from './perception/gnss_fusion.mjs';
import { runWeedDetection } from './perception/vision_weed.mjs';
import { CoveragePlanner } from './planning/coverage_path.mjs';
import { DispatchOptimizer, extractGantt } from './planning/dispatch.mjs';
import { simulateTracking } from './control/pure_pursuit.mjs';
import { runTelemetryScenario } from './cloud/telemetry.mjs';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.resolve(__dirname, '..');

/* ---------------- 命令行参数 ---------------- */
const args = process.argv.slice(2);
const argOf = (name, dflt) => {
  const hit = args.find((a) => a.startsWith('--' + name + '='));
  return hit ? hit.split('=')[1] : dflt;
};
const SEED = Number(argOf('seed', 20240501));
const QUICK = args.includes('--quick');

/* ---------------- 报告工具 ---------------- */
const lines = [];
const out = (s) => {
  lines.push(s);
  console.log(s);
};
const pad = (s, n) => {
  const str = String(s);
  let w = 0;
  for (const ch of str) w += /[\u4e00-\u9fa5\uff00-\uffef]/.test(ch) ? 2 : 1;
  let r = str;
  while (w < n) {
    r += ' ';
    w += 1;
  }
  return r;
};
const num = (v, n) => Number(v).toFixed(n === undefined ? 2 : n);
const line = (ch) => out(ch.repeat(78));

/* =========================================================================
 * 1) 地块生成
 * ========================================================================= */
line('=');
out('        智能农机无人作业系统 · 端到端仿真运行报告');
out('        Intelligent Agricultural Machinery — Autonomous Operation Prototype');
line('=');
out('随机种子: ' + SEED + '    运行模式: ' + (QUICK ? '快速' : '完整') + '    Node.js: ' + process.version);
out('');

const fields = [
  Object.assign(makeRectangleField({ id: 'F1', name: '东风农场 1 号大田', originX: 0, originY: 0, lengthM: 300, widthM: 180 }), {
    crop: '玉米', operation: '播种', cropRowSpacingM: 0.6,
  }),
  Object.assign(makeLShapeField({ id: 'F2', name: '东风农场 2 号 L 形地', originX: 380, originY: 0 }), {
    crop: '大豆', operation: '播种', cropRowSpacingM: 0.45,
  }),
  Object.assign(makeConvexPolygonField({ id: 'F3', name: '北岭不规则地块', centerX: 830, centerY: 320 }), {
    crop: '小麦', operation: '植保', cropRowSpacingM: 0.15,
  }),
  Object.assign(makeRectangleField({ id: 'F4', name: '南坡果园', originX: 0, originY: 420, lengthM: 200, widthM: 120 }), {
    crop: '苹果', operation: '植保', cropRowSpacingM: 4.0, kind: '果园',
  }),
  Object.assign(makeRectangleField({ id: 'F5', name: '西河湾 5 号大田', originX: 420, originY: 420, lengthM: 260, widthM: 150 }), {
    crop: '水稻', operation: '收获', cropRowSpacingM: 0.3,
  }),
  Object.assign(makeRectangleField({ id: 'F6', name: '设施园艺示范棚区', originX: 900, originY: 0, lengthM: 120, widthM: 80 }), {
    crop: '番茄', operation: '耕整', cropRowSpacingM: 0.5, kind: '设施',
  }),
  Object.assign(makeRectangleField({ id: 'F7', name: '东岗 7 号麦田', originX: 720, originY: 620, lengthM: 280, widthM: 170 }), {
    crop: '小麦', operation: '收获', cropRowSpacingM: 0.2,
  }),
  Object.assign(makeRectangleField({ id: 'F8', name: '三里屯 8 号大田', originX: 0, originY: 640, lengthM: 220, widthM: 140 }), {
    crop: '玉米', operation: '播种', cropRowSpacingM: 0.6,
  }),
  Object.assign(makeRectangleField({ id: 'F9', name: '北坡梨园', originX: 1080, originY: 480, lengthM: 180, widthM: 110 }), {
    crop: '梨', operation: '植保', cropRowSpacingM: 4.5, kind: '果园',
  }),
  Object.assign(makeRectangleField({ id: 'F10', name: '南洼 10 号地', originX: 1060, originY: 800, lengthM: 200, widthM: 130 }), {
    crop: '大豆', operation: '耕整', cropRowSpacingM: 0.45,
  }),
];

out('【一】地块与作物');
line('-');
out(pad('编号', 6) + pad('地块名称', 24) + pad('类型', 8) + pad('面积(ha)', 10) + pad('周长(m)', 10) + pad('作物', 8) + pad('作业环节', 10));
let totalAreaHa = 0;
const cropRowsInfo = [];
for (const f of fields) {
  totalAreaHa += f.areaHa;
  const rows = generateCropRows(f, {});
  cropRowsInfo.push({ fieldId: f.id, rowCount: rows.rowCount, totalRowLengthM: rows.totalRowLengthM, theoreticalPlants: rows.theoreticalPlants });
  out(
    pad(f.id, 6) +
      pad(f.name, 24) +
      pad(f.kind, 8) +
      pad(num(f.areaHa, 2), 10) +
      pad(num(f.perimeterM, 0), 10) +
      pad(f.crop, 8) +
      pad(f.operation, 10),
  );
}
out('合计作业面积: ' + num(totalAreaHa, 2) + ' 公顷 (' + num(totalAreaHa * 15, 1) + ' 亩)；作物行理论株数合计 ' + cropRowsInfo.reduce((s, r) => s + r.theoreticalPlants, 0).toLocaleString('en-US') + ' 株');
out('');

/* =========================================================================
 * 2) 全覆盖路径规划
 * ========================================================================= */
const fleet = buildFleet({ depotX: 950, depotY: -80 });
const machineById = new Map(fleet.map((m) => [m.id, m]));
// 机具选型规则：小地块（< 2 ha）优先选幅宽最小的机具（减少地头重叠、提高通过性），
//               大地块优先选作业效率最高的机具（机具-地块匹配）
const pickReference = (operation, areaHa) => {
  const capable = fleet.filter((m) => m.capabilities.includes(operation));
  if (!capable.length) throw new Error('无农机可执行作业环节: ' + operation);
  if (areaHa < 2.0) {
    return capable.reduce((a, b) => (b.swathWidthM < a.swathWidthM ? b : a));
  }
  return capable.reduce((a, b) => (b.fieldCapacityHaPerH > a.fieldCapacityHaPerH ? b : a));
};

out('【二】全覆盖路径规划（牛耕式 + 地头转弯）');
line('-');
out(pad('地块', 6) + pad('农机', 22) + pad('幅宽', 6) + pad('作业方向', 9) + pad('行程数', 7) + pad('作业里程km', 11) + pad('转弯里程m', 10) + pad('转弯Π/Ω', 10) + pad('覆盖率%', 8) + pad('重叠率%', 8) + pad('时长h', 7));
const coveragePlans = [];
for (const f of fields) {
  const ref = pickReference(f.operation, f.areaHa);
  const planner = new CoveragePlanner({
    field: f,
    swathWidthM: ref.swathWidthM,
    overlapRatio: 0.08,
    minTurnRadiusM: ref.minTurnRadiusM,
    headlandWidthM: 2 * ref.swathWidthM,
    workSpeedMps: ref.workSpeedMps,
    aLatMaxMps2: 1.5,
    steerRateDegPerS: 25,
    machineId: ref.id,
    turnStrategy: 'auto',
    // 喷施类作业地头转角关闭喷头（不计入覆盖面积），其余作业连续作业
    cornerArcWorking: f.operation !== '植保',
  });
  const plan = planner.plan();
  coveragePlans.push({ field: f, ref, plan });
  const m = plan.metrics;
  out(
    pad(f.id, 6) +
      pad(ref.name.slice(0, 11), 22) +
      pad(num(m.swathWidthM, 1) + 'm', 6) +
      pad(m.sweepAngleDeg + '°', 9) +
      pad(m.swathCount, 7) +
      pad(num(m.workLengthM / 1000, 2), 11) +
      pad(num(m.turnLengthM, 0), 10) +
      pad(m.piTurnCount + '/' + m.omegaTurnCount, 10) +
      pad(num(m.coverageRate * 100, 2), 8) +
      pad(num(m.overlapRatioMeasured * 100, 2), 8) +
      pad(num(m.totalTimeH, 2), 7),
  );
}
const covAvg = coveragePlans.reduce((s, c) => s + c.plan.metrics.coverageRate, 0) / coveragePlans.length;
const ovlAvg = coveragePlans.reduce((s, c) => s + c.plan.metrics.overlapRatioMeasured, 0) / coveragePlans.length;
const nwAvg = coveragePlans.reduce((s, c) => s + c.plan.metrics.nonWorkingRatio, 0) / coveragePlans.length;
out('平均覆盖率 ' + num(covAvg * 100, 2) + '%    平均重叠率(实测) ' + num(ovlAvg * 100, 2) + '%    平均非作业里程占比 ' + num(nwAvg * 100, 2) + '%');
out('说明：Π 形平头转弯段为 Dubins CSC 解或闭式两段 90° 圆弧 + 直线；Ω 形灯泡转弯为 Dubins CCC 解；');
out('      当行程间距 < 2R 时自动切换为"跳行作业"（组内间距 k*ds >= 2R 用 Π 形，组间仅需一次 Ω 形）。');
out('');

/* =========================================================================
 * 3) 多机协同调度
 * ========================================================================= */
out('【三】多机协同调度（贪心构造 + 局部搜索 vs 朴素轮询基线）');
line('-');
const tasks = coveragePlans.map((c, i) => {
  const capable = fleet.filter((m) => m.capabilities.includes(c.field.operation));
  const refCap = c.field.areaHa / c.plan.metrics.totalTimeH;
  const durationHByMachine = {};
  for (const m of capable) {
    // 机具能力折算：作业效率与幅宽、作业速度成正比
    const cap = refCap * (m.swathWidthM / c.ref.swathWidthM) * (m.workSpeedMps / c.ref.workSpeedMps);
    durationHByMachine[m.id] = c.field.areaHa / cap;
  }
  const windowEnd = c.field.operation === '收获' ? 20 : c.field.operation === '植保' ? 16 : 14;
  return {
    id: 'T' + (i + 1),
    name: c.field.name,
    fieldId: c.field.id,
    capability: c.field.operation,
    areaHa: c.field.areaHa,
    location: c.field.centroid,
    timeWindowH: [i * 0.6, windowEnd],
    durationHByMachine,
    referenceDurationH: c.plan.metrics.totalTimeH,
  };
});
const optimizer = new DispatchOptimizer({ tasks, machines: fleet });
const dispatch = optimizer.solve();
const gantt = extractGantt(dispatch.schedule, { machines: fleet });

out(pad('农机', 6) + pad('机型', 26) + pad('作业地块序列', 30) + pad('班次(h)', 9) + pad('作业(h)', 9) + pad('成本(元)', 9));
for (const g of gantt) {
  const seq = g.items.filter((i) => i.type === 'work').map((i) => i.taskId).join(' → ') || '（空闲）';
  const workH = g.items.filter((i) => i.type === 'work').reduce((s, i) => s + (i.endH - i.startH), 0);
  const mk = machineById.get(g.machineId);
  out(pad(g.machineId, 6) + pad(mk.name.slice(0, 12), 26) + pad(seq, 30) + pad(num(g.endH - g.availableFromH, 2), 9) + pad(num(workH, 2), 9) + pad(num((g.endH - g.availableFromH) * mk.costPerHour, 0), 9));
}
const imp = dispatch.improvement;
out('');
out(pad('指标', 22) + pad('优化方案', 14) + pad('朴素基线', 14) + pad('改进', 12));
const rowM = (name, a, b, pct, unit) =>
  out(pad(name, 22) + pad(num(a, 2) + (unit || ''), 14) + pad(num(b, 2) + (unit || ''), 14) + pad((pct >= 0 ? '-' : '+') + num(Math.abs(pct), 1) + '%', 12));
rowM('目标函数 J', dispatch.schedule.J, dispatch.baseline.schedule.J, imp.objectivePct);
rowM('完工时间', dispatch.schedule.makespanH, dispatch.baseline.schedule.makespanH, imp.makespanPct, ' h');
rowM('空驶里程', dispatch.schedule.transitKm, dispatch.baseline.schedule.transitKm, imp.transitPct, ' km');
rowM('等待时间', dispatch.schedule.waitH, dispatch.baseline.schedule.waitH, imp.waitPct, ' h');
rowM('作业成本', dispatch.schedule.costK, dispatch.baseline.schedule.costK, imp.costPct, ' k元');
out('目标函数权重: w1(完工)=1.0  w2(空驶)=0.35  w3(等待)=0.20  w4(成本)=0.15  w5(超窗)=5.0');
out('局部搜索改进: ' + num(imp.localSearchGainPct, 2) + '%（共 ' + dispatch.searchLog.length + ' 次改进移动：Or-opt 迁移 / 机间交换 / 机内 2-opt）');
out('');

/* =========================================================================
 * 4) 闭环路径跟踪仿真
 * ========================================================================= */
out('【四】路径跟踪闭环仿真（纯跟踪控制 + EKF 融合定位）');
line('-');
const mainPlan = coveragePlans.reduce((a, b) => (b.plan.metrics.workLengthM > a.plan.metrics.workLengthM ? b : a));
const rawPath = mainPlan.plan.path.map((p) => ({ x: p.x, y: p.y, type: p.type }));
let acc = 0;
for (let i = 0; i < rawPath.length; i += 1) {
  if (i > 0) acc += Math.hypot(rawPath[i].x - rawPath[i - 1].x, rawPath[i].y - rawPath[i - 1].y);
  rawPath[i].s = acc;
}
// 取一段包含长直线与地头转弯的代表性路段（最长作业行起始，长度 300 m）
let bestI = 0;
let bestLen = 0;
for (let i = 1; i < rawPath.length; i += 1) {
  if (rawPath[i].type === 'work' && rawPath[i - 1].type === 'work') {
    let j = i;
    while (j < rawPath.length && rawPath[j].type === 'work') j += 1;
    const L = rawPath[j - 1].s - rawPath[i - 1].s;
    if (L > bestLen) {
      bestLen = L;
      bestI = i - 1;
    }
    i = j;
  }
}
const track = simulateTracking(rawPath, {
  startIndex: bestI,
  simLengthM: 300,
  maxTimeS: 300,
  vMaxMps: mainPlan.ref.workSpeedMps,
  aLatMaxMps2: 1.5,
  steerRateDegPerS: 25,
  wheelbaseM: 2.6,
  vTurnMaxMps: 1.6,
  seed: SEED + 7,
});
out('被控对象: ' + mainPlan.ref.name + '   轴距 2.60 m   最大前轮转角 ' + num(track.controller.maxSteerDeg, 0) + '°   转向速率上限 25°/s');
out('仿真路段: ' + mainPlan.field.name + '   长度 ' + num(track.pathLengthM, 1) + ' m   仿真时长 ' + num(track.simTimeS, 1) + ' s   行驶里程 ' + num(track.travelledM, 1) + ' m');
out('横向跟踪误差: RMS = ' + num(track.rmsCrossTrackM * 100, 2) + ' cm   平均绝对误差 = ' + num(track.meanAbsCrossTrackM * 100, 2) + ' cm   最大 = ' + num(track.maxCrossTrackM * 100, 2) + ' cm');
out('航向误差 RMS = ' + num(track.rmsHeadingErrDeg, 3) + '°   平均车速 = ' + num(track.meanSpeedMps, 2) + ' m/s   最高车速 = ' + num(track.maxSpeedMps, 2) + ' m/s');
out('前视距离自适应: ' + num(track.controller.lookaheadMinM, 1) + ' ~ ' + num(track.controller.lookaheadMaxM, 1) + ' m（速度增益 ' + num(track.controller.lookaheadGainS, 2) + ' s，曲率自适应衰减）');
out('速度规划约束: v <= min(v_max, sqrt(a_lat/|kappa|), omega_delta/(L*|dkappa/ds|), v_turn)');
out('');

/* =========================================================================
 * 5) 视觉杂草识别
 * ========================================================================= */
out('【五】视觉杂草识别（ExG = 2G - R - B，连通域特征分类）');
line('-');
out(pad('样区', 6) + pad('真值密度株/m²', 14) + pad('检出簇数', 9) + pad('精确率', 9) + pad('召回率', 9) + pad('F1', 8) + pad('行间召回', 10) + pad('行内召回', 10) + pad('分割IoU', 9));
const weedRuns = [];
const densities = QUICK ? [0.8] : [0.4, 0.8, 1.6];
let wIdx = 0;
for (const d of densities) {
  const { makeSyntheticFieldImage } = await import('./sim/field_simulator.mjs');
  const img = makeSyntheticFieldImage({ seed: SEED + 100 + wIdx * 7, weedDensityPerM2: d, rowSpacingM: 0.6 });
  const res = runWeedDetection(img);
  weedRuns.push({ density: d, res, img: { width: img.width, height: img.height, mPerPx: img.mPerPx, truth: img.truth } });
  const e = res.evaluation;
  out(
    pad('W' + (wIdx + 1), 6) +
      pad(num(img.truth.weedDensityPerM2, 2), 14) +
      pad(res.weedCount, 9) +
      pad(num(e.weedPrecision, 3), 9) +
      pad(num(e.weedRecall, 3), 9) +
      pad(num(e.weedF1, 3), 8) +
      pad(num(e.recallInterRowWeed, 3), 10) +
      pad(num(e.recallInRowWeed, 3), 10) +
      pad(num(e.segmentation.iou, 3), 9),
  );
  wIdx += 1;
}
const wMain = weedRuns[weedRuns.length - 1].res;
out('');
out('样区参数: 640x480 px @ 0.02 m/px = 12.8 m x 9.6 m；作物行距 0.6 m；杂草密度栅格 1 m x 1 m');
out('作物行自动检测: 检出 ' + wMain.rowDetection.detectedRows + ' 行，行距 ' + num(wMain.rowDetection.spacingM, 3) + ' m，与真值平均偏差 ' + num(wMain.rowDetection.meanAbsErrorM * 100, 2) + ' cm');
out('植被分割: Otsu 自动阈值 ExG > ' + wMain.otsuThreshold + '，形态学去噪剔除 ' + wMain.noiseRemovedPx + ' px，分割 IoU ' + num(wMain.evaluation.segmentation.iou, 3));
out('杂草密度栅格（' + wMain.grid.cols + ' x ' + wMain.grid.rows + ' 格，单位 簇/m²）:');
for (let r = wMain.grid.rows - 1; r >= 0; r -= 1) {
  let rowStr = '   ';
  for (let c = 0; c < wMain.grid.cols; c += 1) {
    const v = wMain.grid.density[r * wMain.grid.cols + c];
    rowStr += (v === 0 ? '·' : v < 1 ? '▁' : v < 2 ? '▃' : v < 3 ? '▅' : '█') + ' ';
  }
  out(rowStr);
}
out('   密度分级统计: 无 ' + wMain.grid.zoneCount['无'] + ' 格，轻 ' + wMain.grid.zoneCount['轻'] + ' 格，中 ' + wMain.grid.zoneCount['中'] + ' 格，重 ' + wMain.grid.zoneCount['重'] + ' 格');
out('变量喷药处方: 变量作业总药量 ' + num(wMain.prescription.variableTotalL, 3) + ' L，均一喷施 ' + num(wMain.prescription.uniformTotalL, 3) + ' L，节省 ' + num(wMain.prescription.savingPct, 1) + '%');
out('');

/* =========================================================================
 * 6) GNSS/RTK + IMU 融合定位
 * ========================================================================= */
out('【六】GNSS/RTK + IMU 扩展卡尔曼融合定位');
line('-');
const fusion = runFusionScenario({ seed: SEED, laps: 1 });
const fm = fusion.metrics;
out('状态向量 x = [x, y, theta, v, omega]；控制 u = [a, alpha]；观测：RTK 位置 / 双天线航向 / 轮速 / 陀螺');
out('轨迹: 闭环真值轨迹 ' + num(fusion.track.lengthM, 1) + ' m，时长 ' + num(fusion.track.durationS, 1) + ' s，含 4 处 90° 地头圆弧（R=6 m）');
out('定位误差: 全程 RMS = ' + num(fm.rmsErrorM * 100, 2) + ' cm，最大 ' + num(fm.maxErrorM * 100, 2) + ' cm');
out('  · RTK 固定解区间 RMS = ' + num(fm.rmsErrorRtkFixedM * 100, 2) + ' cm（对应水平精度 1 cm + 1 ppm 的板卡指标）');
out('  · RTK 浮点解区间 RMS = ' + num(fm.rmsErrorRtkFloatM * 100, 2) + ' cm');
out('  · 丢星区间（航位推算）误差 RMS = ' + num(fm.rmsErrorOutageM * 100, 2) + ' cm，最大 ' + num(fm.maxErrorOutageM * 100, 2) + ' cm，恢复后 3 s 内 RMS = ' + num(fm.rmsErrorRecoveryM * 100, 2) + ' cm');
out('航向误差 RMS = ' + num(fm.headingRmsDeg, 3) + '°    GNSS 观测接受率 = ' + num(fm.gnssAcceptedRate * 100, 2) + '%');
out('NIS 卡方门控: 检出并剔除野值 ' + fm.rejectedOutliers + ' 个（注入多路径野值 ' + fm.injectedOutliers + ' 个）');
out('丢星降级: 检测到连续丢星 ' + num(fm.detectedOutageS, 1) + ' s（>2.0 s 判定门限即进入航位推算并膨胀过程噪声 4 倍），降级累计 ' + num(fm.deadReckoningS, 1) + ' s');
out('');

/* =========================================================================
 * 7) 遥测与异常检测
 * ========================================================================= */
out('【七】农机物联网遥测与异常检测');
line('-');
const telMachines = fleet.filter((m) => ['M1', 'M2', 'M3'].includes(m.id));
// 遥测场景固定 30 min（计算量极小），快速模式仅减少视觉样区数量，保证各模式指标口径一致
const tel = runTelemetryScenario({ machines: telMachines, durationMin: 30, seed: SEED + 3 });
const ts = tel.stats;
out('链路: 边缘网关 ' + ts.gateway.samplesCollected + ' 次采样 -> 编码 CAN 报文 ' + ts.gateway.framesEncoded + ' 帧 -> 上行 ' + ts.gateway.batchesSent + ' 批（丢包 ' + ts.gateway.batchesLost + ' 批 / ' + ts.gateway.framesDropped + ' 帧）-> 云端时序点 ' + ts.points + ' 个');
out('云端: 设备 ' + ts.devices + ' 台，信号 ' + ts.signals + ' 类，时序键 ' + ts.seriesKeys + ' 个，上行数据量 ' + num(ts.gateway.bytesUplink / 1024, 1) + ' KiB');
out('检测规则: 绝对阈值 + 滑动窗口 3-sigma(60 s) + EWMA 控制图(lambda=0.2) + 变化率(10 s/40%)');
out('报警统计: 共 ' + ts.alarms + ' 条（严重 ' + ts.alarmsCritical + ' 条，警告 ' + ts.alarmsWarning + ' 条），故障检出率 ' + num(ts.detectionRate * 100, 1) + '%，平均检出时延 ' + num(ts.meanLatencyS, 1) + ' s');
out('');
out(pad('设备', 6) + pad('注入故障', 34) + pad('检出', 6) + pad('首报(s)', 9) + pad('时延(s)', 9) + pad('报警数', 8));
for (const f of tel.faultLog) {
  out(
    pad(f.deviceId, 6) +
      pad(f.label.slice(0, 17), 34) +
      pad(f.detected ? '是' : '否', 6) +
      pad(f.firstAlarmS === null ? '—' : num(f.firstAlarmS, 0), 9) +
      pad(f.latencyS === null ? '—' : num(Math.max(0, f.latencyS), 0), 9) +
      pad(f.alarmCount, 8),
  );
}
out('');
out('报警明细（前 8 条）:');
for (const a of tel.alarms.slice(0, 8)) {
  out('  [' + num(a.t, 0) + 's] ' + a.severity.toUpperCase() + ' ' + a.machineName.slice(0, 8) + ' ' + a.label + ' = ' + a.value + ' (' + a.rule + ')' + (a.spn ? '  DTC SPN' + a.spn + '/FMI' + a.fmi : ''));
  out('        ' + a.message);
}
out('');

/* =========================================================================
 * 8) 结构化结果输出
 * ========================================================================= */
const result = {
  meta: {
    project: '智能农机无人作业系统软件原型',
    version: '1.0.0',
    generatedAt: new Date().toISOString(),
    node: process.version,
    seed: SEED,
    mode: QUICK ? 'quick' : 'full',
  },
  fields: fields.map((f, i) => ({
    id: f.id,
    name: f.name,
    kind: f.kind,
    crop: f.crop,
    operation: f.operation,
    areaHa: Number(f.areaHa.toFixed(4)),
    areaM2: Number(f.areaM2.toFixed(2)),
    perimeterM: Number(f.perimeterM.toFixed(2)),
    centroid: { x: Number(f.centroid.x.toFixed(2)), y: Number(f.centroid.y.toFixed(2)) },
    cropRowSpacingM: f.cropRowSpacingM,
    cropRows: cropRowsInfo[i],
    polygon: f.polygon.map((p) => ({ x: Number(p.x.toFixed(2)), y: Number(p.y.toFixed(2)) })),
  })),
  coverage: coveragePlans.map((c) => ({
    fieldId: c.field.id,
    machineId: c.ref.id,
    machineName: c.ref.name,
    metrics: Object.fromEntries(Object.entries(c.plan.metrics).map(([k, v]) => [k, typeof v === 'number' ? Number(v.toFixed(4)) : v])),
    turnCostSamples: c.plan.turnCostDetail.slice(0, 12).map((t) => ({
      type: t.type,
      word: t.word,
      lengthM: Number(t.lengthM.toFixed(3)),
      timeS: Number(t.timeS.toFixed(3)),
      lateralOffsetM: t.lateralOffsetM === null || t.lateralOffsetM === undefined ? null : Number(t.lateralOffsetM.toFixed(3)),
    })),
    pathSample: c.plan.path.filter((_, i) => i % Math.max(1, Math.floor(c.plan.path.length / 200)) === 0).map((p) => ({
      x: Number(p.x.toFixed(2)),
      y: Number(p.y.toFixed(2)),
      type: p.type,
    })),
  })),
  dispatch: {
    tasks: tasks.map((t) => ({
      id: t.id,
      name: t.name,
      fieldId: t.fieldId,
      capability: t.capability,
      areaHa: Number(t.areaHa.toFixed(3)),
      timeWindowH: t.timeWindowH,
    })),
    assignment: dispatch.assignment,
    baselineAssignment: dispatch.baseline.assignment,
    schedule: {
      J: Number(dispatch.schedule.J.toFixed(4)),
      makespanH: Number(dispatch.schedule.makespanH.toFixed(4)),
      transitKm: Number(dispatch.schedule.transitKm.toFixed(4)),
      waitH: Number(dispatch.schedule.waitH.toFixed(4)),
      costK: Number(dispatch.schedule.costK.toFixed(4)),
      tardinessH: Number(dispatch.schedule.tardinessH.toFixed(4)),
    },
    baselineSchedule: {
      J: Number(dispatch.baseline.schedule.J.toFixed(4)),
      makespanH: Number(dispatch.baseline.schedule.makespanH.toFixed(4)),
      transitKm: Number(dispatch.baseline.schedule.transitKm.toFixed(4)),
      waitH: Number(dispatch.baseline.schedule.waitH.toFixed(4)),
      costK: Number(dispatch.baseline.schedule.costK.toFixed(4)),
      tardinessH: Number(dispatch.baseline.schedule.tardinessH.toFixed(4)),
    },
    improvement: Object.fromEntries(Object.entries(imp).map(([k, v]) => [k, Number(v.toFixed(4))])),
    gantt: gantt.map((g) => ({
      machineId: g.machineId,
      machineName: g.machineName,
      availableFromH: g.availableFromH,
      endH: Number(g.endH.toFixed(3)),
      items: g.items.map((i) => ({
        taskId: i.taskId,
        type: i.type,
        startH: Number(i.startH.toFixed(3)),
        endH: Number(i.endH.toFixed(3)),
      })),
    })),
  },
  tracking: {
    fieldId: mainPlan.field.id,
    machineId: mainPlan.ref.id,
    controller: track.controller,
    pathLengthM: Number(track.pathLengthM.toFixed(3)),
    travelledM: Number(track.travelledM.toFixed(3)),
    simTimeS: Number(track.simTimeS.toFixed(2)),
    rmsCrossTrackM: Number(track.rmsCrossTrackM.toFixed(5)),
    meanAbsCrossTrackM: Number(track.meanAbsCrossTrackM.toFixed(5)),
    maxCrossTrackM: Number(track.maxCrossTrackM.toFixed(5)),
    rmsHeadingErrDeg: Number(track.rmsHeadingErrDeg.toFixed(4)),
    meanSpeedMps: Number(track.meanSpeedMps.toFixed(4)),
    maxSpeedMps: Number(track.maxSpeedMps.toFixed(4)),
    steerSaturationRatio: Number(track.steerSaturationRatio.toFixed(4)),
    series: track.series.map((p) => ({
      t: p.t,
      v: Number(p.v.toFixed(3)),
      vTarget: Number(p.vTarget.toFixed(3)),
      crossTrackM: Number(p.crossTrackM.toFixed(4)),
      steerDeg: Number(p.steerDeg.toFixed(2)),
      lookaheadM: Number(p.lookaheadM.toFixed(3)),
      headingErrDeg: Number(p.headingErrDeg.toFixed(3)),
    })),
  },
  vision: {
    tiles: weedRuns.map((w) => ({
      densityTruthPerM2: Number(w.img.truth.weedDensityPerM2.toFixed(3)),
      imageSize: { width: w.img.width, height: w.img.height, mPerPx: w.img.mPerPx },
      rowDetection: w.res.rowDetection,
      exgThreshold: w.res.exgThreshold,
      otsuThreshold: w.res.otsuThreshold,
      weedClusterCount: w.res.weedCount,
      cropBlobCount: w.res.cropBlobCount,
      grid: { cols: w.res.grid.cols, rows: w.res.grid.rows, density: w.res.grid.density.map((d) => Number(d.toFixed(3))), zones: w.res.grid.zones, zoneCount: w.res.grid.zoneCount },
      prescription: { variableTotalL: w.res.prescription.variableTotalL, uniformTotalL: w.res.prescription.uniformTotalL, savingPct: w.res.prescription.savingPct },
      evaluation: w.res.evaluation,
      weedList: w.res.weeds.slice(0, 40).map((b) => ({
        x: Number(b.centroidM.x.toFixed(3)),
        y: Number(b.centroidM.y.toFixed(3)),
        areaM2: Number(b.areaM2.toFixed(4)),
        outOfBandRatio: Number(b.outOfBandRatio.toFixed(3)),
        reason: b.reason,
      })),
    })),
  },
  gnssFusion: {
    track: fusion.track,
    metrics: Object.fromEntries(Object.entries(fm).map(([k, v]) => [k, Number(v.toFixed(5))])),
    diagnostics: {
      mode: fusion.diagnostics.mode,
      stats: fusion.diagnostics.stats,
      maxOutageS: Number(fusion.diagnostics.maxOutageS.toFixed(3)),
      events: fusion.diagnostics.events,
    },
  },
  telemetry: {
    params: tel.params,
    stats: ts,
    faults: tel.faultLog.map((f) => ({
      deviceId: f.deviceId,
      machineName: f.machineName,
      kind: f.kind,
      label: f.label,
      startS: f.startS,
      detected: f.detected,
      latencyS: f.latencyS === null ? null : Number(f.latencyS.toFixed(2)),
      alarmCount: f.alarmCount,
      alarmRules: f.alarmRules,
    })),
    alarms: tel.alarms.map((a) => ({
      t: a.t,
      deviceId: a.deviceId,
      machineName: a.machineName,
      signal: a.signal,
      label: a.label,
      value: a.value,
      rule: a.rule,
      severity: a.severity,
      message: a.message,
      spn: a.spn,
      fmi: a.fmi,
      dtcBytes: a.dtcBytes,
    })),
    seriesSample: Object.fromEntries(
      Object.entries(tel.cloud.toJSON(30)).slice(0, 8),
    ),
  },
  summary: {
    totalAreaHa: Number(totalAreaHa.toFixed(3)),
    meanCoverageRate: Number(covAvg.toFixed(5)),
    meanOverlapRatio: Number(ovlAvg.toFixed(5)),
    meanNonWorkingRatio: Number(nwAvg.toFixed(5)),
    dispatchObjectiveImprovementPct: Number(imp.objectivePct.toFixed(3)),
    dispatchTransitImprovementPct: Number(imp.transitPct.toFixed(3)),
    trackingRmsCrossTrackM: Number(track.rmsCrossTrackM.toFixed(5)),
    gnssRmsErrorM: Number(fm.rmsErrorM.toFixed(5)),
    weedPrecision: wMain.evaluation.weedPrecision,
    weedRecall: wMain.evaluation.weedRecall,
    weedF1: wMain.evaluation.weedF1,
    telemetryDetectionRate: ts.detectionRate,
    telemetryAlarms: ts.alarms,
  },
};

const dataDir = path.join(ROOT, 'data');
if (!fs.existsSync(dataDir)) fs.mkdirSync(dataDir, { recursive: true });
const outPath = path.join(dataDir, 'run_result.json');
// 同时输出纯文本中文作业报告（便于归档、打印与 diff）
const reportPath = path.join(dataDir, 'run_report.txt');
fs.writeFileSync(reportPath, lines.join(String.fromCharCode(10)) + String.fromCharCode(10), 'utf8');
out('【八】关键指标达成情况');
line('-');
const checks = [
  ['平均覆盖率', num(covAvg * 100, 2) + ' %', '> 98 %', covAvg > 0.98],
  ['平均重叠率(实测)', num(ovlAvg * 100, 2) + ' %', '5 % ~ 15 %', ovlAvg >= 0.05 && ovlAvg <= 0.15],
  ['横向跟踪误差 RMS', num(track.rmsCrossTrackM * 100, 2) + ' cm', '< 10 cm', track.rmsCrossTrackM < 0.1],
  ['RTK 固定解定位 RMS', num(fm.rmsErrorRtkFixedM * 100, 2) + ' cm', '< 5 cm', fm.rmsErrorRtkFixedM < 0.05],
  ['调度目标函数改进', num(imp.objectivePct, 1) + ' %', '> 10 %', imp.objectivePct > 10],
  ['杂草识别 F1', num(wMain.evaluation.weedF1, 3), '> 0.6', wMain.evaluation.weedF1 > 0.6],
  ['遥测故障检出率', num(ts.detectionRate * 100, 0) + ' %', '= 100 %', ts.detectionRate === 1],
  ['平均故障检出时延', num(ts.meanLatencyS, 1) + ' s', '< 60 s', ts.meanLatencyS !== null && ts.meanLatencyS < 60],
];
out(pad('指标', 24) + pad('实测值', 14) + pad('判据', 14) + '结论');
for (const [name, val, crit, ok] of checks) {
  out(pad(name, 24) + pad(val, 14) + pad(crit, 14) + (ok ? '✔ 达标' : '✘ 未达标'));
}
out('');
out('结构化结果已写入: ' + outPath);
out('文本作业报告已写入: ' + reportPath);
out('（' + num(fs.statSync(outPath).size / 1024, 1) + ' KiB，含地块、路径、调度甘特、跟踪曲线、识别统计、融合精度与遥测报警全量数据）');
line('=');
