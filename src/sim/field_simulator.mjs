/**
 * src/sim/field_simulator.mjs
 * ---------------------------------------------------------------------------
 * 智能农机无人作业系统 —— 地块 / 农机 / 传感器仿真环境
 *
 * 本模块是整个软件原型的"真值"来源，提供：
 *   1) 地块几何：矩形大田、L 形（凹多边形）地块、凸多边形地块；
 *   2) 作物行：按行距生成行中心线、行长与理论株数（可选物化植株点）；
 *   3) 农机装备库：MACHINE_TYPES / buildFleet（能力、幅宽、成本、CAN 节点）；
 *   4) 传感器仿真：GNSS/RTK（固定解 / 浮点解 / 丢星 / 多路径野值）与 IMU（零偏 + 噪声）；
 *   5) 合成农田影像：规则行播作物 + 随机杂草，输出 RGB 影像与像素级真值标签。
 *
 * 坐标系：局部 ENU 平面直角坐标，单位 m；x 轴向东，y 轴向北。
 * 所有地块多边形按逆时针（CCW）给出，即 shoelace 有向面积为正。
 */

/* =========================================================================
 * 1. 随机数发生器（mulberry32，可复现）
 * ========================================================================= */

/**
 * 创建确定性伪随机数发生器，保证仿真结果可复现。
 * @param {number} seed 32 位整数种子
 */
export function createRng(seed) {
  let s = (seed >>> 0) || 0x9e3779b9;
  function next() {
    s = (s + 0x6d2b79f5) >>> 0;
    let t = s;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  }
  return {
    next,
    /** 均匀分布 U(a,b) */
    uniform: (a, b) => a + (b - a) * next(),
    /** 均匀整数 [a,b] */
    int: (a, b) => Math.floor(a + (b - a + 1) * next()) % (b - a + 1),
    /** 正态分布 N(mean, std)，Box-Muller 变换 */
    normal(mean = 0, std = 1) {
      let u = 0;
      let v = 0;
      while (u === 0) u = next();
      while (v === 0) v = next();
      return mean + std * Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
    },
    pick: (arr) => arr[Math.min(arr.length - 1, Math.floor(next() * arr.length))],
  };
}

/* =========================================================================
 * 2. 平面几何工具
 * ========================================================================= */

export function dist(a, b) {
  return Math.hypot(a.x - b.x, a.y - b.y);
}

/** 多边形有向面积（shoelace 公式），CCW 为正 */
export function polygonArea(poly) {
  let s = 0;
  for (let i = 0; i < poly.length; i += 1) {
    const p = poly[i];
    const q = poly[(i + 1) % poly.length];
    s += p.x * q.y - q.x * p.y;
  }
  return s / 2;
}

/** 多边形形心（面积加权） */
export function polygonCentroid(poly) {
  const a = polygonArea(poly);
  let cx = 0;
  let cy = 0;
  for (let i = 0; i < poly.length; i += 1) {
    const p = poly[i];
    const q = poly[(i + 1) % poly.length];
    const cross = p.x * q.y - q.x * p.y;
    cx += (p.x + q.x) * cross;
    cy += (p.y + q.y) * cross;
  }
  return { x: cx / (6 * a), y: cy / (6 * a) };
}

export function polygonBBox(poly) {
  let minX = Infinity;
  let minY = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;
  for (const p of poly) {
    if (p.x < minX) minX = p.x;
    if (p.y < minY) minY = p.y;
    if (p.x > maxX) maxX = p.x;
    if (p.y > maxY) maxY = p.y;
  }
  return { minX, minY, maxX, maxY, width: maxX - minX, height: maxY - minY };
}

export function polygonPerimeter(poly) {
  let s = 0;
  for (let i = 0; i < poly.length; i += 1) s += dist(poly[i], poly[(i + 1) % poly.length]);
  return s;
}

/** 射线法判断点是否在多边形内部 */
export function pointInPolygon(pt, poly) {
  let inside = false;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i, i += 1) {
    const xi = poly[i].x;
    const yi = poly[i].y;
    const xj = poly[j].x;
    const yj = poly[j].y;
    const hit = (yi > pt.y) !== (yj > pt.y);
    if (hit && pt.x < ((xj - xi) * (pt.y - yi)) / (yj - yi) + xi) inside = !inside;
  }
  return inside;
}

/** 绕原点旋转 */
export function rotatePoint(p, angRad) {
  const c = Math.cos(angRad);
  const s = Math.sin(angRad);
  return { x: p.x * c - p.y * s, y: p.x * s + p.y * c };
}

/**
 * 水平扫描线 y = yLine 与多边形的交线段（偶奇规则，半开区间避免顶点重复计数）。
 * @returns {Array<{x0:number,x1:number}>} 按 x 升序的区间列表
 */
export function scanlineIntervals(poly, yLine) {
  const xs = [];
  for (let i = 0; i < poly.length; i += 1) {
    const p = poly[i];
    const q = poly[(i + 1) % poly.length];
    if ((p.y <= yLine && q.y > yLine) || (q.y <= yLine && p.y > yLine)) {
      const t = (yLine - p.y) / (q.y - p.y);
      xs.push(p.x + t * (q.x - p.x));
    }
  }
  xs.sort((a, b) => a - b);
  const out = [];
  for (let i = 0; i + 1 < xs.length; i += 2) {
    if (xs[i + 1] - xs[i] > 1e-6) out.push({ x0: xs[i], x1: xs[i + 1] });
  }
  return out;
}

/* =========================================================================
 * 3. 地块生成
 * ========================================================================= */

function buildField(spec) {
  const poly = spec.polygon.map((p) => ({ x: p.x, y: p.y }));
  const areaM2 = Math.abs(polygonArea(poly));
  const c = polygonCentroid(poly);
  // 若为顺时针则翻转为 CCW，保证下游算法方向一致
  const ccw = polygonArea(poly) > 0 ? poly : poly.slice().reverse();
  return {
    id: spec.id,
    name: spec.name,
    kind: spec.kind || '大田',
    polygon: ccw,
    areaM2,
    areaHa: areaM2 / 10000,
    centroid: c,
    bbox: polygonBBox(ccw),
    perimeterM: polygonPerimeter(ccw),
    cropRowSpacingM: spec.cropRowSpacingM || 0.6,
    plantSpacingM: spec.plantSpacingM || 0.22,
    weedPressure: spec.weedPressure || 'medium',
    soil: spec.soil || '壤土',
    slopeDeg: spec.slopeDeg === undefined ? 1.5 : spec.slopeDeg,
  };
}

/** 矩形大田地块 */
export function makeRectangleField(opts) {
  const o = Object.assign(
    { id: 'F-rect', name: '矩形大田', originX: 0, originY: 0, lengthM: 300, widthM: 180, rotationDeg: 0 },
    opts || {},
  );
  const rad = (o.rotationDeg * Math.PI) / 180;
  const local = [
    { x: 0, y: 0 },
    { x: o.lengthM, y: 0 },
    { x: o.lengthM, y: o.widthM },
    { x: 0, y: o.widthM },
  ];
  const polygon = local.map((p) => {
    const r = rotatePoint(p, rad);
    return { x: r.x + o.originX, y: r.y + o.originY };
  });
  return buildField({ ...o, polygon });
}

/** L 形（凹多边形）地块，用于验证凹边界下的全覆盖规划 */
export function makeLShapeField(opts) {
  const o = Object.assign(
    { id: 'F-lshape', name: 'L 形地块', originX: 0, originY: 0, armXM: 240, armYM: 160, notchXM: 120, notchYM: 90 },
    opts || {},
  );
  const local = [
    { x: 0, y: 0 },
    { x: o.armXM, y: 0 },
    { x: o.armXM, y: o.notchYM },
    { x: o.notchXM, y: o.notchYM },
    { x: o.notchXM, y: o.armYM },
    { x: 0, y: o.armYM },
  ];
  const polygon = local.map((p) => ({ x: p.x + o.originX, y: p.y + o.originY }));
  return buildField({ ...o, polygon });
}

/** 凸多边形地块（由极径序列生成，保证严格凸） */
export function makeConvexPolygonField(opts) {
  const o = Object.assign(
    {
      id: 'F-convex',
      name: '凸多边形地块',
      centerX: 0,
      centerY: 0,
      radii: [130, 112, 126, 106, 121, 116],
      rotationDeg: 12,
    },
    opts || {},
  );
  const n = o.radii.length;
  const polygon = [];
  for (let i = 0; i < n; i += 1) {
    const ang = (2 * Math.PI * i) / n + (o.rotationDeg * Math.PI) / 180;
    polygon.push({ x: o.centerX + o.radii[i] * Math.cos(ang), y: o.centerY + o.radii[i] * Math.sin(ang) });
  }
  return buildField({ ...o, polygon });
}

/**
 * 作物行生成：以 angleDeg 为行方向，按行距切分地块。
 * @returns {{angleDeg:number,rowCount:number,totalRowLengthM:number,theoreticalPlants:number,rows:Array}}
 */
export function generateCropRows(field, opts) {
  const o = Object.assign(
    {
      rowSpacingM: field.cropRowSpacingM,
      plantSpacingM: field.plantSpacingM,
      angleDeg: 0,
      materializePlants: false,
    },
    opts || {},
  );
  const rad = (-o.angleDeg * Math.PI) / 180;
  const rot = field.polygon.map((p) => rotatePoint(p, rad));
  const bb = polygonBBox(rot);
  const rows = [];
  let totalLen = 0;
  const nRow = Math.floor((bb.maxY - bb.minY) / o.rowSpacingM);
  for (let k = 0; k < nRow; k += 1) {
    const y = bb.minY + (k + 0.5) * o.rowSpacingM;
    const iv = scanlineIntervals(rot, y);
    for (const seg of iv) {
      const len = seg.x1 - seg.x0;
      totalLen += len;
      if (rows.length < 4000) {
        const row = { index: rows.length, y, x0: seg.x0, x1: seg.x1, lengthM: len };
        if (o.materializePlants) {
          row.plants = [];
          const nP = Math.max(1, Math.floor(len / o.plantSpacingM));
          for (let i = 0; i < nP; i += 1) {
            const lx = seg.x0 + (i + 0.5) * (len / nP);
            const back = rotatePoint({ x: lx, y }, -rad);
            row.plants.push({ x: back.x, y: back.y });
          }
        }
        rows.push(row);
      }
    }
  }
  return {
    angleDeg: o.angleDeg,
    rowSpacingM: o.rowSpacingM,
    rowCount: rows.length,
    totalRowLengthM: totalLen,
    theoreticalPlants: Math.round(totalLen / o.plantSpacingM),
    rows,
  };
}

/* =========================================================================
 * 4. 农机装备库
 * ========================================================================= */

/** 农机类型库：能力、幅宽、速度、成本、CAN 源地址 */
export const MACHINE_TYPES = {
  'tractor-lx2204': {
    id: 'tractor-lx2204',
    name: '东方红 LX2204 无人驾驶拖拉机',
    category: 'tractor',
    powerKw: 162,
    swathWidthM: 4.5,
    workSpeedMps: 2.2,
    transitSpeedMps: 7.0,
    minTurnRadiusM: 5.5,
    fieldCapacityHaPerH: 1.6,
    fuelLPerH: 22.0,
    costPerHour: 160,
    capabilities: ['耕整', '播种', '施肥'],
    canSourceAddress: 0x21,
  },
  'tractor-fj1004': {
    id: 'tractor-fj1004',
    name: '丰疆智能 FJ-1004 无人拖拉机',
    category: 'tractor',
    powerKw: 73.5,
    swathWidthM: 3.0,
    workSpeedMps: 2.0,
    transitSpeedMps: 6.0,
    minTurnRadiusM: 4.0,
    fieldCapacityHaPerH: 1.1,
    fuelLPerH: 11.5,
    costPerHour: 95,
    capabilities: ['耕整', '播种', '施肥'],
    canSourceAddress: 0x22,
  },
  'harvester-gm100': {
    id: 'harvester-gm100',
    name: '雷沃谷神 GM100 联合收割机',
    category: 'harvester',
    powerKw: 129,
    swathWidthM: 5.0,
    workSpeedMps: 1.6,
    transitSpeedMps: 5.5,
    minTurnRadiusM: 5.0,
    fieldCapacityHaPerH: 0.95,
    fuelLPerH: 26.0,
    costPerHour: 220,
    capabilities: ['收获'],
    canSourceAddress: 0x23,
  },
  'sprayer-uav-t60': {
    id: 'sprayer-uav-t60',
    name: '大疆 T60 植保无人机（含地面站）',
    category: 'uav',
    powerKw: 0,
    swathWidthM: 6.0,
    workSpeedMps: 5.5,
    transitSpeedMps: 12.0,
    minTurnRadiusM: 2.0,
    fieldCapacityHaPerH: 8.0,
    fuelLPerH: 0,
    costPerHour: 180,
    capabilities: ['植保', '施肥'],
    canSourceAddress: 0x24,
  },
};

/**
 * 构建机群实例（含初始停放位置、可用时间、时间窗）。
 * @param {number} fieldCenterX 机库参考位置
 */
export function buildFleet(opts) {
  const o = Object.assign({ depotX: -60, depotY: -60, seed: 7 }, opts || {});
  const defs = [
    { type: 'tractor-fj1004', availableFromH: 0.0, capabilityNote: '播种/施肥主责' },
    { type: 'tractor-lx2204', availableFromH: 0.5, capabilityNote: '耕整/播种' },
    { type: 'harvester-gm100', availableFromH: 0.0, capabilityNote: '收获主责' },
    { type: 'sprayer-uav-t60', availableFromH: 1.0, capabilityNote: '植保/追肥' },
  ];
  return defs.map((d, i) => {
    const t = MACHINE_TYPES[d.type];
    return {
      id: 'M' + (i + 1),
      typeId: t.id,
      name: t.name,
      category: t.category,
      swathWidthM: t.swathWidthM,
      workSpeedMps: t.workSpeedMps,
      transitSpeedMps: t.transitSpeedMps,
      minTurnRadiusM: t.minTurnRadiusM,
      fieldCapacityHaPerH: t.fieldCapacityHaPerH,
      fuelLPerH: t.fuelLPerH,
      costPerHour: t.costPerHour,
      capabilities: t.capabilities.slice(),
      canSourceAddress: t.canSourceAddress,
      // 停放位置：机库周边散开，避免所有农机同点出发
      home: { x: o.depotX + i * 25, y: o.depotY - i * 12 },
      availableFromH: d.availableFromH,
      capabilityNote: d.capabilityNote,
    };
  });
}

/* =========================================================================
 * 5. 真值轨迹与传感器仿真
 * ========================================================================= */

/**
 * 构建"直道 + 90 度圆弧"闭环真值轨迹（模拟田间往复作业的一个循环）。
 *
 * 轨迹分两步生成：
 *   1) 几何参考路径：由 S（直线段）与 A（圆弧段）解析求得，可用弧长精确求位姿；
 *   2) 动力学可实现真值：横摆角速度经过一阶惯性环节（等效转向执行延迟 tau = 0.25 s），
 *      再按 CTRV 模型积分得到。这样真值轨迹曲率连续、无角速度阶跃，
 *      符合电液转向系统 25 deg/s 的转向速率限制，也避免给滤波器注入不可实现的激励。
 */
export function buildLoopTruthTrack(opts) {
  const o = Object.assign(
    {
      straightA: 80,
      straightB: 50,
      radius: 6,
      speed: 2.5,
      laps: 4,
      dt: 0.02,
      originX: 0,
      originY: 0,
      // 转向执行一阶惯性时间常数（s），对应横摆角速度的建立过程
      omegaTauS: 0.25,
    },
    opts || {},
  );
  const segs = [];
  for (let i = 0; i < o.laps; i += 1) {
    segs.push({ type: 'S', len: o.straightA });
    segs.push({ type: 'A', radius: o.radius, angle: Math.PI / 2, dir: 1 });
    segs.push({ type: 'S', len: o.straightB });
    segs.push({ type: 'A', radius: o.radius, angle: Math.PI / 2, dir: 1 });
    segs.push({ type: 'S', len: o.straightA });
    segs.push({ type: 'A', radius: o.radius, angle: Math.PI / 2, dir: 1 });
    segs.push({ type: 'S', len: o.straightB });
    segs.push({ type: 'A', radius: o.radius, angle: Math.PI / 2, dir: 1 });
  }
  let total = 0;
  for (const s of segs) total += s.type === 'S' ? s.len : s.radius * s.angle;

  // 几何参考：返回参考位姿与参考横摆角速度（直道 0，圆弧 v/R）
  const refAt = (s) => {
    let x = o.originX;
    let y = o.originY;
    let th = 0;
    let rest = s;
    let acc = 0;
    let segOmega = 0;
    for (const seg of segs) {
      const L = seg.type === 'S' ? seg.len : seg.radius * seg.angle;
      segOmega = seg.type === 'S' ? 0 : (seg.dir * o.speed) / seg.radius;
      if (rest >= L) {
        if (seg.type === 'S') {
          x += L * Math.cos(th);
          y += L * Math.sin(th);
        } else {
          const dth = seg.dir * seg.angle;
          const cx = x - seg.dir * seg.radius * Math.sin(th);
          const cy = y + seg.dir * seg.radius * Math.cos(th);
          x = cx + seg.dir * seg.radius * Math.sin(th + dth);
          y = cy - seg.dir * seg.radius * Math.cos(th + dth);
          th += dth;
        }
        acc += L;
        rest -= L;
      } else {
        if (seg.type === 'S') {
          x += rest * Math.cos(th);
          y += rest * Math.sin(th);
        } else {
          const dth = (seg.dir * seg.angle * rest) / L;
          const cx = x - seg.dir * seg.radius * Math.sin(th);
          const cy = y + seg.dir * seg.radius * Math.cos(th);
          x = cx + seg.dir * seg.radius * Math.sin(th + dth);
          y = cy - seg.dir * seg.radius * Math.cos(th + dth);
          th += dth;
        }
        acc += rest;
        rest = 0;
        break;
      }
    }
    return { x, y, theta: th, omegaRef: segOmega, s: acc };
  };

  const durationS = total / o.speed;
  const samples = [];
  const n = Math.floor(durationS / o.dt);
  let theta = 0;
  let px = o.originX;
  let py = o.originY;
  let omega = 0;
  for (let i = 0; i <= n; i += 1) {
    const t = i * o.dt;
    const s = (o.speed * t) % total;
    const ref = refAt(s);
    // 一阶惯性环节：d(omega)/dt = (omega_ref - omega) / tau
    omega += ((ref.omegaRef - omega) * o.dt) / o.omegaTauS;
    samples.push({
      t: Number(t.toFixed(4)),
      x: px,
      y: py,
      theta,
      v: o.speed,
      omega,
      omegaRef: ref.omegaRef,
      refX: ref.x,
      refY: ref.y,
    });
    px += o.speed * Math.cos(theta) * o.dt;
    py += o.speed * Math.sin(theta) * o.dt;
    theta += omega * o.dt;
  }
  return { samples, totalLengthM: total, durationS, laps: o.laps, speed: o.speed, dt: o.dt };
}

/**
 * GNSS/RTK 观测仿真。
 * 默认场景：RTK 固定解为主，含一段浮点解窗口（电离层/树荫）、一段完全丢星窗口（机库/树冠遮挡）
 * 以及 0.4% 的多路径野值。
 */
export function simulateGnssTrack(track, opts) {
  const o = Object.assign(
    {
      seed: 20240501,
      rateHz: 5,
      sigmaFixedM: 0.01,
      sigmaFloatM: 0.2,
      sigmaSingleM: 1.5,
      sigmaHeadingRad: (0.3 * Math.PI) / 180,
      outageWindows: null,
      floatWindows: null,
      outlierRate: 0.004,
      outlierMagnitudeM: [3, 9],
    },
    opts || {},
  );
  const rng = createRng(o.seed);
  const T = track.durationS;
  const floatWindows = o.floatWindows || [{ t0: T * 0.34, t1: T * 0.46 }];
  const outageWindows = o.outageWindows || [{ t0: T * 0.58, t1: T * 0.58 + 6.0 }];
  const inWindow = (t, ws) => ws.some((w) => t >= w.t0 && t < w.t1);

  const dtG = 1 / o.rateHz;
  const gnss = [];
  const heading = [];
  let outliers = 0;
  for (let t = 0; t <= T + 1e-9; t += dtG) {
    const idx = Math.min(track.samples.length - 1, Math.round(t / track.dt));
    const tr = track.samples[idx];
    // 双天线定向：遮挡期间同样失效
    if (!inWindow(t, outageWindows)) {
      heading.push({
        t: Number(t.toFixed(4)),
        theta: tr.theta + rng.normal(0, o.sigmaHeadingRad),
        sigmaRad: o.sigmaHeadingRad,
      });
    }
    let fix = 'rtk-fixed';
    if (inWindow(t, floatWindows)) fix = 'rtk-float';
    if (inWindow(t, outageWindows)) fix = 'none';
    if (fix === 'none') continue;
    const sigma = fix === 'rtk-float' ? o.sigmaFloatM : o.sigmaFixedM;
    let mx = tr.x + rng.normal(0, sigma);
    let my = tr.y + rng.normal(0, sigma);
    let outlier = false;
    if (rng.next() < o.outlierRate) {
      const m = rng.uniform(o.outlierMagnitudeM[0], o.outlierMagnitudeM[1]);
      const a = rng.uniform(0, 2 * Math.PI);
      mx += m * Math.cos(a);
      my += m * Math.sin(a);
      outlier = true;
      outliers += 1;
    }
    gnss.push({
      t: Number(t.toFixed(4)),
      x: mx,
      y: my,
      fix,
      sigmaM: sigma,
      hdop: fix === 'rtk-fixed' ? rng.uniform(0.6, 0.9) : rng.uniform(1.0, 1.8),
      satellites: fix === 'rtk-fixed' ? rng.int(22, 32) : rng.int(14, 21),
      outlier,
    });
  }
  return {
    gnss,
    heading,
    events: {
      floatWindows,
      outageWindows,
      injectedOutliers: outliers,
      truthOutlierCount: gnss.filter((g) => g.outlier).length,
    },
  };
}

/**
 * IMU 观测仿真：加速度计（纵向加速度 a）+ 陀螺仪（横摆角速度 omega）。
 * 含常值零偏与高斯白噪声；alpha 由陀螺差分得到（供 EKF 做控制输入）。
 */
export function simulateImu(track, opts) {
  const o = Object.assign(
    {
      seed: 8675309,
      rateHz: 50,
      accelNoise: 0.06,
      accelBias: 0.02,
      gyroNoiseRad: 0.004,
      // 陀螺零偏：ADIS16470 级别 MEMS IMU 的零偏不稳定性约 8 deg/h ~ 10 deg/h
      // 10 deg/h = 4.85e-5 rad/s
      gyroBiasRad: 4.85e-5,
      // 角加速度由陀螺差分得到，工程上需一阶低通（tau=0.05 s）抑制差分噪声
      alphaFilterTauS: 0.05,
    },
    opts || {},
  );
  const rng = createRng(o.seed);
  const dt = 1 / o.rateHz;
  const out = [];
  let prevOmega = null;
  let alphaFilt = 0;
  for (let t = 0; t <= track.durationS + 1e-9; t += dt) {
    const i = Math.min(track.samples.length - 1, Math.round(t / track.dt));
    const tr = track.samples[i];
    const i0 = Math.max(0, i - 1);
    const aTrue = (track.samples[i].v - track.samples[i0].v) / (track.dt * (i - i0 || 1));
    const omegaTrue = tr.omega;
    const omega = omegaTrue + o.gyroBiasRad + rng.normal(0, o.gyroNoiseRad);
    if (prevOmega === null) {
      alphaFilt = 0;
    } else {
      const raw = (omega - prevOmega) / dt;
      alphaFilt += ((raw - alphaFilt) * dt) / o.alphaFilterTauS;
    }
    const alpha = alphaFilt;
    prevOmega = omega;
    out.push({
      t: Number(t.toFixed(4)),
      ax: aTrue + o.accelBias + rng.normal(0, o.accelNoise),
      omega,
      alpha,
      speed: tr.v + rng.normal(0, 0.02),
      trueAx: aTrue,
      trueOmega: omegaTrue,
    });
  }
  return out;
}

/* =========================================================================
 * 6. 合成农田影像（供视觉杂草识别模块使用）
 * ========================================================================= */

function drawBlob(img, cx, cy, rx, ry, color, labelValue, rng, softness) {
  const x0 = Math.max(0, Math.floor(cx - rx - 1));
  const x1 = Math.min(img.width - 1, Math.ceil(cx + rx + 1));
  const y0 = Math.max(0, Math.floor(cy - ry - 1));
  const y1 = Math.min(img.height - 1, Math.ceil(cy + ry + 1));
  for (let y = y0; y <= y1; y += 1) {
    for (let x = x0; x <= x1; x += 1) {
      const dx = (x - cx) / rx;
      const dy = (y - cy) / ry;
      const r2 = dx * dx + dy * dy;
      if (r2 > 1) continue;
      // 边缘软化：接近边界处按概率落点，模拟叶片边缘与光照过渡
      const edge = 1 - Math.sqrt(r2);
      if (edge < softness && rng.next() > edge / softness) continue;
      const k = (y * img.width + x) * 3;
      img.rgb[k] = color[0];
      img.rgb[k + 1] = color[1];
      img.rgb[k + 2] = color[2];
      img.label[y * img.width + x] = labelValue;
    }
  }
}

/**
 * 生成合成农田影像：土壤背景 + 行播作物 + 随机杂草。
 * 输出像素级真值标签（0=土壤 1=作物 2=杂草），用于评估识别精确率/召回率。
 */
export function makeSyntheticFieldImage(opts) {
  const o = Object.assign(
    {
      widthPx: 640,
      heightPx: 480,
      mPerPx: 0.02,
      rowSpacingM: 0.6,
      cropPlantSpacingM: 0.22,
      cropRadiusM: 0.09,
      weedDensityPerM2: 0.8,
      weedRadiusMinM: 0.06,
      weedRadiusMaxM: 0.25,
      inRowWeedRatio: 0.25,
      soilColor: [122, 96, 70],
      cropColor: [58, 138, 48],
      weedColor: [86, 152, 58],
      noiseSigma: 5,
      illumination: 0.18,
      seed: 12345,
    },
    opts || {},
  );
  const rng = createRng(o.seed);
  const W = o.widthPx;
  const H = o.heightPx;
  const img = { width: W, height: H, mPerPx: o.mPerPx, rgb: new Uint8Array(W * H * 3), label: new Uint8Array(W * H) };
  const tileW = W * o.mPerPx;
  const tileH = H * o.mPerPx;

  // 1) 土壤背景 + 光照梯度 + 噪声
  for (let y = 0; y < H; y += 1) {
    for (let x = 0; x < W; x += 1) {
      const grad = 1 + o.illumination * (0.5 - (x / W) * 0.4 - (y / H) * 0.6);
      const k = (y * W + x) * 3;
      img.rgb[k] = clamp255(o.soilColor[0] * grad + rng.normal(0, o.noiseSigma));
      img.rgb[k + 1] = clamp255(o.soilColor[1] * grad + rng.normal(0, o.noiseSigma));
      img.rgb[k + 2] = clamp255(o.soilColor[2] * grad + rng.normal(0, o.noiseSigma));
      img.label[y * W + x] = 0;
    }
  }

  // 2) 规则行播作物：行中心线沿 x 方向，行距 rowSpacingM
  const rowSpacingPx = o.rowSpacingM / o.mPerPx;
  const cropRadiusPx = o.cropRadiusM / o.mPerPx;
  const plantSpacingPx = o.cropPlantSpacingM / o.mPerPx;
  const rowCentersPx = [];
  for (let yc = rowSpacingPx / 2; yc < H; yc += rowSpacingPx) rowCentersPx.push(yc);
  let cropPlants = 0;
  for (const yc of rowCentersPx) {
    for (let xc = plantSpacingPx / 2; xc < W; xc += plantSpacingPx) {
      // 播种横向/纵向落籽偏差：实际播种机纵向（沿行）偏差大于横向
      const jx = rng.normal(0, 1.2);
      const jy = rng.normal(0, 1.0);
      const s = rng.uniform(0.85, 1.15);
      drawBlob(
        img,
        xc + jx,
        yc + jy,
        cropRadiusPx * 1.25 * s,
        cropRadiusPx * 0.95 * s,
        o.cropColor,
        1,
        rng,
        0.35,
      );
      cropPlants += 1;
    }
  }

  // 3) 随机杂草：约 70% 行间、30% 行内（行内杂草更难与作物区分，属真实难点）
  const tileAreaM2 = tileW * tileH;
  const weedCount = Math.max(1, Math.round(o.weedDensityPerM2 * tileAreaM2));
  const weeds = [];
  let guard = 0;
  while (weeds.length < weedCount && guard < weedCount * 40) {
    guard += 1;
    const x = rng.uniform(0.15, tileW - 0.15);
    const inRow = rng.next() < o.inRowWeedRatio;
    let y;
    if (inRow) {
      y = rng.pick(rowCentersPx) * o.mPerPx + rng.normal(0, 0.05);
    } else {
      // 行间：距最近行中心至少 0.16 m，保证真值上不与作物行带混淆
      const k = rng.int(0, Math.max(0, rowCentersPx.length - 2));
      const yTop = rowCentersPx[k] * o.mPerPx;
      const yBot = rowCentersPx[Math.min(rowCentersPx.length - 1, k + 1)] * o.mPerPx;
      y = rng.uniform(yTop + 0.16, yBot - 0.16);
    }
    if (y < 0.1 || y > tileH - 0.1) continue;
    // 避免杂草之间过度重叠
    let ok = true;
    for (const w of weeds) {
      if (Math.hypot(w.x - x, w.y - y) < 0.14) {
        ok = false;
        break;
      }
    }
    if (!ok) continue;
    const r = rng.uniform(o.weedRadiusMinM, o.weedRadiusMaxM);
    weeds.push({ x, y, radiusM: r, inRow });
    // 杂草由 2~3 个不规则叶簇组成
    const lobes = rng.int(2, 3);
    for (let i = 0; i < lobes; i += 1) {
      const ax = x + rng.normal(0, r * 0.35);
      const ay = y + rng.normal(0, r * 0.35);
      const rr = r * rng.uniform(0.6, 1.0);
      drawBlob(img, ax / o.mPerPx, ay / o.mPerPx, rr / o.mPerPx, rr / o.mPerPx * rng.uniform(0.8, 1.2), o.weedColor, 2, rng, 0.4);
    }
  }

  let weedPx = 0;
  let cropPx = 0;
  for (let i = 0; i < img.label.length; i += 1) {
    if (img.label[i] === 2) weedPx += 1;
    else if (img.label[i] === 1) cropPx += 1;
  }
  return {
    ...img,
    truth: {
      weeds,
      weedCount: weeds.length,
      weedPixelCount: weedPx,
      cropPixelCount: cropPx,
      cropPlants,
      rowCentersPx,
      rowSpacingM: o.rowSpacingM,
      tileWidthM: tileW,
      tileHeightM: tileH,
      weedDensityPerM2: weeds.length / tileAreaM2,
    },
  };
}

function clamp255(v) {
  if (v < 0) return 0;
  if (v > 255) return 255;
  return v | 0;
}
