/**
 * src/control/pure_pursuit.mjs
 * ---------------------------------------------------------------------------
 * 纯跟踪（Pure Pursuit）路径跟踪控制器 + 曲率限速速度规划 + 闭环仿真
 *
 * 1) 纯跟踪几何
 *    在前视点 P_ld 上取路径点，车辆后轴中心到 P_ld 的距离为前视距离 L_d，
 *    车辆航向与 P_ld 方向夹角为 alpha，则前轮转角（自行车模型）：
 *        delta = atan2( 2 * L * sin(alpha), L_d )        L 为轴距
 *    该式可由"车辆沿半径 R = L_d / (2 sin alpha) 的圆弧通过 P_ld"推得，
 *    对纯圆弧路径稳态横向误差为零。
 *
 * 2) 自适应前视距离
 *        L_d = clamp(k_v * v + L_min, L_min, L_max) / (1 + k_c * |kappa|)
 *    速度越高前视越远（抑制振荡），曲率越大前视越近（提高弯道跟踪精度）。
 *
 * 3) 速度规划（曲率限速 + 双向斜坡）
 *        v_lim(s) = min( v_max, sqrt( a_lat_max / |kappa(s)| ) )
 *        后向遍历：v[i] = min(v[i], sqrt(v[i+1]^2 + 2*a_dec*ds))
 *        前向遍历：v[i] = min(v[i], sqrt(v[i-1]^2 + 2*a_acc*ds))
 *
 * 4) 闭环仿真：车辆真实状态用自行车模型积分，控制器只使用
 *    GNSS/RTK + IMU 的 EKF 融合位姿（见 src/perception/gnss_fusion.mjs），
 *    包含 RTK 噪声、转向速率限制与执行延迟，统计横向误差 RMS/最大值。
 */

import { GnssImuFusion } from '../perception/gnss_fusion.mjs';
import { createRng } from '../sim/field_simulator.mjs';

/** 把折线路径重采样为等弧长间距（默认 0.2 m） */
export function resamplePath(points, stepM) {
  const step = stepM === undefined ? 0.2 : stepM;
  const out = [];
  if (!points.length) return out;
  let prev = points[0];
  out.push({ x: prev.x, y: prev.y, s: 0, type: prev.type || 'work' });
  let sPrev = 0; // 上一个输出点的累计弧长
  let sCur = 0; // 当前折线段的起点弧长
  for (let i = 1; i < points.length; i += 1) {
    const b = points[i];
    const segLen = Math.hypot(b.x - prev.x, b.y - prev.y);
    if (segLen < 1e-12) continue;
    const ux = (b.x - prev.x) / segLen;
    const uy = (b.y - prev.y) / segLen;
    while (sPrev + step <= sCur + segLen + 1e-9) {
      const target = sPrev + step;
      const d = target - sCur;
      out.push({ x: prev.x + ux * d, y: prev.y + uy * d, s: target, type: b.type || prev.type || 'work' });
      sPrev = target;
    }
    sCur += segLen;
    prev = b;
  }
  return out;
}

/** 折线曲率估计（三点外接圆，窗口 w 平滑） */
export function pathCurvature(points, windowM) {
  const n = points.length;
  const w = windowM === undefined ? 1.0 : windowM;
  const kappa = new Float64Array(n);
  for (let i = 0; i < n; i += 1) {
    // 取前后各 w 米的点
    let i0 = i;
    let i1 = i;
    while (i0 > 0 && points[i].s - points[i0].s < w) i0 -= 1;
    while (i1 < n - 1 && points[i1].s - points[i].s < w) i1 += 1;
    const a = points[i0];
    const b = points[i];
    const c = points[i1];
    const ab = Math.hypot(b.x - a.x, b.y - a.y);
    const bc = Math.hypot(c.x - b.x, c.y - b.y);
    const ca = Math.hypot(a.x - c.x, a.y - c.y);
    const area2 = Math.abs((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y));
    kappa[i] = ab * bc * ca > 1e-9 ? (2 * area2) / (ab * bc * ca) : 0;
  }
  return kappa;
}

/**
 * 速度规划：曲率限速 + 前后向加减速斜坡。
 * @returns {{v:Float64Array, vLimit:Float64Array, maxLatAccelMps2:number}}
 */
export function planSpeedProfile(points, opts) {
  const o = Object.assign(
    {
      vMaxMps: 3.0,
      vMinMps: 0.3,
      aLatMaxMps2: 1.5,
      aAccelMps2: 0.8,
      aDecelMps2: 1.2,
      curvatureWindowM: 1.0,
      // 转向速率约束：前轮转角变化率上限 (rad/s) 与转向机构响应速度对应
      steerRateRadPerS: (25 * Math.PI) / 180,
      wheelbaseM: 2.6,
      // 曲率变化率 d(kappa)/ds 的平滑窗口 (m)
      curvatureRateWindowM: 2.0,
      // 地头转弯限速：|kappa| 超过阈值即按转弯限速行驶（工程取值 1.6 m/s ~ 5.8 km/h）
      curvatureTurnThreshold: 0.08,
      vTurnMaxMps: 1.6,
    },
    opts || {},
  );
  const n = points.length;
  const kappa = pathCurvature(points, o.curvatureWindowM);
  // 曲率变化率 d(kappa)/ds：在前视窗口内取平均，用于转向速率约束
  const wr = o.curvatureRateWindowM;
  const dkds = new Float64Array(n);
  for (let i = 0; i < n; i += 1) {
    let j0 = i;
    let j1 = i;
    while (j0 > 0 && points[i].s - points[j0].s < wr) j0 -= 1;
    while (j1 < n - 1 && points[j1].s - points[i].s < wr) j1 += 1;
    const dsSpan = Math.max(1e-6, points[j1].s - points[j0].s);
    dkds[i] = Math.abs(kappa[j1] - kappa[j0]) / dsSpan;
  }
  const vLimit = new Float64Array(n);
  const vSteer = new Float64Array(n);
  const v = new Float64Array(n);
  for (let i = 0; i < n; i += 1) {
    const k = Math.max(Math.abs(kappa[i]), 1e-6);
    const vk = Math.sqrt(o.aLatMaxMps2 / k);
    // 转向速率约束：d(delta)/dt = omega_delta 且 d(delta)/ds ~ L*d(kappa)/ds
    //   => v <= omega_delta / (L * |d kappa / ds|)
    // 该约束决定了进入/退出圆弧时（曲率阶跃）必须提前减速，否则转向执行跟不上
    const vSteering = dkds[i] > 1e-6 ? o.steerRateRadPerS / (o.wheelbaseM * dkds[i]) : Infinity;
    vSteer[i] = vSteering;
    // 转弯限速：曲率较大处（转弯段）统一按地头转弯速度行驶
    const vTurn = Math.abs(kappa[i]) > o.curvatureTurnThreshold ? o.vTurnMaxMps : Infinity;
    vLimit[i] = Math.min(o.vMaxMps, Math.max(o.vMinMps, Math.min(vk, vSteering, vTurn)));
    v[i] = vLimit[i];
  }
  // 后向遍历：保证能在限速点前减速
  for (let i = n - 2; i >= 0; i -= 1) {
    const ds = Math.max(1e-6, points[i + 1].s - points[i].s);
    v[i] = Math.min(v[i], Math.sqrt(v[i + 1] * v[i + 1] + 2 * o.aDecelMps2 * ds));
  }
  // 前向遍历：受加速度限制
  for (let i = 1; i < n; i += 1) {
    const ds = Math.max(1e-6, points[i].s - points[i - 1].s);
    v[i] = Math.min(v[i], Math.sqrt(v[i - 1] * v[i - 1] + 2 * o.aAccelMps2 * ds));
  }
  for (let i = 0; i < n; i += 1) v[i] = Math.max(o.vMinMps, Math.min(o.vMaxMps, v[i]));
  return { v, vLimit, kappa, dkds, vSteer, params: o };
}

/** 车辆位姿到折线路径的最近点与横向误差（带索引提示，O(1) 摊销） */
export function crossTrackError(pose, points, hint) {
  const n = points.length;
  const from = Math.max(0, (hint || 0) - 60);
  const to = Math.min(n - 1, (hint || 0) + 240);
  let bestD = Infinity;
  let bestI = from;
  for (let i = from; i <= to; i += 1) {
    const d = (points[i].x - pose.x) * (points[i].x - pose.x) + (points[i].y - pose.y) * (points[i].y - pose.y);
    if (d < bestD) {
      bestD = d;
      bestI = i;
    }
  }
  // 在最近顶点相邻的两段上做正交投影，得到真实的垂距（避免"最近顶点距离"带来的锯齿误差）
  let best = null;
  for (let k = Math.max(from, bestI - 1); k <= Math.min(n - 2, bestI + 1); k += 1) {
    const a = points[k];
    const b = points[k + 1];
    const dx = b.x - a.x;
    const dy = b.y - a.y;
    const L2 = dx * dx + dy * dy;
    let t = L2 > 1e-12 ? ((pose.x - a.x) * dx + (pose.y - a.y) * dy) / L2 : 0;
    t = Math.max(0, Math.min(1, t));
    const px = a.x + t * dx;
    const py = a.y + t * dy;
    const d2 = (pose.x - px) * (pose.x - px) + (pose.y - py) * (pose.y - py);
    if (!best || d2 < best.d2) {
      const cross = dx * (pose.y - a.y) - dy * (pose.x - a.x);
      best = { d2, errorM: (cross >= 0 ? 1 : -1) * Math.sqrt(d2), index: k, s: a.s + t * (b.s - a.s) };
    }
  }
  if (!best) {
    const p = points[bestI];
    return { errorM: Math.hypot(pose.x - p.x, pose.y - p.y), absErrorM: Math.hypot(pose.x - p.x, pose.y - p.y), index: bestI, s: p.s };
  }
  return { errorM: best.errorM, absErrorM: Math.abs(best.errorM), index: best.index, s: best.s };
}

export class PurePursuitController {
  /**
   * @param {object} opts
   *   wheelbaseM        轴距 (m)
   *   maxSteerRad       最大前轮转角 (rad)
   *   steerRateDegPerS  转向速率上限 (deg/s)
   *   steerLagS         转向执行一阶滞后 (s)
   *   lookaheadMinM     最小前视距离 (m)
   *   lookaheadMaxM     最大前视距离 (m)
   *   lookaheadGainS    前视距离速度增益 (s)
   *   lookaheadCurvGain 前视距离曲率衰减增益
   */
  constructor(opts) {
    const o = Object.assign(
      {
        wheelbaseM: 2.6,
        maxSteerRad: (35 * Math.PI) / 180,
        steerRateDegPerS: 25,
        steerLagS: 0.1,
        lookaheadMinM: 2.0,
        lookaheadMaxM: 6.0,
        // 前视距离速度增益：整定扫描表明本仿真条件下 KV 越小跟踪越紧（0.4 -> RMS 1.7 cm），
        // 但过小会放大位姿野值与执行延迟的影响，取 0.6 作为精度与鲁棒性的折中
        lookaheadGainS: 0.6,
        lookaheadCurvGain: 2.5,
        vMaxMps: 3.0,
        aLatMaxMps2: 1.5,
        aAccelMps2: 0.8,
        aDecelMps2: 1.2,
        curvatureWindowM: 1.0,
        curvatureRateWindowM: 2.0,
        curvatureTurnThreshold: 0.08,
        vTurnMaxMps: 1.6,
      },
      opts || {},
    );
    this.opt = o;
    this.points = [];
    this.speed = null;
  }

  /** 设定路径（含重采样与速度规划） */
  setPath(points) {
    this.points = resamplePath(points, 0.2);
    this.speed = planSpeedProfile(this.points, {
      vMaxMps: this.opt.vMaxMps,
      aLatMaxMps2: this.opt.aLatMaxMps2,
      aAccelMps2: this.opt.aAccelMps2,
      aDecelMps2: this.opt.aDecelMps2,
      curvatureWindowM: this.opt.curvatureWindowM,
      steerRateRadPerS: (this.opt.steerRateDegPerS * Math.PI) / 180,
      wheelbaseM: this.opt.wheelbaseM,
      curvatureRateWindowM: this.opt.curvatureRateWindowM,
      curvatureTurnThreshold: this.opt.curvatureTurnThreshold,
      vTurnMaxMps: this.opt.vTurnMaxMps,
    });
    this.steerCmd = 0;
    this.steerActual = 0;
    this.indexHint = 0;
    return this;
  }

  /** 自适应前视距离 */
  lookaheadDistance(v, curvature) {
    const o = this.opt;
    const base = Math.min(o.lookaheadMaxM, Math.max(o.lookaheadMinM, o.lookaheadGainS * Math.abs(v) + o.lookaheadMinM));
    const k = Math.abs(curvature || 0);
    const scaled = base / (1 + o.lookaheadCurvGain * k);
    return Math.min(o.lookaheadMaxM, Math.max(o.lookaheadMinM * 0.8, scaled));
  }

  /** 前视点在路径上的索引（按弧长搜索） */
  lookaheadPoint(pose, ld) {
    const pts = this.points;
    const cte = crossTrackError(pose, pts, this.indexHint);
    this.indexHint = cte.index;
    const sTarget = cte.s + ld;
    let i = cte.index;
    while (i < pts.length - 1 && pts[i].s < sTarget) i += 1;
    return { point: pts[i], cte };
  }

  /** 计算前轮转角（含转向速率限制） */
  computeSteering(pose, v, dt) {
    const o = this.opt;
    const kappaHere = this.speed ? this.speed.kappa[this.indexHint] || 0 : 0;
    const ld = this.lookaheadDistance(v, kappaHere);
    const { point, cte } = this.lookaheadPoint(pose, ld);
    const dx = point.x - pose.x;
    const dy = point.y - pose.y;
    let alpha = Math.atan2(dy, dx) - pose.theta;
    while (alpha > Math.PI) alpha -= 2 * Math.PI;
    while (alpha < -Math.PI) alpha += 2 * Math.PI;
    const dist = Math.hypot(dx, dy);
    let delta = Math.atan2(2 * o.wheelbaseM * Math.sin(alpha), Math.max(0.5, dist));
    delta = Math.max(-o.maxSteerRad, Math.min(o.maxSteerRad, delta));
    this.steerCmd = delta;
    // 转向执行模型：一阶滞后（电液比例阀响应）+ 转向速率上限（前轮转角速率）
    const maxRate = (o.steerRateDegPerS * Math.PI) / 180; // rad/s
    const lagGain = Math.min(1, dt / Math.max(o.steerLagS, 1e-6)); // 一阶惯性系数
    let next = this.steerActual + (delta - this.steerActual) * lagGain;
    const stepLimit = maxRate * dt;
    const step = next - this.steerActual;
    if (Math.abs(step) > stepLimit) next = this.steerActual + Math.sign(step) * stepLimit;
    this.steerActual = next;
    return {
      deltaCmdRad: delta,
      deltaRad: this.steerActual,
      lookaheadM: ld,
      alphaRad: alpha,
      targetPoint: point,
      crossTrackM: cte.errorM,
    };
  }

  /** 目标速度（按当前位置查速度曲线） */
  targetSpeed() {
    if (!this.speed) return this.opt.vMaxMps;
    return this.speed.v[Math.min(this.speed.v.length - 1, this.indexHint)];
  }
}

/**
 * 闭环跟踪仿真：真实车辆用自行车模型积分，控制器使用 EKF 融合位姿。
 * @param {Array} path 规划路径点（含 type 字段）
 * @param {object} opts {dt, maxTimeS, startIndex, simLengthM, useFusion, seed, ...控制器参数}
 */
export function simulateTracking(path, opts) {
  const o = Object.assign(
    {
      dt: 0.02,
      maxTimeS: 150,
      useFusion: true,
      seed: 4001,
      gnssRateHz: 5,
      recordEveryS: 0.5,
    },
    opts || {},
  );
  const ctrl = new PurePursuitController(o);
  // 截取仿真路径段（默认整条路径，调用方可指定长度限制）
  let seg = path;
  if (o.simLengthM && path.length > 2) {
    const start = o.startIndex || 0;
    const s0 = path[start] && path[start].s !== undefined ? path[start].s : 0;
    let end = start;
    const targetS = s0 + o.simLengthM;
    while (end < path.length - 1 && (path[end].s === undefined ? end - start : path[end].s - s0) < targetS) end += 1;
    seg = path.slice(start, end + 1);
    if (seg.length < 2) seg = path;
  }
  ctrl.setPath(seg);
  const pts = ctrl.points;
  const rng = createRng(o.seed);
  const fusion = new GnssImuFusion({
    initialPose: { x: pts[0].x, y: pts[0].y, theta: Math.atan2(pts[1].y - pts[0].y, pts[1].x - pts[0].x), v: 0 },
    initialSigmaM: 0.5,
  });

  // 真实车辆状态
  let state = {
    x: pts[0].x,
    y: pts[0].y,
    theta: Math.atan2(pts[1].y - pts[0].y, pts[1].x - pts[0].x),
    v: 0,
  };
  const dt = o.dt;
  const gnssEvery = Math.round(1 / o.gnssRateHz / dt);
  const series = [];
  let sumErr2 = 0;
  let sumAbsErr = 0;
  let maxErr = 0;
  let nErr = 0;
  let sumHeadingErr2 = 0;
  let sumSpeed = 0;
  let maxSpeed = 0;
  let steerSaturationSteps = 0;
  const t0 = Date.now();
  const steps = Math.floor(o.maxTimeS / dt);
  let travelled = 0;
  let prevX = state.x;
  let prevY = state.y;

  for (let k = 0; k < steps; k += 1) {
    const t = k * dt;
    // ---- 控制器（使用融合位姿） ----
    const est = fusion.getPose();
    const ctl = ctrl.computeSteering(est, est.v, dt);
    if (Math.abs(ctl.deltaCmdRad) >= ctrl.opt.maxSteerRad - 1e-6) steerSaturationSteps += 1;
    const vTarget = ctrl.targetSpeed();
    // ---- 速度控制：受加减速限制 ----
    const dv = vTarget - state.v;
    const dvMax = (dv > 0 ? o.aAccelMps2 || 0.8 : o.aDecelMps2 || 1.2) * dt;
    const vOld = state.v;
    state.v += Math.abs(dv) > dvMax ? Math.sign(dv) * dvMax : dv;
    if (state.v < 0.05) state.v = 0.05;
    const axTrue = (state.v - vOld) / dt;
    // ---- 车辆运动学（自行车模型） ----
    state.x += state.v * Math.cos(state.theta) * dt;
    state.y += state.v * Math.sin(state.theta) * dt;
    state.theta += ((state.v / ctrl.opt.wheelbaseM) * Math.tan(ctl.deltaRad)) * dt;
    travelled += Math.hypot(state.x - prevX, state.y - prevY);
    prevX = state.x;
    prevY = state.y;

    // ---- 传感器仿真 + 融合 ----
    const ax = axTrue + rng.normal(0, 0.06); // 加速度计：真实纵向加速度 + 噪声
    const omega = ((state.v / ctrl.opt.wheelbaseM) * Math.tan(ctl.deltaRad));
    const gnss = k % gnssEvery === 0
      ? {
          x: state.x + rng.normal(0, 0.01),
          y: state.y + rng.normal(0, 0.01),
          fix: 'rtk-fixed',
          sigmaM: 0.01,
          hdop: 0.8,
        }
      : null;
    const heading = k % gnssEvery === 0 ? { theta: state.theta + rng.normal(0, (0.3 * Math.PI) / 180), sigmaRad: (0.3 * Math.PI) / 180 } : null;
    fusion.step({
      dt,
      ax,
      alpha: rng.normal(0, 0.3),
      omega: omega + rng.normal(0, 0.004),
      speed: state.v + rng.normal(0, 0.02),
      gnss,
      heading,
    });

    // ---- 误差统计（相对真实路径） ----
    const cteTrue = crossTrackError(state, pts, ctrl.indexHint);
    sumErr2 += cteTrue.errorM * cteTrue.errorM;
    sumAbsErr += Math.abs(cteTrue.errorM);
    if (Math.abs(cteTrue.errorM) > maxErr) maxErr = Math.abs(cteTrue.errorM);
    const idx = Math.min(pts.length - 1, cteTrue.index + 1);
    const pathHeading = Math.atan2(pts[idx].y - pts[cteTrue.index].y, pts[idx].x - pts[cteTrue.index].x);
    let hErr = state.theta - pathHeading;
    while (hErr > Math.PI) hErr -= 2 * Math.PI;
    while (hErr < -Math.PI) hErr += 2 * Math.PI;
    sumHeadingErr2 += hErr * hErr;
    nErr += 1;
    sumSpeed += state.v;
    if (state.v > maxSpeed) maxSpeed = state.v;
    if (k % Math.round(o.recordEveryS / dt) === 0) {
      series.push({
        t: Number(t.toFixed(2)),
        x: state.x,
        y: state.y,
        theta: state.theta,
        v: state.v,
        vTarget,
        crossTrackM: cteTrue.errorM,
        headingErrDeg: (hErr * 180) / Math.PI,
        steerDeg: (ctl.deltaRad * 180) / Math.PI,
        lookaheadM: ctl.lookaheadM,
      });
    }
    if (cteTrue.index >= pts.length - 2 && travelled > 10) break;
  }
  const simTimeS = series.length ? series[series.length - 1].t : 0;
  return {
    controller: {
      wheelbaseM: ctrl.opt.wheelbaseM,
      lookaheadMinM: ctrl.opt.lookaheadMinM,
      lookaheadMaxM: ctrl.opt.lookaheadMaxM,
      lookaheadGainS: ctrl.opt.lookaheadGainS,
      maxSteerDeg: (ctrl.opt.maxSteerRad * 180) / Math.PI,
    },
    pathLengthM: pts.length ? pts[pts.length - 1].s : 0,
    travelledM: travelled,
    simTimeS,
    rmsCrossTrackM: nErr ? Math.sqrt(sumErr2 / nErr) : 0,
    meanAbsCrossTrackM: nErr ? sumAbsErr / nErr : 0,
    maxCrossTrackM: maxErr,
    rmsHeadingErrDeg: nErr ? (Math.sqrt(sumHeadingErr2 / nErr) * 180) / Math.PI : 0,
    meanSpeedMps: nErr ? sumSpeed / nErr : 0,
    maxSpeedMps: maxSpeed,
    steerSaturationRatio: nErr ? steerSaturationSteps / nErr : 0,
    computeMs: Date.now() - t0,
    series,
    fusionDiagnostics: fusion.getDiagnostics(),
  };
}
