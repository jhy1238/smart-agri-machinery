/**
 * src/perception/gnss_fusion.mjs
 * ---------------------------------------------------------------------------
 * GNSS/RTK + IMU 扩展卡尔曼滤波（EKF）定位融合
 *
 * 状态向量   x = [x, y, theta, v, omega]^T        （位置 m、航向 rad、速度 m/s、横摆角速度 rad/s）
 * 控制输入   u = [a, alpha]^T                     （纵向加速度 m/s^2、角加速度 rad/s^2）
 *
 * 运动模型（CTRV 恒速恒角速度 + 纵向加速度，Euler 离散）：
 *   x+     = x + (v/omega) * [ sin(theta + omega*dt) - sin(theta) ]
 *   y+     = y + (v/omega) * [ cos(theta) - cos(theta + omega*dt) ]
 *   theta+ = theta + omega*dt
 *   v+     = v + a*dt
 *   omega+ = omega + alpha*dt
 *   |omega| < 1e-4 时退化为直线模型：x+ = x + v*cos(theta)*dt，y+ = y + v*sin(theta)*dt
 *
 * 过程噪声  Q = G * diag(sigma_a^2, sigma_alpha^2) * G^T，
 *           G = [ dt^2/2*cos(theta), 0 ; dt^2/2*sin(theta), 0 ; 0 ; dt^2/2 ; dt ; dt ] 的转置形式
 *
 * 观测模型（多速率融合）：
 *   1) RTK 位置 (x, y)  —— 噪声按解算类型给定：固定解 0.010 m、浮点解 0.20 m、DGPS 0.50 m、单点 1.50 m
 *   2) 双天线定向 theta —— sigma = 0.3 deg = 5.24e-3 rad
 *   3) 轮速/雷达测速 v  —— sigma = 0.02 m/s
 *   4) 陀螺仪角速度 omega —— sigma = 0.004 rad/s
 *
 * 野值与丢星：
 *   - 用 NIS（归一化新息平方）做卡方门控，2 自由度 99% 分位 9.210，1 自由度 6.635；
 *   - 连续 > 2.0 s 无有效 GNSS 观测即进入航位推算（dead-reckoning）降级模式并膨胀过程噪声。
 */

import { buildLoopTruthTrack, simulateGnssTrack, simulateImu } from '../sim/field_simulator.mjs';

/* =========================================================================
 * 1. 基础数学工具
 * ========================================================================= */

/** 将角度归一化到 (-pi, pi] */
export function wrapAngle(a) {
  let x = a;
  while (x > Math.PI) x -= 2 * Math.PI;
  while (x <= -Math.PI) x += 2 * Math.PI;
  return x;
}

/** 两角之差（归一化到 (-pi, pi]），用于航向新息计算 */
export function angleDiff(a, b) {
  return wrapAngle(a - b);
}

export const CHI2_99 = { 1: 6.635, 2: 9.210, 3: 11.345 };

export function matIdentity(n) {
  const m = [];
  for (let i = 0; i < n; i += 1) {
    const row = new Array(n).fill(0);
    row[i] = 1;
    m.push(row);
  }
  return m;
}

export function matMul(A, B) {
  const n = A.length;
  const k = B.length;
  const p = B[0].length;
  const out = [];
  for (let i = 0; i < n; i += 1) {
    const row = new Array(p).fill(0);
    for (let j = 0; j < p; j += 1) {
      let s = 0;
      for (let t = 0; t < k; t += 1) s += A[i][t] * B[t][j];
      row[j] = s;
    }
    out.push(row);
  }
  return out;
}

export function matTranspose(A) {
  const out = [];
  for (let j = 0; j < A[0].length; j += 1) {
    const row = [];
    for (let i = 0; i < A.length; i += 1) row.push(A[i][j]);
    out.push(row);
  }
  return out;
}

export function matAdd(A, B) {
  return A.map((row, i) => row.map((v, j) => v + B[i][j]));
}

export function matSub(A, B) {
  return A.map((row, i) => row.map((v, j) => v - B[i][j]));
}

export function matScale(A, s) {
  return A.map((row) => row.map((v) => v * s));
}

export function matVec(A, v) {
  return A.map((row) => row.reduce((s, a, j) => s + a * v[j], 0));
}

/** 对称化：消除浮点误差导致的非对称 */
export function symmetrize(A) {
  const n = A.length;
  const out = [];
  for (let i = 0; i < n; i += 1) {
    const row = new Array(n).fill(0);
    for (let j = 0; j < n; j += 1) row[j] = (A[i][j] + A[j][i]) / 2;
    out.push(row);
  }
  return out;
}

/** 高斯-约当消元求逆（n <= 6，带部分主元） */
export function matInverse(A) {
  const n = A.length;
  const M = A.map((row, i) => row.concat(matIdentity(n)[i]));
  for (let col = 0; col < n; col += 1) {
    let piv = col;
    for (let r = col + 1; r < n; r += 1) if (Math.abs(M[r][col]) > Math.abs(M[piv][col])) piv = r;
    if (Math.abs(M[piv][col]) < 1e-12) throw new Error('matInverse: 矩阵奇异');
    const tmp = M[col];
    M[col] = M[piv];
    M[piv] = tmp;
    const p = M[col][col];
    for (let j = 0; j < 2 * n; j += 1) M[col][j] /= p;
    for (let r = 0; r < n; r += 1) {
      if (r === col) continue;
      const f = M[r][col];
      if (f === 0) continue;
      for (let j = 0; j < 2 * n; j += 1) M[r][j] -= f * M[col][j];
    }
  }
  return M.map((row) => row.slice(n));
}

/** 二次型 v^T A v */
export function quadForm(v, A) {
  let s = 0;
  for (let i = 0; i < v.length; i += 1) {
    for (let j = 0; j < v.length; j += 1) s += v[i] * A[i][j] * v[j];
  }
  return s;
}

/** 对角矩阵 */
export function matDiag(vals) {
  const n = vals.length;
  const m = [];
  for (let i = 0; i < n; i += 1) {
    const row = new Array(n).fill(0);
    row[i] = vals[i];
    m.push(row);
  }
  return m;
}

/* =========================================================================
 * 2. 通用扩展卡尔曼滤波器
 * ========================================================================= */

export class ExtendedKalmanFilter {
  /**
   * @param {object} opts
   * @param {number[]} opts.state       初始状态 [x,y,theta,v,omega]
   * @param {number[][]} opts.covariance 初始协方差 P (5x5)
   * @param {number} opts.sigmaA        纵向加速度噪声标准差 (m/s^2)
   * @param {number} opts.sigmaAlpha    角加速度噪声标准差 (rad/s^2)
   */
  constructor(opts) {
    this.x = opts.state.slice();
    this.P = opts.covariance.map((r) => r.slice());
    this.sigmaA = opts.sigmaA === undefined ? 0.25 : opts.sigmaA;
    this.sigmaAlpha = opts.sigmaAlpha === undefined ? 0.08 : opts.sigmaAlpha;
    this.inflation = 1; // 降级模式下的过程噪声膨胀系数
    this.rejectedCount = 0;
    this.acceptedCount = 0;
    this.lastNis = 0;
    this.nisHistory = [];
    // 滤波一致性保护：连续 N 次被 NIS 门控拒绝，说明 P 已过度收敛（不一致），
    // 此时按 recoveryInflation 膨胀 P 并强制接纳该观测，避免滤波器"锁死"在错误状态。
    this.consecutiveRejections = 0;
    this.maxConsecutiveRejections = 3;
    this.recoveryInflation = 25;
    this.forcedRecoveries = 0;
  }

  /** 状态预测：x+ = f(x,u)，P+ = F P F^T + Q */
  predict(dt, control) {
    const c = control || {};
    const a = c.a || 0;
    const alpha = c.alpha || 0;
    const th = this.x[2];
    const v = this.x[3];
    const w = this.x[4];
    const F = matIdentity(5);
    let xn;
    if (Math.abs(w) > 1e-4) {
      const s = Math.sin(th);
      const co = Math.cos(th);
      const th2 = th + w * dt;
      const s2 = Math.sin(th2);
      const c2 = Math.cos(th2);
      xn = [this.x[0] + (v / w) * (s2 - s), this.x[1] + (v / w) * (co - c2), wrapAngle(th2), v + a * dt, w + alpha * dt];
      // 雅可比 F = df/dx
      F[0][2] = (v / w) * (c2 - co);
      F[0][3] = (s2 - s) / w;
      F[0][4] = v * ((c2 * dt * w - (s2 - s)) / (w * w));
      F[1][2] = (v / w) * (s2 - s);
      F[1][3] = (co - c2) / w;
      F[1][4] = v * ((s2 * dt * w - (co - c2)) / (w * w));
      F[2][4] = dt;
    } else {
      const s = Math.sin(th);
      const co = Math.cos(th);
      xn = [this.x[0] + v * co * dt, this.x[1] + v * s * dt, wrapAngle(th + w * dt), v + a * dt, w + alpha * dt];
      F[0][2] = -v * s * dt;
      F[0][3] = co * dt;
      F[1][2] = v * co * dt;
      F[1][3] = s * dt;
      F[2][4] = dt;
    }
    // 过程噪声
    const s = Math.sin(th);
    const co = Math.cos(th);
    const G = [
      [0.5 * dt * dt * co, 0],
      [0.5 * dt * dt * s, 0],
      [0, 0.5 * dt * dt],
      [dt, 0],
      [0, dt],
    ];
    const Qn = matDiag([this.sigmaA * this.sigmaA, this.sigmaAlpha * this.sigmaAlpha]);
    const Q = matScale(matMul(matMul(G, Qn), matTranspose(G)), this.inflation);
    this.x = xn;
    this.P = symmetrize(matAdd(matMul(matMul(F, this.P), matTranspose(F)), Q));
    return this.x.slice();
  }

  /**
   * 观测更新：z = h(x) + v，v ~ N(0, R)
   * @param {number[]} z 观测向量
   * @param {number[][]} H 观测雅可比
   * @param {number[][]} R 观测噪声协方差
   * @param {object} opts {gate:boolean, chi2:number, wrapIdx:number[], label:string}
   */
  update(z, H, R, opts) {
    const o = Object.assign({ gate: true, chi2: CHI2_99[z.length] || 9.21, wrapIdx: [], label: 'obs' }, opts || {});
    const Hx = matVec(H, this.x);
    const nu = z.map((zi, i) => (o.wrapIdx.includes(i) ? angleDiff(zi, Hx[i]) : zi - Hx[i]));
    const S = matAdd(matMul(matMul(H, this.P), matTranspose(H)), R);
    const Sinv = matInverse(S);
    const nis = quadForm(nu, Sinv);
    this.lastNis = nis;
    this.nisHistory.push({ label: o.label, nis });
    if (this.nisHistory.length > 400) this.nisHistory.splice(0, 200);
    if (o.gate && nis > o.chi2) {
      this.rejectedCount += 1;
      this.consecutiveRejections += 1;
      if (this.consecutiveRejections < this.maxConsecutiveRejections) {
        return { accepted: false, nis, innovation: nu };
      }
      // 一致性恢复：膨胀 P 后按正常增益进行更新
      this.P = matScale(this.P, this.recoveryInflation);
      this.forcedRecoveries += 1;
      this.consecutiveRejections = 0;
      const Sf = matAdd(matMul(matMul(H, this.P), matTranspose(H)), R);
      const Kf = matMul(matMul(this.P, matTranspose(H)), matInverse(Sf));
      const dxf = matVec(Kf, nu);
      this.x = this.x.map((xi, i) => xi + dxf[i]);
      this.x[2] = wrapAngle(this.x[2]);
      this.P = symmetrize(matMul(matSub(matIdentity(5), matMul(Kf, H)), this.P));
      this.acceptedCount += 1;
      return { accepted: true, nis, innovation: nu, forced: true };
    }
    this.consecutiveRejections = 0;
    const K = matMul(matMul(this.P, matTranspose(H)), Sinv);
    const dx = matVec(K, nu);
    this.x = this.x.map((xi, i) => xi + dx[i]);
    this.x[2] = wrapAngle(this.x[2]);
    this.P = symmetrize(matMul(matSub(matIdentity(5), matMul(K, H)), this.P));
    this.acceptedCount += 1;
    return { accepted: true, nis, innovation: nu, gain: K };
  }

  /** 当前位置标准差（由 P 的对角线得到） */
  sigmaPosition() {
    return { sigmaXM: Math.sqrt(Math.max(0, this.P[0][0])), sigmaYM: Math.sqrt(Math.max(0, this.P[1][1])) };
  }
}

/* =========================================================================
 * 3. GNSS/RTK + IMU 融合器
 * ========================================================================= */

/** 不同 GNSS 解算类型的水平位置标准差（1 sigma，单位 m） */
export const GNSS_FIX_SIGMA_M = {
  'rtk-fixed': 0.01,
  'rtk-float': 0.2,
  dgps: 0.5,
  single: 1.5,
  none: null,
};

export class GnssImuFusion {
  constructor(opts) {
    const o = Object.assign(
      {
        initialPose: { x: 0, y: 0, theta: 0, v: 0 },
        initialSigmaM: 1.0,
        // 过程噪声与 IMU 噪声模型匹配：加速度计噪声 0.06 + 零偏 0.02 (m/s^2)；
        // 角加速度经 0.05 s 低通后残余噪声约 0.3 rad/s^2
        sigmaA: 0.2,
        sigmaAlpha: 0.5,
        sigmaHeadingRad: (0.3 * Math.PI) / 180,
        sigmaSpeedMps: 0.02,
        sigmaGyroRad: 0.004,
        deadReckoningThresholdS: 2.0,
        degradationInflation: 4.0,
        gate: true,
      },
      opts || {},
    );
    this.opt = o;
    const P = matDiag([
      o.initialSigmaM * o.initialSigmaM,
      o.initialSigmaM * o.initialSigmaM,
      (5 * Math.PI) / 180 * ((5 * Math.PI) / 180),
      0.25,
      0.01,
    ]);
    this.ekf = new ExtendedKalmanFilter({
      state: [o.initialPose.x, o.initialPose.y, o.initialPose.theta, o.initialPose.v, 0],
      covariance: P,
      sigmaA: o.sigmaA,
      sigmaAlpha: o.sigmaAlpha,
    });
    this.time = 0;
    this.lastAcceptedSigmaM = null;
    this.lastGnssAcceptedT = -Infinity;
    this.mode = 'initializing';
    this.currentFix = 'none';
    this.outageStartT = null;
    this.maxOutageS = 0;
    this.stats = {
      gnssTotal: 0,
      gnssAccepted: 0,
      gnssRejectedOutlier: 0,
      gnssRejectedNis: 0,
      headingAccepted: 0,
      speedAccepted: 0,
      gyroAccepted: 0,
      deadReckoningS: 0,
      outlierFlags: 0,
    };
    this.log = [];
  }

  /** 用 IMU 控制量推演状态（50 Hz） */
  predictImu(imu, dt) {
    this.lastDt = dt;
    this.ekf.predict(dt, { a: imu.ax, alpha: imu.alpha });
    this.time += dt;
    return this.ekf.x.slice();
  }

  /** RTK 位置观测；按解算类型设定 R，并做 NIS 门控 */
  updateGnss(gnss) {
    this.stats.gnssTotal += 1;
    const base = GNSS_FIX_SIGMA_M[gnss.fix];
    this.currentFix = gnss.fix;
    if (base === null || base === undefined) return { accepted: false, reason: 'no-fix' };
    const hdop = gnss.hdop === undefined ? 1.0 : gnss.hdop;
    const sigma = base * (1 + 0.25 * Math.max(0, hdop - 1.0));
    // 观测质量突变自适应：固定解(0.01 m)切换到浮点解(0.20 m)时，滤波器仍处于
    // 高度自信状态，若不放宽自身协方差会持续 NIS 拒收并误入航位推算。
    // 按 (sigma_new/sigma_old)^2 膨胀位置协方差（上限 400 倍）。
    if (this.lastAcceptedSigmaM && sigma / this.lastAcceptedSigmaM > 3.0) {
      const f = Math.min(400, (sigma / this.lastAcceptedSigmaM) ** 2);
      this.ekf.P[0][0] *= f;
      this.ekf.P[1][1] *= f;
      this.log.push({ t: this.time, event: 'covariance-inflated-for-fix-change', factor: f, fix: gnss.fix });
    }
    const R = matDiag([sigma * sigma, sigma * sigma]);
    const H = [
      [1, 0, 0, 0, 0],
      [0, 1, 0, 0, 0],
    ];
    const res = this.ekf.update([gnss.x, gnss.y], H, R, { gate: this.opt.gate, chi2: CHI2_99[2], label: 'gnss' });
    if (res.accepted) {
      this.stats.gnssAccepted += 1;
      this.lastAcceptedSigmaM = sigma;
      this.lastGnssAcceptedT = this.time;
      if (this.outageStartT !== null) {
        const dur = this.time - this.outageStartT;
        if (dur > this.maxOutageS) this.maxOutageS = dur;
        this.log.push({ t: this.time, event: 'gnss-recovered', outageS: dur });
        this.outageStartT = null;
      }
      this.mode = gnss.fix;
      this.ekf.inflation = 1;
    } else {
      this.stats.gnssRejectedNis += 1;
      if (gnss.outlier) this.stats.outlierFlags += 1;
      this.log.push({ t: this.time, event: 'gnss-rejected', nis: res.nis, wasInjectedOutlier: !!gnss.outlier });
    }
    return res;
  }

  /** 双天线定向航向观测 */
  updateHeading(hdg) {
    const sigma = hdg.sigmaRad === undefined ? this.opt.sigmaHeadingRad : hdg.sigmaRad;
    const H = [[0, 0, 1, 0, 0]];
    const res = this.ekf.update([hdg.theta], H, matDiag([sigma * sigma]), {
      gate: this.opt.gate,
      chi2: CHI2_99[1],
      wrapIdx: [0],
      label: 'heading',
    });
    if (res.accepted) this.stats.headingAccepted += 1;
    return res;
  }

  /** 轮速 / 雷达测速观测 */
  updateSpeed(v) {
    const H = [[0, 0, 0, 1, 0]];
    const res = this.ekf.update([v], H, matDiag([this.opt.sigmaSpeedMps * this.opt.sigmaSpeedMps]), {
      gate: false,
      chi2: CHI2_99[1],
      label: 'speed',
    });
    if (res.accepted) this.stats.speedAccepted += 1;
    return res;
  }

  /** 陀螺仪横摆角速度观测 */
  updateGyro(omega) {
    const H = [[0, 0, 0, 0, 1]];
    const res = this.ekf.update([omega], H, matDiag([this.opt.sigmaGyroRad * this.opt.sigmaGyroRad]), {
      gate: false,
      chi2: CHI2_99[1],
      label: 'gyro',
    });
    if (res.accepted) this.stats.gyroAccepted += 1;
    return res;
  }

  /** 降级判定：连续丢星超过阈值即进入航位推算并膨胀过程噪声 */
  evaluateDegradation() {
    const dtSince = this.time - this.lastGnssAcceptedT;
    if (dtSince > this.opt.deadReckoningThresholdS) {
      if (this.mode !== 'dead-reckoning') {
        this.log.push({ t: this.time, event: 'enter-dead-reckoning', dtSinceGnssS: dtSince });
        this.outageStartT = this.lastGnssAcceptedT;
      }
      this.mode = 'dead-reckoning';
      this.ekf.inflation = this.opt.degradationInflation;
      this.stats.deadReckoningS += this.lastDt || 0;
    }
    return this.mode;
  }

  /** 统一入口：一次 50 Hz 融合步（IMU 预测 + 可用观测更新） */
  step(sample) {
    this.predictImu({ ax: sample.ax, alpha: sample.alpha }, sample.dt);
    if (sample.omega !== undefined && sample.omega !== null) this.updateGyro(sample.omega);
    if (sample.speed !== undefined && sample.speed !== null) this.updateSpeed(sample.speed);
    if (sample.gnss) this.updateGnss(sample.gnss);
    if (sample.heading) this.updateHeading(sample.heading);
    this.evaluateDegradation();
    return this.getPose();
  }

  getPose() {
    const x = this.ekf.x;
    const s = this.ekf.sigmaPosition();
    return { x: x[0], y: x[1], theta: x[2], v: x[3], omega: x[4], sigmaXM: s.sigmaXM, sigmaYM: s.sigmaYM, mode: this.mode };
  }

  getDiagnostics() {
    return {
      mode: this.mode,
      currentFix: this.currentFix,
      lastNis: this.ekf.lastNis,
      rejectedCount: this.ekf.rejectedCount,
      acceptedCount: this.ekf.acceptedCount,
      forcedRecoveries: this.ekf.forcedRecoveries,
      maxOutageS: this.maxOutageS,
      stats: { ...this.stats },
      events: this.log.slice(-20),
    };
  }
}

/* =========================================================================
 * 4. 端到端融合场景（真值轨迹 + 传感器仿真 + EKF + 精度评估）
 * ========================================================================= */

export function runFusionScenario(opts) {
  const o = Object.assign(
    {
      seed: 20240501,
      laps: 1,
      dt: 0.02,
      warmupS: 2.0,
      gnssRateHz: 5,
      imuRateHz: 50,
    },
    opts || {},
  );
  const track = buildLoopTruthTrack({ laps: o.laps, dt: o.dt, speed: 2.5 });
  const sensors = simulateGnssTrack(track, { seed: o.seed, rateHz: o.gnssRateHz });
  const imu = simulateImu(track, { seed: o.seed + 1, rateHz: o.imuRateHz });
  const dt = 1 / o.imuRateHz;

  const truthAt = (t) => track.samples[Math.min(track.samples.length - 1, Math.round(t / track.dt))];
  const first = track.samples[0];
  const fusion = new GnssImuFusion({
    initialPose: { x: first.x, y: first.y, theta: first.theta, v: first.v },
    initialSigmaM: 1.0,
  });

  const gnssByT = new Map();
  for (const g of sensors.gnss) gnssByT.set(Math.round(g.t / dt), g);
  const hdgByT = new Map();
  for (const h of sensors.heading) hdgByT.set(Math.round(h.t / dt), h);

  const series = [];
  for (let i = 0; i < imu.length; i += 1) {
    const s = imu[i];
    const gnss = gnssByT.get(i) || null;
    const hdg = hdgByT.get(i) || null;
    fusion.step({ dt, ax: s.ax, alpha: s.alpha, omega: s.omega, speed: s.speed, gnss, heading: hdg });
    const truth = truthAt(s.t);
    const pose = fusion.getPose();
    const err = Math.hypot(pose.x - truth.x, pose.y - truth.y);
    const hErr = angleDiff(pose.theta, truth.theta);
    series.push({
      t: Number(s.t.toFixed(3)),
      errM: err,
      headingErrRad: hErr,
      mode: pose.mode,
      fix: gnss ? gnss.fix : 'none',
      sigmaXM: pose.sigmaXM,
      truthX: truth.x,
      truthY: truth.y,
      x: pose.x,
      y: pose.y,
    });
  }

  const inWin = (t, ws) => ws.some((w) => t >= w.t0 && t < w.t1);
  const acc = { all: [], fixed: [], float: [], outage: [], recovery: [] };
  for (const p of series) {
    if (p.t < o.warmupS) continue;
    acc.all.push(p.errM);
    if (inWin(p.t, sensors.events.outageWindows)) acc.outage.push(p.errM);
    else if (inWin(p.t, sensors.events.floatWindows)) acc.float.push(p.errM);
    else acc.fixed.push(p.errM);
    const ow = sensors.events.outageWindows[0];
    if (ow && p.t >= ow.t1 && p.t < ow.t1 + 3.0) acc.recovery.push(p.errM);
  }
  const rms = (a) => (a.length ? Math.sqrt(a.reduce((s, v) => s + v * v, 0) / a.length) : 0);
  const maxOf = (a) => (a.length ? Math.max(...a) : 0);
  const diag = fusion.getDiagnostics();

  return {
    track: { lengthM: track.totalLengthM, durationS: track.durationS, laps: track.laps },
    events: sensors.events,
    diagnostics: diag,
    series,
    metrics: {
      rmsErrorM: rms(acc.all),
      maxErrorM: maxOf(acc.all),
      rmsErrorRtkFixedM: rms(acc.fixed),
      rmsErrorRtkFloatM: rms(acc.float),
      rmsErrorOutageM: rms(acc.outage),
      maxErrorOutageM: maxOf(acc.outage),
      rmsErrorRecoveryM: rms(acc.recovery),
      headingRmsDeg: (rms(series.filter((p) => p.t > o.warmupS).map((p) => p.headingErrRad)) * 180) / Math.PI,
      gnssAcceptedRate: diag.stats.gnssTotal ? diag.stats.gnssAccepted / diag.stats.gnssTotal : 0,
      rejectedOutliers: diag.stats.gnssRejectedNis,
      injectedOutliers: sensors.events.injectedOutliers,
      detectedOutageS: diag.maxOutageS,
      deadReckoningS: diag.stats.deadReckoningS,
    },
  };
}
