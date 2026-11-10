/**
 * src/planning/coverage_path.mjs
 * ---------------------------------------------------------------------------
 * 农田全覆盖路径规划（牛耕式 + 地头转弯 + 重叠率建模）
 *
 * 规划流程：
 *   1) 地块边界多边形（支持凸/凹）按作业幅宽 w 与重叠率 o 生成地头(headland)多圈内缩路径；
 *   2) 剩余内部区域按最优作业方向做牛耕式(boustrophedon)扫描线分解，
 *      作业行间距 ds = w * (1 - o)，即相邻行程重叠 o*w；
 *   3) 相邻行程之间用地头转弯连接，转弯轨迹由 Dubins 最短路径给出：
 *        - CSC 词（LSL/RSL/LSR/RSR）-> Π 形转弯（平头/三程转弯，需要地头深度 >= 2R）
 *        - CCC 词（LRL/RLR）      -> Ω 形转弯（灯泡形转弯，行程间距 w_eff < 2R 时使用）
 *      转弯代价 = 几何长度 / 转弯限速 + 转向执行时间；
 *   4) 以 0.25 m 栅格化统计覆盖率与重叠率（几何真实值，非设定值）。
 *
 * 关键公式：
 *   - 重叠率          o = w_overlap / w
 *   - 行程间距        ds = w * (1 - o)
 *   - 转弯限速        v_turn = min(v_max, sqrt(a_lat_max * R))
 *   - Dubins 路径长度 L = (t + p + q) * R
 */

import { polygonArea, polygonBBox, polygonCentroid, pointInPolygon, scanlineIntervals, rotatePoint, dist } from '../sim/field_simulator.mjs';

const TWO_PI = Math.PI * 2;

export function mod2pi(x) {
  let v = x % TWO_PI;
  if (v < 0) v += TWO_PI;
  return v;
}

/* =========================================================================
 * 1. 多边形内缩（地头预留 / 内部区域求解）
 * ========================================================================= */

/** 边的内法线（CCW 多边形内部在边左侧） */
function inwardNormal(a, b) {
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  const L = Math.hypot(dx, dy) || 1;
  return { x: -dy / L, y: dx / L };
}

/**
 * 多边形等距内缩（斜接 miter）。d > 0 向内缩，d < 0 向外扩。
 * 顶点新位置 = 相邻两条偏移直线的交点；近似平行时退化为沿平均法线平移。
 */
export function offsetPolygon(poly, d) {
  const n = poly.length;
  const out = [];
  for (let i = 0; i < n; i += 1) {
    const p0 = poly[(i - 1 + n) % n];
    const p1 = poly[i];
    const p2 = poly[(i + 1) % n];
    const n1 = inwardNormal(p0, p1);
    const n2 = inwardNormal(p1, p2);
    const a1 = { x: p0.x + n1.x * d, y: p0.y + n1.y * d };
    const a2 = { x: p1.x + n2.x * d, y: p1.y + n2.y * d };
    const d1 = { x: p1.x - p0.x, y: p1.y - p0.y };
    const d2 = { x: p2.x - p1.x, y: p2.y - p1.y };
    const den = d1.x * d2.y - d1.y * d2.x;
    if (Math.abs(den) < 1e-9) {
      out.push({ x: (a1.x + a2.x) / 2, y: (a1.y + a2.y) / 2 });
      continue;
    }
    const wx = a2.x - a1.x;
    const wy = a2.y - a1.y;
    const t = (wx * d2.y - wy * d2.x) / den;
    out.push({ x: a1.x + d1.x * t, y: a1.y + d1.y * t });
  }
  return out;
}

/* =========================================================================
 * 2. Dubins 最短路径（车辆最小转弯半径约束下的两点位姿连接）
 * ========================================================================= */

/**
 * CCC 型 Dubins 解（RLR / LRL）的精确几何构造。
 * 首圆与末圆半径均为 R；中间圆与二者外切（切换转向方向），
 * 圆心 C2 为"到 C1、C3 距离均为 2R"的两交点之一；各段转角由切点方位角差得到。
 * @param {number} firstDir 首段转向：+1 = 左转(LRL)，-1 = 右转(RLR)
 */
export function cccCandidates(start, end, radius, firstDir) {
  const dirs = [firstDir, -firstDir, firstDir];
  const nrm = (th, d) => (d > 0 ? { x: -Math.sin(th), y: Math.cos(th) } : { x: Math.sin(th), y: -Math.cos(th) });
  const n1 = nrm(start.theta, dirs[0]);
  const n3 = nrm(end.theta, dirs[2]);
  const C1 = { x: start.x + radius * n1.x, y: start.y + radius * n1.y };
  const C3 = { x: end.x + radius * n3.x, y: end.y + radius * n3.y };
  const dx = C3.x - C1.x;
  const dy = C3.y - C1.y;
  const D = Math.hypot(dx, dy);
  if (D < 1e-9 || D > 4 * radius - 1e-6) return [];
  const midx = (C1.x + C3.x) / 2;
  const midy = (C1.y + C3.y) / 2;
  const h = Math.sqrt(Math.max(0, 4 * radius * radius - (D / 2) * (D / 2)));
  const px = -dy / D;
  const py = dx / D;
  const sweep = (from, to, dir) => {
    const a0 = Math.atan2(from.y, from.x);
    const a1 = Math.atan2(to.y, to.x);
    let d = a1 - a0;
    if (dir > 0) {
      while (d < 0) d += TWO_PI;
    } else {
      while (d > 0) d -= TWO_PI;
      d = -d;
    }
    return d;
  };
  const out = [];
  for (const sgn of [1, -1]) {
    const C2 = { x: midx + sgn * h * px, y: midy + sgn * h * py };
    const u12 = { x: (C2.x - C1.x) / 2, y: (C2.y - C1.y) / 2 };
    const u23 = { x: (C2.x - C3.x) / 2, y: (C2.y - C3.y) / 2 };
    const T12 = { x: C1.x + radius * (u12.x / radius), y: C1.y + radius * (u12.y / radius) };
    const T23 = { x: C3.x + radius * (u23.x / radius), y: C3.y + radius * (u23.y / radius) };
    const t = sweep({ x: start.x - C1.x, y: start.y - C1.y }, { x: T12.x - C1.x, y: T12.y - C1.y }, dirs[0]);
    const pp = sweep({ x: T12.x - C2.x, y: T12.y - C2.y }, { x: T23.x - C2.x, y: T23.y - C2.y }, dirs[1]);
    const q = sweep({ x: T23.x - C3.x, y: T23.y - C3.y }, { x: end.x - C3.x, y: end.y - C3.y }, dirs[2]);
    if (pp < 1e-9) continue;
    out.push({
      word: dirs.map((dd) => (dd > 0 ? 'L' : 'R')).join(''),
      t,
      p: pp,
      q,
      length: (t + pp + q) * radius,
      radiusM: radius,
    });
  }
  return out;
}

export function dubinsCandidates(start, end, r) {
  const dx = end.x - start.x;
  const dy = end.y - start.y;
  const D = Math.hypot(dx, dy);
  if (D < 1e-9) return null;
  const d = D / r;
  const th = Math.atan2(dy, dx);
  const alpha = mod2pi(start.theta - th);
  const beta = mod2pi(end.theta - th);
  const sa = Math.sin(alpha);
  const ca = Math.cos(alpha);
  const sb = Math.sin(beta);
  const cb = Math.cos(beta);
  const cab = Math.cos(alpha - beta);
  const cands = [];
  // LSL
  {
    const tmp0 = d + sa - sb;
    const pSq = 2 + d * d - 2 * cab + 2 * d * (sa - sb);
    if (pSq >= 0) {
      const tmp1 = Math.atan2(cb - ca, tmp0);
      cands.push({ word: 'LSL', t: mod2pi(-alpha + tmp1), p: Math.sqrt(pSq), q: mod2pi(beta - tmp1) });
    }
  }
  // RSR
  {
    const tmp0 = d - sa + sb;
    const pSq = 2 + d * d - 2 * cab + 2 * d * (sb - sa);
    if (pSq >= 0) {
      const tmp1 = Math.atan2(ca - cb, tmp0);
      cands.push({ word: 'RSR', t: mod2pi(alpha - tmp1), p: Math.sqrt(pSq), q: mod2pi(-beta + tmp1) });
    }
  }
  // LSR
  {
    const pSq = -2 + d * d + 2 * cab + 2 * d * (sa + sb);
    if (pSq >= 0) {
      const p = Math.sqrt(pSq);
      const tmp = Math.atan2(-ca - cb, d + sa + sb) - Math.atan2(-2.0, p);
      cands.push({ word: 'LSR', t: mod2pi(-alpha + tmp), p, q: mod2pi(-mod2pi(beta) + tmp) });
    }
  }
  // RSL
  {
    const pSq = d * d - 2 + 2 * cab - 2 * d * (sa + sb);
    if (pSq >= 0) {
      const p = Math.sqrt(pSq);
      const tmp = Math.atan2(ca + cb, d - sa - sb) - Math.atan2(2.0, p);
      cands.push({ word: 'RSL', t: mod2pi(alpha - tmp), p, q: mod2pi(beta - tmp) });
    }
  }
  // CCC 词（RLR / LRL）：采用精确几何构造而非闭式解，
  // 中间圆与首末圆外切，切点连线即为圆心的连线，保证端点精确闭合。
  for (const firstDir of [-1, 1]) {
    for (const c of cccCandidates(start, end, r, firstDir)) cands.push(c);
  }
  const out = [];
  for (const c of cands) {
    if (!(c.p >= 0)) continue;
    out.push({ ...c, length: (c.t + c.p + c.q) * r, radiusM: r });
  }
  return out;
}

/** 在给定词表中择优（word 为空表示不限制） */
function pickCandidate(cands, preferWords) {
  const pool = preferWords ? cands.filter((c) => preferWords.includes(c.word)) : cands;
  let best = null;
  for (const c of pool) if (!best || c.length < best.length) best = c;
  return best;
}

/**
 * Π 形平头转弯（flat turn）闭式几何：两段 90 度圆弧 + 一段直线。
 *   路径长度 L = pi*R + w - 2R        （w 为相邻行程的横向间距）
 *   几何约束  w >= 2R 且纵向偏差约为 0
 *   地头深度需求 R，横向占用 w
 * 当 w < 2R 时该转弯几何上不可实现，必须改用 Ω 形（灯泡）转弯或跳行作业。
 */
export function flatTurn(start, end, radius, opts) {
  const o = Object.assign({ maxLongitudinalM: 3.0 }, opts || {});
  const u = { x: Math.cos(start.theta), y: Math.sin(start.theta) };
  const left = { x: -u.y, y: u.x };
  const dx = end.x - start.x;
  const dy = end.y - start.y;
  const wLat = dx * left.x + dy * left.y; // 横向（左正）
  const wLon = dx * u.x + dy * u.y; // 纵向
  let dth = mod2pi(end.theta - start.theta);
  if (dth > Math.PI) dth -= Math.PI * 2;
  const isReversal = Math.abs(Math.abs(dth) - Math.PI) < 1e-6;
  if (!isReversal || Math.abs(wLat) < 2 * radius - 1e-9 || Math.abs(wLon) > o.maxLongitudinalM) return null;
  const dir = wLat > 0 ? 1 : -1; // 左转 / 右转
  const R = radius;
  const mid = Math.abs(wLat) - 2 * R;
  // 纵向偏差用转弯前后的直线段补足（前进方向，不允许倒车）
  const leadIn = wLon > 0 ? wLon : 0;
  const leadOut = wLon < 0 ? -wLon : 0;
  const segs = [];
  if (leadIn > 1e-6) segs.push({ type: 'S', param: leadIn / R });
  segs.push({ type: dir > 0 ? 'L' : 'R', param: Math.PI / 2 });
  segs.push({ type: 'S', param: mid / R });
  segs.push({ type: dir > 0 ? 'L' : 'R', param: Math.PI / 2 });
  if (leadOut > 1e-6) segs.push({ type: 'S', param: leadOut / R });
  const length = Math.PI * R + mid + leadIn + leadOut;
  return {
    word: dir > 0 ? 'flat-left' : 'flat-right',
    type: 'Π',
    lengthM: length,
    segments: segs,
    radiusM: R,
    lateralOffsetM: Math.abs(wLat),
    longitudinalOffsetM: wLon,
    midStraightM: mid,
    leadInM: leadIn,
    leadOutM: leadOut,
  };
}

/**
 * Ω 形灯泡转弯（bulb turn）：使用 Dubins 的 CCC 词（LRL/RLR）解，
 * 行程间距 w < 2R 时唯一可行的地头调头几何，路径明显长于 Π 形转弯。
 */
export function bulbTurn(start, end, radius, step) {
  const cands = dubinsCandidates(start, end, radius);
  const best = pickCandidate(cands, ['LRL', 'RLR']);
  if (!best) return null;
  const path = dubinsPath(start, end, radius, step, ['LRL', 'RLR']);
  return { ...path, type: 'Ω', isBulb: true };
}

/**
 * 跳行作业行序：把 N 条作业行按 k = nSkip+1 分组，
 * 组内相隔 k 行（横向间距 k*ds >= 2R，可用 Π 形转弯），组间相邻（仅需一次 Ω 转弯）。
 * 这样把 O(N) 次 Ω 转弯降为 O(1) 次，显著缩短地头空驶里程。
 */
export function skipRowOrder(strips, k) {
  const N = strips.length;
  const order = [];
  const blocks = Math.min(k, N);
  for (let j = 0; j < blocks; j += 1) {
    const block = [];
    for (let i = j; i < N; i += k) block.push(i);
    if (!block.length) continue;
    const seq = j % 2 === 0 ? block : block.slice().reverse();
    for (const i of seq) order.push(strips[i]);
  }
  return order;
}

/**
 * 从给定起始位姿出发，按段序列（L/R/S）在世界坐标系中积分采样。
 * 用于 Π 形平头转弯等以车辆自身坐标系定义的转弯几何。
 */
export function sampleTurnPath(startPose, turn, radius, step) {
  const st = step === undefined ? 0.5 : step;
  let x = startPose.x;
  let y = startPose.y;
  let theta = startPose.theta;
  const pts = [{ x, y, theta, s: 0 }];
  let s = 0;
  for (const seg of turn.segments) {
    const segLen = seg.param * radius;
    if (segLen <= 1e-9) continue;
    const n = Math.max(1, Math.ceil(segLen / st));
    const dl = segLen / n;
    for (let i = 1; i <= n; i += 1) {
      if (seg.type === 'S') {
        x += dl * Math.cos(theta);
        y += dl * Math.sin(theta);
      } else {
        const dir = seg.type === 'L' ? 1 : -1;
        const dth = (dl / radius) * dir;
        const cx = x - dir * radius * Math.sin(theta);
        const cy = y + dir * radius * Math.cos(theta);
        x = cx + dir * radius * Math.sin(theta + dth);
        y = cy - dir * radius * Math.cos(theta + dth);
        theta += dth;
      }
      s += dl;
      pts.push({ x, y, theta: mod2pi(theta), s });
    }
  }
  return pts;
}

/**
 * 生成 Dubins 路径（含采样点）。
 * @returns {{word:string,type:string,lengthM:number,segments:Array,points:Array}}
 */
export function dubinsPath(start, end, radius, sampleStepM, preferWords) {
  const best = pickCandidate(dubinsCandidates(start, end, radius), preferWords || null);
  if (!best) {
    // 退化情形（同点位姿）：直接原地转向
    const L = Math.abs(mod2pi(end.theta - start.theta)) * radius;
    return {
      word: 'LRL',
      type: 'Ω',
      lengthM: L,
      segments: [{ type: 'L', param: L / radius }],
      points: [{ x: start.x, y: start.y, theta: start.theta, s: 0 }],
    };
  }
  const step = sampleStepM === undefined ? 0.5 : sampleStepM;
  const dx = end.x - start.x;
  const dy = end.y - start.y;
  const th = Math.atan2(dy, dx);
  const c = Math.cos(th);
  const s = Math.sin(th);
  const alpha = mod2pi(start.theta - th);
  const segs = [
    { type: best.word[0], param: best.t },
    { type: best.word[1], param: best.p },
    { type: best.word[2], param: best.q },
  ];
  // 在归一化坐标系中积分，再变换回世界坐标
  const pts = [];
  let x = 0;
  let y = 0;
  let theta = alpha;
  let arc = 0;
  pts.push({ x: start.x, y: start.y, theta: start.theta, s: 0 });
  for (const seg of segs) {
    if (seg.param <= 1e-9) continue;
    const segLen = seg.param * radius;
    const nSteps = Math.max(1, Math.ceil(segLen / step));
    const dl = segLen / nSteps;
    for (let i = 1; i <= nSteps; i += 1) {
      if (seg.type === 'S') {
        x += dl * Math.cos(theta);
        y += dl * Math.sin(theta);
      } else {
        const dir = seg.type === 'L' ? 1 : -1;
        const dth = (dl / radius) * dir;
        const cx = x - dir * radius * Math.sin(theta);
        const cy = y + dir * radius * Math.cos(theta);
        x = cx + dir * radius * Math.sin(theta + dth);
        y = cy - dir * radius * Math.cos(theta + dth);
        theta += dth;
      }
      arc += dl;
      pts.push({
        x: start.x + (x * c - y * s),
        y: start.y + (x * s + y * c),
        theta: mod2pi(theta + th),
        s: arc,
      });
    }
  }
  // 转弯类型判定以"总航向变化量"为准：
  //   总转角接近 180 deg -> Π 形平头转弯；>= 315 deg（形成回环）-> Ω 形灯泡转弯
  let totalTurn = 0;
  for (const s of segs) {
    if (s.type === 'S') continue;
    totalTurn += s.param * (s.type === 'L' ? 1 : -1);
  }
  const type = Math.abs(totalTurn) >= 1.75 * Math.PI ? 'Ω' : 'Π';
  const turnWord = best.word === 'LRL' || best.word === 'RLR' ? 'Ω' : type === 'Ω' ? best.word + '(回环)' : best.word;
  return { word: turnWord, type, lengthM: best.length, segments: segs, points: pts, totalTurnRad: totalTurn };
}

/**
 * 地头环圈路径生成：沿内缩多边形行驶，在顶点处用半径为 R 的圆弧过渡（圆角弯）。
 * 当相邻边长不足以容纳切线段时按 45% 边长为上限自动减小该处转弯半径，
 * 并记录实际半径（工程上对应"减速 + 小半径转向"）。
 *
 * @returns {{points:Array, straights:Array, turns:Array, lengthM:number}}
 */
export function filletLoop(loop, radius, opts) {
  const o = Object.assign({ stepM: 0.5, maxTangentRatio: 0.45 }, opts || {});
  const n = loop.length;
  const corners = [];
  for (let i = 0; i < n; i += 1) {
    const prev = loop[(i - 1 + n) % n];
    const cur = loop[i];
    const next = loop[(i + 1) % n];
    const lu = Math.hypot(cur.x - prev.x, cur.y - prev.y) || 1;
    const lv = Math.hypot(next.x - cur.x, next.y - cur.y) || 1;
    const u = { x: (cur.x - prev.x) / lu, y: (cur.y - prev.y) / lu };
    const v = { x: (next.x - cur.x) / lv, y: (next.y - cur.y) / lv };
    const cross = u.x * v.y - u.y * v.x;
    const dot = u.x * v.x + u.y * v.y;
    const delta = Math.atan2(cross, dot); // 左转为正
    const absD = Math.abs(delta);
    let Reff = radius;
    if (absD < 1e-6) {
      corners.push({ idx: i, delta, Reff: 0, T: 0, pIn: cur, pOut: cur, u, v, center: cur });
      continue;
    }
    let T = radius * Math.tan(absD / 2);
    const maxT = o.maxTangentRatio * Math.min(lu, lv);
    let reduced = false;
    if (T > maxT) {
      T = maxT;
      Reff = T / Math.tan(absD / 2);
      reduced = true;
    }
    const pIn = { x: cur.x - u.x * T, y: cur.y - u.y * T };
    const pOut = { x: cur.x + v.x * T, y: cur.y + v.y * T };
    const dir = delta > 0 ? 1 : -1;
    const center = { x: pIn.x - dir * Reff * u.y, y: pIn.y + dir * Reff * u.x };
    corners.push({ idx: i, delta, Reff, T, pIn, pOut, u, v, center, reduced });
  }
  const points = [];
  const straights = [];
  const turns = [];
  let total = 0;
  for (let i = 0; i < n; i += 1) {
    const prevC = corners[(i - 1 + n) % n];
    const c = corners[i];
    // 直线段：上一顶点的出切点 -> 本顶点的入切点
    const a = prevC.pOut;
    const b = c.pIn;
    const len = Math.hypot(b.x - a.x, b.y - a.y);
    if (len > 1e-6) {
      const theta = Math.atan2(b.y - a.y, b.x - a.x);
      straights.push({ x0: a.x, y0: a.y, x1: b.x, y1: b.y, lengthM: len, theta });
      const nS = Math.max(1, Math.ceil(len / o.stepM));
      for (let t = 0; t <= nS; t += 1) {
        const f = t / nS;
        points.push({ x: a.x + (b.x - a.x) * f, y: a.y + (b.y - a.y) * f, theta, type: 'work' });
      }
      total += len;
    }
    // 圆角段
    if (c.Reff > 1e-6 && Math.abs(c.delta) > 1e-6) {
      const arcLen = c.Reff * Math.abs(c.delta);
      const a0 = Math.atan2(c.pIn.y - c.center.y, c.pIn.x - c.center.x);
      const nS = Math.max(2, Math.ceil(arcLen / o.stepM));
      for (let t = 0; t <= nS; t += 1) {
        const ang = a0 + (c.delta * t) / nS;
        points.push({
          x: c.center.x + c.Reff * Math.cos(ang),
          y: c.center.y + c.Reff * Math.sin(ang),
          theta: mod2pi(ang + (c.delta > 0 ? Math.PI / 2 : -Math.PI / 2)),
          type: 'turn',
        });
      }
      turns.push({
        idx: c.idx,
        angleDeg: (Math.abs(c.delta) * 180) / Math.PI,
        radiusM: c.Reff,
        lengthM: arcLen,
        reduced: !!c.reduced,
        direction: c.delta > 0 ? 'left' : 'right',
        centerX: c.center.x,
        centerY: c.center.y,
        startAngle: a0,
        deltaAngle: c.delta,
      });
      total += arcLen;
    }
  }
  return { points, straights, turns, lengthM: total };
}

/* =========================================================================
 * 3. 牛耕式扫描线分解
 * ========================================================================= */

/**
 * 生成牛耕式作业条带。
 * @param {Array} polygon CCW 多边形（内部区域）
 * @param {object} opts {swathWidthM, overlapRatio, angleDeg|null, angleStepDeg}
 * @returns {{strips:Array, angleDeg:number, swathCount:number, workLengthM:number}}
 *   strips[i] = {index, x0,y0,x1,y1, dir:+1|-1, lengthM}
 */
export function boustrophedonPath(polygon, opts) {
  const o = Object.assign(
    { swathWidthM: 3.0, overlapRatio: 0.08, angleDeg: null, angleStepDeg: 2, minSwathLengthM: 0.5 },
    opts || {},
  );
  const ds = o.swathWidthM * (1 - o.overlapRatio);
  const evaluate = (angleDeg) => {
    const rad = (-angleDeg * Math.PI) / 180;
    const rot = polygon.map((p) => rotatePoint(p, rad));
    const bb = polygonBBox(rot);
    const nLine = Math.max(1, Math.floor((bb.maxY - bb.minY) / ds) + 1);
    const y0 = bb.minY + ((bb.maxY - bb.minY) - (nLine - 1) * ds) / 2;
    const strips = [];
    let workLen = 0;
    let intervalCount = 0;
    for (let k = 0; k < nLine; k += 1) {
      const y = y0 + k * ds;
      const iv = scanlineIntervals(rot, y);
      intervalCount += iv.length;
      // 偶数行正序，奇数行逆序（蛇形）
      const ordered = k % 2 === 0 ? iv.slice() : iv.slice().reverse();
      for (const seg of ordered) {
        if (seg.x1 - seg.x0 < o.minSwathLengthM) continue;
        const forward = k % 2 === 0;
        const ax = forward ? seg.x0 : seg.x1;
        const bx = forward ? seg.x1 : seg.x0;
        const p0 = rotatePoint({ x: ax, y }, -rad);
        const p1 = rotatePoint({ x: bx, y }, -rad);
        strips.push({
          index: strips.length,
          swathIndex: k,
          x0: p0.x,
          y0: p0.y,
          x1: p1.x,
          y1: p1.y,
          dir: forward ? 1 : -1,
          lengthM: seg.x1 - seg.x0,
          scanY: y,
        });
        workLen += seg.x1 - seg.x0;
      }
    }
    return { strips, angleDeg, workLen, intervalCount, nLine };
  };
  if (o.angleDeg !== null && o.angleDeg !== undefined) return evaluate(o.angleDeg);
  // 自动搜索最优作业方向：以"总行程段数"最少为首要目标（转弯次数最少），
  // 以"作业里程最短"为次要目标（减少地块外无效重叠）
  let best = null;
  for (let a = 0; a < 180; a += o.angleStepDeg) {
    const r = evaluate(a);
    const score = r.intervalCount * 1000 + r.workLen;
    if (!best || score < best.score) best = { ...r, score };
  }
  return best;
}

/* =========================================================================
 * 4. 转弯代价模型
 * ========================================================================= */

/**
 * 地头转弯代价模型。
 * Π 形转弯（CSC/Dubins 平头转弯）：几何长度短，但需要地头深度 >= 2R；
 * Ω 形转弯（CCC/灯泡转弯）：行程间距 < 2R 时必须使用，路径长、耗时长。
 * 代价 = 几何长度 / 转弯限速 + 转向执行时间（+ 大角度转弯的农具提升/落放时间）
 */
export function turnCostModel(turn, opts) {
  const o = Object.assign(
    {
      minTurnRadiusM: 4.0,
      vMaxMps: 3.0,
      aLatMaxMps2: 1.5,
      steerRateDegPerS: 25,
      implementLiftTimeS: 1.2,
      liftAngleThresholdDeg: 150,
    },
    opts || {},
  );
  const radius = turn.radiusM || o.minTurnRadiusM;
  const vTurn = Math.min(o.vMaxMps, Math.sqrt(o.aLatMaxMps2 * radius));
  const turnAngleDeg = ((turn.lengthM / radius) * 180) / Math.PI;
  const steerTimeS = turnAngleDeg / o.steerRateDegPerS;
  const needLift = turnAngleDeg >= o.liftAngleThresholdDeg;
  const liftTimeS = needLift ? o.implementLiftTimeS : 0;
  const travelTimeS = turn.lengthM / vTurn;
  return {
    type: turn.type,
    word: turn.word,
    lengthM: turn.lengthM,
    turnAngleDeg,
    vTurnMps: vTurn,
    travelTimeS,
    steerTimeS: Math.min(steerTimeS, travelTimeS),
    implementLiftTimeS: liftTimeS,
    timeS: Math.max(travelTimeS, steerTimeS) + liftTimeS,
  };
}

/* =========================================================================
 * 5. 栅格化覆盖率 / 重叠率
 * ========================================================================= */

/**
 * 以栅格化方式统计覆盖率与重叠率（几何真实值）。
 * @param {Array} polygon 地块边界
 * @param {Array} strips 作业条带 [{x0,y0,x1,y1}]
 * @param {object} opts {swathWidthM, cellSizeM}
 */
export function rasterizeCoverage(polygon, strips, opts) {
  const o = Object.assign({ swathWidthM: 3.0, cellSizeM: 0.25 }, opts || {});
  const bb = polygonBBox(polygon);
  const pad = o.swathWidthM + o.cellSizeM * 2;
  const minX = bb.minX - pad;
  const minY = bb.minY - pad;
  const cols = Math.ceil((bb.maxX - bb.minX + 2 * pad) / o.cellSizeM);
  const rows = Math.ceil((bb.maxY - bb.minY + 2 * pad) / o.cellSizeM);
  const total = cols * rows;
  const count = new Uint8Array(total);
  const inField = new Uint8Array(total);
  const cx = (i) => minX + (i + 0.5) * o.cellSizeM;
  const cy = (j) => minY + (j + 0.5) * o.cellSizeM;
  const cell = (x, y) => {
    const i = Math.floor((x - minX) / o.cellSizeM);
    const j = Math.floor((y - minY) / o.cellSizeM);
    if (i < 0 || j < 0 || i >= cols || j >= rows) return -1;
    return j * cols + i;
  };
  for (let j = 0; j < rows; j += 1) {
    for (let i = 0; i < cols; i += 1) {
      if (pointInPolygon({ x: cx(i), y: cy(j) }, polygon)) inField[j * cols + i] = 1;
    }
  }
  const half = o.swathWidthM / 2;
  for (const s of strips) {
    const len = Math.hypot(s.x1 - s.x0, s.y1 - s.y0);
    const ux = (s.x1 - s.x0) / (len || 1);
    const uy = (s.y1 - s.y0) / (len || 1);
    const sMinX = Math.min(s.x0, s.x1) - half;
    const sMaxX = Math.max(s.x0, s.x1) + half;
    const sMinY = Math.min(s.y0, s.y1) - half;
    const sMaxY = Math.max(s.y0, s.y1) + half;
    const i0 = Math.max(0, Math.floor((sMinX - minX) / o.cellSizeM));
    const i1 = Math.min(cols - 1, Math.floor((sMaxX - minX) / o.cellSizeM));
    const j0 = Math.max(0, Math.floor((sMinY - minY) / o.cellSizeM));
    const j1 = Math.min(rows - 1, Math.floor((sMaxY - minY) / o.cellSizeM));
    for (let j = j0; j <= j1; j += 1) {
      for (let i = i0; i <= i1; i += 1) {
        const px = cx(i) - s.x0;
        const py = cy(j) - s.y0;
        const proj = px * ux + py * uy;
        if (proj < -half || proj > len + half) continue;
        const perp = Math.abs(px * uy - py * ux);
        if (perp > half) continue;
        const idx = j * cols + i;
        if (count[idx] < 255) count[idx] += 1;
      }
    }
  }
  let fieldCells = 0;
  let coveredCells = 0;
  let sumCount = 0;
  for (let idx = 0; idx < total; idx += 1) {
    if (!inField[idx]) continue;
    fieldCells += 1;
    if (count[idx] > 0) {
      coveredCells += 1;
      sumCount += count[idx];
    }
  }
  const cellArea = o.cellSizeM * o.cellSizeM;
  return {
    cellSizeM: o.cellSizeM,
    gridCols: cols,
    gridRows: rows,
    fieldCells,
    coveredCells,
    coverageRate: fieldCells ? coveredCells / fieldCells : 0,
    coveredMultiplicity: sumCount,
    overlapRatio: sumCount > 0 ? (sumCount - coveredCells) / sumCount : 0,
    fieldAreaM2: fieldCells * cellArea,
    workedAreaM2: sumCount * cellArea,
    uncoveredCells: fieldCells - coveredCells,
  };
}

/* =========================================================================
 * 6. 全覆盖规划器
 * ========================================================================= */

export class CoveragePlanner {
  /**
   * @param {object} opts
   *   field               地块对象（含 polygon / areaM2）
   *   swathWidthM         作业幅宽 (m)
   *   overlapRatio        设定重叠率（0~0.2）
   *   minTurnRadiusM      最小转弯半径 (m)
   *   headlandWidthM      地头宽度 (m)，默认 2 倍幅宽
   *   workSpeedMps        作业速度 (m/s)
   *   transitSpeedMps     转移速度 (m/s)
   *   aLatMaxMps2         允许横向加速度 (m/s^2)
   *   steerRateDegPerS    转向速率 (deg/s)
   *   cellSizeM           覆盖率栅格尺寸 (m)
   *   angleDeg            作业方向（null = 自动寻优）
   */
  constructor(opts) {
    const o = Object.assign(
      {
        swathWidthM: 3.0,
        overlapRatio: 0.08,
        minTurnRadiusM: 4.0,
        headlandWidthM: null,
        workSpeedMps: 2.2,
        transitSpeedMps: 6.0,
        aLatMaxMps2: 1.5,
        steerRateDegPerS: 25,
        cellSizeM: 0.25,
        angleDeg: null,
        // 转弯策略：'auto' | 'adjacent' | 'skip-row'（详见 plan() 内注释）
        turnStrategy: 'auto',
        // 地头环圈转角是否保持机具作业状态：
        //   true  —— 播种/耕整/收获等连续作业，转角圆弧计入覆盖面积；
        //   false —— 喷施类作业（喷头在转弯时关闭，避免重喷与飘移），圆弧不计入覆盖面积
        cornerArcWorking: true,
        machineId: 'M?',
      },
      opts || {},
    );
    this.opt = o;
    this.field = o.field;
    this.swathWidthM = o.swathWidthM;
    this.overlapRatio = o.overlapRatio;
    this.minTurnRadiusM = o.minTurnRadiusM;
    this.headlandWidthM = o.headlandWidthM === null ? 2 * o.swathWidthM : o.headlandWidthM;
  }

  plan() {
    const o = this.opt;
    const poly = this.field.polygon;
    const headlandPasses = Math.max(1, Math.round(this.headlandWidthM / this.swathWidthM));
    const ds = this.swathWidthM * (1 - this.overlapRatio);

    // ---- 1) 地头内缩多边形（多圈） ----
    const headlandLoops = [];
    for (let k = 0; k < headlandPasses; k += 1) {
      const d = (k + 0.5) * this.swathWidthM;
      const p = offsetPolygon(poly, d);
      if (polygonArea(p) > 1) headlandLoops.push(p);
    }
    const innerPolygon = offsetPolygon(poly, headlandPasses * this.swathWidthM);

    // ---- 2) 内部区域牛耕式分解 ----
    const bou = boustrophedonPath(innerPolygon, {
      swathWidthM: this.swathWidthM,
      overlapRatio: this.overlapRatio,
      angleDeg: o.angleDeg,
    });

    // ---- 3) 地头环圈路径（沿内缩多边形行驶，转角用 Dubins 圆滑） ----
    const workStrips = [];
    const turnPaths = [];
    const pathPoints = [];
    let workLengthM = 0;
    let headlandLengthM = 0;
    let turnLengthM = 0;
    let piTurnCount = 0;
    let omegaTurnCount = 0;
    let turnTimeS = 0;
    let cornerArcLengthM = 0;
    let cornerTimeS = 0;
    let cornerCount = 0;
    let turnCostDetail = [];
    let lastPose = null;

    const pushPoint = (p, type, extra) => {
      pathPoints.push(Object.assign({ x: p.x, y: p.y, theta: p.theta, type }, extra || {}));
    };

    // 地头环圈：沿内缩多边形行驶，顶点处用半径 R 的圆弧过渡（转弯半径不足时自动减小）
    for (let k = 0; k < headlandLoops.length; k += 1) {
      const loop = headlandLoops[k];
      const fil = filletLoop(loop, this.minTurnRadiusM, { stepM: 0.5 });
      let loopLen = 0;
      const first = fil.points[0];
      // 由上一段（或上一圈）接入本圈起点
      if (lastPose) {
        const turn = dubinsPath(lastPose, { x: first.x, y: first.y, theta: first.theta }, this.minTurnRadiusM);
        const cost = turnCostModel(turn, {
          minTurnRadiusM: this.minTurnRadiusM,
          vMaxMps: o.workSpeedMps,
          aLatMaxMps2: o.aLatMaxMps2,
          steerRateDegPerS: o.steerRateDegPerS,
        });
        if (turn.lengthM > 0.05) {
          turnLengthM += turn.lengthM;
          turnTimeS += cost.timeS;
          if (cost.type === 'Ω') omegaTurnCount += 1;
          else piTurnCount += 1;
          turnPaths.push({ ...cost, where: 'headland-entry' });
          for (let t = 1; t < turn.points.length; t += 1) pushPoint(turn.points[t], 'turn', { turnType: cost.type, where: 'headland-entry' });
        }
      }
      for (const s of fil.straights) {
        workStrips.push({ x0: s.x0, y0: s.y0, x1: s.x1, y1: s.y1, kind: 'headland' });
        loopLen += s.lengthM;
      }
      for (const pt of fil.points) pushPoint(pt, pt.type, { loop: k });
      // 地头环圈的转角圆弧：机具保持入土/喷施状态连续作业（仅减速通过），
      // 因此计入作业里程与覆盖面积；其耗时按转弯限速计算。
      for (const t of fil.turns) {
        const cost = turnCostModel(
          { word: t.direction === 'left' ? 'LSL' : 'RSR', type: 'Π', lengthM: t.lengthM, radiusM: t.radiusM },
          {
            minTurnRadiusM: this.minTurnRadiusM,
            vMaxMps: o.workSpeedMps,
            aLatMaxMps2: o.aLatMaxMps2,
            steerRateDegPerS: o.steerRateDegPerS,
          },
        );
        cornerArcLengthM += t.lengthM;
        cornerTimeS += cost.timeS;
        cornerCount += 1;
        turnPaths.push({ ...cost, where: 'headland-corner', radiusReduced: t.reduced });
        if (o.cornerArcWorking) {
          // 圆弧段作为作业条带纳入覆盖率统计（机具保持作业状态）
          const nArcSeg = 16;
          for (let q = 0; q < nArcSeg; q += 1) {
            const angA = t.startAngle + (t.deltaAngle * q) / nArcSeg;
            const angB = t.startAngle + (t.deltaAngle * (q + 1)) / nArcSeg;
            workStrips.push({
              x0: t.centerX + t.radiusM * Math.cos(angA),
              y0: t.centerY + t.radiusM * Math.sin(angA),
              x1: t.centerX + t.radiusM * Math.cos(angB),
              y1: t.centerY + t.radiusM * Math.sin(angB),
              kind: 'headland-arc',
            });
          }
          loopLen += t.lengthM;
        } else {
          // 喷施类作业转弯时关闭喷头：圆弧里程计入非作业里程
          turnLengthM += t.lengthM;
          piTurnCount += 1;
        }
      }
      headlandLengthM += loopLen;
      workLengthM += loopLen;
      lastPose = { x: fil.points[fil.points.length - 1].x, y: fil.points[fil.points.length - 1].y, theta: fil.points[fil.points.length - 1].theta };
    }

    // ---- 4) 内部牛耕式条带 + Π/Ω 转弯 ----
    // 转弯策略：
    //   adjacent  —— 相邻行程间转弯，行程间距 ds < 2R 时只能用 Ω 形转弯（灯泡转弯）；
    //   skip-row  —— 跳行作业，组内行程间距 k*ds >= 2R，可用 Π 形转弯（平头转弯），
    //                组间仅需一次 Ω 转弯，地头空驶里程大幅缩短；
    //   auto      —— ds >= 2R 用 adjacent，否则用 skip-row。
    const skipRows = Math.max(0, Math.ceil((2 * this.minTurnRadiusM) / ds) - 1);
    const strategy = o.turnStrategy === 'auto' ? (ds >= 2 * this.minTurnRadiusM ? 'adjacent' : 'skip-row') : o.turnStrategy;
    let orderedStrips = bou.strips;
    let effectiveSkip = 0;
    if (strategy === 'skip-row' && skipRows >= 1 && bou.strips.length > skipRows + 1) {
      orderedStrips = skipRowOrder(bou.strips, skipRows + 1);
      effectiveSkip = skipRows;
    }
    for (let i = 0; i < orderedStrips.length; i += 1) {
      const s = orderedStrips[i];
      const theta = Math.atan2(s.y1 - s.y0, s.x1 - s.x0);
      const startPose = { x: s.x0, y: s.y0, theta };
      const endPose = { x: s.x1, y: s.y1, theta };
      if (lastPose) {
        // 优先尝试 Π 形平头转弯（几何可行且代价更低时），否则用 Ω 形灯泡转弯
        // 纵向偏差容许量：转弯前的直线段沿上一行程方向延伸，最远不超出地头宽度
        let turn = flatTurn(lastPose, startPose, this.minTurnRadiusM, {
          maxLongitudinalM: Math.max(1.0, 0.85 * this.headlandWidthM),
        });
        if (turn) {
          turn.points = sampleTurnPath(lastPose, turn, this.minTurnRadiusM);
        } else {
          turn = bulbTurn(lastPose, startPose, this.minTurnRadiusM) || dubinsPath(lastPose, startPose, this.minTurnRadiusM);
        }
        const cost = turnCostModel(turn, {
          minTurnRadiusM: this.minTurnRadiusM,
          vMaxMps: o.workSpeedMps,
          aLatMaxMps2: o.aLatMaxMps2,
          steerRateDegPerS: o.steerRateDegPerS,
        });
        turnLengthM += turn.lengthM;
        turnTimeS += cost.timeS;
        if (cost.type === 'Ω') omegaTurnCount += 1;
        else piTurnCount += 1;
        turnCostDetail.push({ ...cost, index: turnCostDetail.length, lateralOffsetM: turn.lateralOffsetM || null });
        turnPaths.push({ ...cost, from: lastPose, to: startPose, where: 'swath-transition' });
        for (let t = 1; t < turn.points.length; t += 1) {
          pushPoint(turn.points[t], 'turn', { turnType: cost.type, where: 'swath-transition' });
        }
      }
      // 作业条带
      pushPoint(startPose, 'work', { swath: s.swathIndex, strip: i });
      const nSamp = Math.max(1, Math.ceil(s.lengthM / 1.0));
      for (let t = 1; t <= nSamp; t += 1) {
        const f = t / nSamp;
        pushPoint({ x: s.x0 + (s.x1 - s.x0) * f, y: s.y0 + (s.y1 - s.y0) * f, theta }, 'work', { swath: s.swathIndex, strip: i });
      }
      workStrips.push({ x0: s.x0, y0: s.y0, x1: s.x1, y1: s.y1, kind: 'swath' });
      workLengthM += s.lengthM;
      lastPose = endPose;
    }

    // ---- 5) 覆盖率 / 重叠率栅格化 ----
    const raster = rasterizeCoverage(poly, workStrips, {
      swathWidthM: this.swathWidthM,
      cellSizeM: o.cellSizeM,
    });
    // 单独统计"相邻行程重叠率"（仅计直线作业条带，不含地头转角圆弧的重复覆盖），
    // 该指标与设定重叠率 o 直接可比；含转角的全量重复覆盖率另列为 overlapRatioWithCorners。
    const rasterSwaths = rasterizeCoverage(
      poly,
      workStrips.filter((s) => s.kind !== 'headland-arc'),
      { swathWidthM: this.swathWidthM, cellSizeM: o.cellSizeM },
    );

    // 作业时间：直线作业段按作业速度，地头转角圆弧按转弯限速时间计入；
    // 非作业里程（调头转弯）时间单列累加到总时长。
    const workingArcM = o.cornerArcWorking ? cornerArcLengthM : 0;
    const workTimeS = (workLengthM - workingArcM) / o.workSpeedMps + cornerTimeS;
    const totalTimeS = workTimeS + turnTimeS;
    const fieldAreaM2 = Math.abs(polygonArea(poly));
    const metrics = {
      fieldAreaM2,
      fieldAreaHa: fieldAreaM2 / 10000,
      swathWidthM: this.swathWidthM,
      overlapRatioDesign: this.overlapRatio,
      overlapRatioMeasured: rasterSwaths.overlapRatio,
      overlapRatioWithCorners: raster.overlapRatio,
      coverageRateSwathsOnly: rasterSwaths.coverageRate,
      coverageRate: raster.coverageRate,
      coverageUncoveredM2: raster.uncoveredCells * raster.cellSizeM * raster.cellSizeM,
      workedAreaM2: raster.workedAreaM2,
      stripSpacingM: ds,
      headlandWidthM: this.headlandWidthM,
      headlandPasses,
      swathCount: bou.strips.length,
      scanLineCount: bou.nLine,
      sweepAngleDeg: bou.angleDeg,
      workLengthM,
      headlandLengthM,
      swathLengthM: workLengthM - headlandLengthM,
      turnLengthM,
      nonWorkingLengthM: turnLengthM,
      totalLengthM: workLengthM + turnLengthM,
      nonWorkingRatio: turnLengthM / (workLengthM + turnLengthM),
      turnStrategy: strategy,
      skipRows: effectiveSkip,
      skipRowSpacingM: (effectiveSkip + 1) * ds,
      turnCount: piTurnCount + omegaTurnCount,
      piTurnCount,
      omegaTurnCount,
      headlandCornerCount: cornerCount,
      headlandCornerArcM: cornerArcLengthM,
      headlandCornerTimeS: cornerTimeS,
      meanTurnLengthM: piTurnCount + omegaTurnCount > 0 ? turnLengthM / (piTurnCount + omegaTurnCount) : 0,
      turnTimeS: turnTimeS + cornerTimeS,
      maneuverTimeS: turnTimeS + cornerTimeS,
      workTimeS,
      totalTimeS,
      totalTimeH: totalTimeS / 3600,
      fieldEfficiencyHaPerH: totalTimeS > 0 ? fieldAreaM2 / 10000 / (totalTimeS / 3600) : 0,
      effectiveWorkEfficiencyHaPerH: workTimeS > 0 ? fieldAreaM2 / 10000 / (workTimeS / 3600) : 0,
      pathPointCount: pathPoints.length,
    };

    return {
      fieldId: this.field.id,
      fieldName: this.field.name,
      machineId: o.machineId,
      polygon: poly,
      innerPolygon,
      headlandLoops,
      sweepAngleDeg: bou.angleDeg,
      strips: bou.strips,
      orderedStrips,
      turnStrategy: strategy,
      workStrips,
      turns: turnPaths,
      turnCostDetail,
      path: pathPoints,
      raster,
      metrics,
    };
  }
}
