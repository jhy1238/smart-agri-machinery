/**
 * src/cloud/telemetry.mjs
 * ---------------------------------------------------------------------------
 * 农机物联网遥测链路：CAN/ISOBUS 报文采集 -> 编码 -> 上报 -> 云端存储 -> 异常检测 -> 报警
 *
 * 1) 总线层：J1939 / ISO 11783 29 位标识符
 *      bit 28-26  Priority(3) | bit 25 EDP(1) | bit 24 DP(1) | bit 23-16 PF(8) | bit 15-8 PS(8) | bit 7-0 SA(8)
 *      PGN = (DP << 16) | (PF << 8) | (PF < 240 ? 0 : PS)      （PDU1 时 PS 为目的地址，PDU2 时为组扩展）
 *    报文参数（SPN）按"分辨率 + 偏移"编码，例如：
 *      SPN 110 冷却液温度  PGN 65262  1 ℃/bit  偏移 -40
 *      SPN 190 发动机转速  PGN 61444  0.125 rpm/bit
 *      SPN 183 燃油率      PGN 65266  0.05 L/h per bit
 *      SPN 84  车速        PGN 65265  1/256 km/h per bit
 *      DM1 故障码 PGN 65226：DTC = SPN(19 bit) + FMI(5 bit) + OC(1 bit) + CM(1 bit)
 *
 * 2) 链路层：边缘网关 1 Hz 采样 -> 批量上行（模拟 1% 丢包、120 ms 时延）-> 云端内存时序库
 *
 * 3) 应用层：异常检测（多规则融合）
 *      - 绝对阈值规则：如冷却液温度 > 105 ℃ 严重、> 98 ℃ 警告
 *      - 滑动窗口 3-sigma：|x - mean_w| > k * sigma_w（窗口 60 s）
 *      - EWMA 控制图：z_k = lambda*x_k + (1-lambda)*z_{k-1}，控制限 mu +- 3*sigma*sqrt(lambda/(2-lambda))
 *      - 变化率规则：|x(t) - x(t-dt)| / dt 超限（识别燃油率突变等）
 *    报警去抖：同一 (设备, 信号, 规则) 在抑制窗口内只报一次，但统计触发次数。
 */

import { createRng } from '../sim/field_simulator.mjs';

/* =========================================================================
 * 1. J1939 / ISOBUS 报文编解码
 * ========================================================================= */

/** SPN 参数表：位置以字节为单位（byte 从 0 开始），数值 = raw * resolution + offset */
export const SPN_DEFS = {
  110: { name: '发动机冷却液温度', pgn: 65262, byte: 0, lenBits: 8, resolution: 1, offset: -40, unit: '℃', signal: 'coolantTempC' },
  175: { name: '发动机机油温度', pgn: 65262, byte: 3, lenBits: 8, resolution: 1, offset: -40, unit: '℃', signal: 'oilTempC' },
  190: { name: '发动机转速', pgn: 61444, byte: 3, lenBits: 16, resolution: 0.125, offset: 0, unit: 'rpm', signal: 'engineSpeedRpm' },
  513: { name: '发动机负荷率', pgn: 61444, byte: 2, lenBits: 8, resolution: 1, offset: -125, unit: '%', signal: 'engineLoadPct' },
  183: { name: '燃油消耗率', pgn: 65266, byte: 0, lenBits: 16, resolution: 0.05, offset: 0, unit: 'L/h', signal: 'fuelRateLph' },
  84: { name: '车速', pgn: 65265, byte: 1, lenBits: 16, resolution: 1 / 256, offset: 0, unit: 'km/h', signal: 'groundSpeedKph' },
  168: { name: '蓄电池电压', pgn: 65271, byte: 4, lenBits: 16, resolution: 0.05, offset: 0, unit: 'V', signal: 'batteryVoltageV' },
  // ---- 厂内自定义 PGN 65440（ISO 11783 允许的专有报文），承载农机作业参数 ----
  9001: { name: '液压油温度', pgn: 65440, byte: 0, lenBits: 8, resolution: 1, offset: -40, unit: '℃', signal: 'hydraulicOilTempC' },
  9002: { name: 'PTO 转速', pgn: 65440, byte: 2, lenBits: 16, resolution: 0.125, offset: 0, unit: 'rpm', signal: 'ptoRpm' },
  9003: { name: '尿素液位', pgn: 65440, byte: 5, lenBits: 8, resolution: 0.4, offset: 0, unit: '%', signal: 'defLevelPct' },
};

/** 信号 -> SPN 反查表 */
export const SIGNAL_TO_SPN = Object.fromEntries(Object.entries(SPN_DEFS).map(([spn, d]) => [d.signal, Number(spn)]));

/** J1939 FMI 故障模式标识（节选） */
export const FMI_TEXT = {
  0: '数据有效但高于正常工作范围（最严重）',
  1: '数据有效但低于正常工作范围（最严重）',
  2: '数据不稳定、间断或不正确',
  3: '电压高于正常范围',
  4: '电压低于正常范围',
  5: '电流低于正常范围或开路',
  16: '数据有效但高于正常工作范围（轻微）',
  17: '数据有效但低于正常工作范围（轻微）',
  18: '数据有效但低于正常工作范围（轻微）',
  31: '状态存在（条件满足）',
};

/** J1939 报文默认优先级（标准规定值）：EEC1 为 3，其余常用报文为 6 */
export const PGN_DEFAULT_PRIORITY = {
  61444: 3, // EEC1 电子发动机控制器 1
  65262: 6, // ET1 发动机温度 1
  65266: 6, // LFE 燃油经济性
  65265: 6, // CCVS 巡航控制/车速
  65271: 6, // VEP 车辆电气电源
  65226: 6, // DM1 活动故障码
  65440: 6, // 厂内专有报文
};

/** 编码 29 位 CAN 标识符 */
export function encodeCanId(pgn, sourceAddress, priority) {
  const pri = priority === undefined ? (PGN_DEFAULT_PRIORITY[pgn] === undefined ? 6 : PGN_DEFAULT_PRIORITY[pgn]) : priority;
  const dp = (pgn >> 16) & 0x01;
  const pf = (pgn >> 8) & 0xff;
  const ps = pgn & 0xff;
  return (((pri & 0x7) << 26) | (dp << 24) | (pf << 16) | (ps << 8) | (sourceAddress & 0xff)) >>> 0;
}

/** 解析 29 位 CAN 标识符 */
export function decodeCanId(canId) {
  const priority = (canId >>> 26) & 0x07;
  const dp = (canId >>> 24) & 0x01;
  const pf = (canId >>> 16) & 0xff;
  const ps = (canId >>> 8) & 0xff;
  const sa = canId & 0xff;
  const pgn = (dp << 16) | (pf << 8) | (pf < 240 ? 0 : ps);
  return { priority, dp, pf, ps, sa, pgn };
}

/** 把物理量按 SPN 定义写入数据场（小端） */
function writeSpn(data, def, value) {
  const raw = Math.max(0, Math.round((value - def.offset) / def.resolution));
  if (def.lenBits === 8) {
    data[def.byte] = raw & 0xff;
  } else {
    data[def.byte] = raw & 0xff;
    data[def.byte + 1] = (raw >> 8) & 0xff;
  }
}

function readSpn(data, def) {
  const raw = def.lenBits === 8 ? data[def.byte] : data[def.byte] | (data[def.byte + 1] << 8);
  return raw * def.resolution + def.offset;
}

/**
 * 编码一帧 CAN 报文（8 字节数据场）。
 * @param {number} pgn
 * @param {number} sa 源地址
 * @param {object} values {signal: value}
 */
export function encodeCanFrame(pgn, sa, values, priority) {
  const defs = Object.values(SPN_DEFS).filter((d) => d.pgn === pgn);
  if (!defs.length) throw new Error('未知 PGN: ' + pgn);
  const data = new Uint8Array(8).fill(0xff);
  let count = 0;
  for (const d of defs) {
    if (values[d.signal] === undefined) continue;
    writeSpn(data, d, values[d.signal]);
    count += 1;
  }
  if (!count) throw new Error('PGN ' + pgn + ' 无有效信号');
  return {
    canId: encodeCanId(pgn, sa, priority),
    pgn,
    sa,
    dlc: 8,
    data: Array.from(data),
    hex: Array.from(data).map((b) => b.toString(16).padStart(2, '0')).join(' '),
  };
}

/** 解码一帧 CAN 报文，返回物理量 */
export function decodeCanFrame(frame) {
  const id = decodeCanId(frame.canId);
  const defs = Object.values(SPN_DEFS).filter((d) => d.pgn === id.pgn);
  const data = Uint8Array.from(frame.data);
  const values = {};
  const spns = {};
  for (const d of defs) {
    values[d.signal] = readSpn(data, d);
    spns[d.signal] = Number(Object.keys(SPN_DEFS).find((k) => SPN_DEFS[k] === d));
  }
  return { ...id, dlc: frame.dlc, values, spns };
}

/**
 * 编码 DM1 故障码（J1939-73）：4 字节
 *   byte0        = SPN[7:0]
 *   byte1        = SPN[15:8] 的低 3 位 | FMI << 3
 *   byte2        = OC << 7 | CM << 6
 *   byte3        = 0xFF（保留）
 */
export function encodeDtc(dtc) {
  const spn = dtc.spn & 0x7ffff;
  const fmi = dtc.fmi & 0x1f;
  const oc = dtc.oc ? 1 : 0;
  const cm = dtc.cm ? 1 : 0;
  return [spn & 0xff, ((spn >> 8) & 0x07) | (fmi << 3), (oc << 7) | (cm << 6), 0xff];
}

export function decodeDtc(bytes) {
  const spn = bytes[0] | ((bytes[1] & 0x07) << 8);
  const fmi = (bytes[1] >> 3) & 0x1f;
  const oc = (bytes[2] >> 7) & 0x01;
  const cm = (bytes[2] >> 6) & 0x01;
  return { spn, fmi, oc, cm, fmiText: FMI_TEXT[fmi] || '未知 FMI ' + fmi };
}

/** 构造 DM1 报文（PGN 65226） */
export function encodeDm1(sa, dtcs) {
  const data = new Uint8Array(8).fill(0xff);
  // byte0/1: 灯状态（本原型置 0），byte2 起为 DTC
  data[0] = 0x00;
  data[1] = 0x00;
  let off = 2;
  const list = dtcs.slice(0, 1); // 单包 DM1 仅承载 1 个 DTC（其余需 BAM 传输）
  for (const d of list) {
    const b = encodeDtc(d);
    for (let i = 0; i < 4; i += 1) data[off + i] = b[i];
    off += 4;
  }
  return { canId: encodeCanId(65226, sa, 6), pgn: 65226, sa, dlc: 8, data: Array.from(data), dtcs: list };
}

export function decodeDm1(frame) {
  const data = Uint8Array.from(frame.data);
  const out = [];
  for (let off = 2; off + 3 < 8; off += 4) {
    const dtc = decodeDtc([data[off], data[off + 1], data[off + 2], data[off + 3]]);
    if (dtc.spn === 0 && dtc.fmi === 0) continue;
    if (dtc.spn === 524287) continue;
    out.push(dtc);
  }
  return out;
}

/* =========================================================================
 * 2. 云端时序库（内存实现）
 * ========================================================================= */

export class TelemetryCloud {
  constructor() {
    this.series = new Map(); // key: deviceId|signal -> {t:[], v:[]}
    this.alarms = [];
    this.framesIngested = 0;
    this.batchesIngested = 0;
    this.devices = new Map();
  }

  registerDevice(device) {
    this.devices.set(device.id, device);
  }

  /** 写入一批遥测记录 [{deviceId, t, values:{signal:value}, frames:[...]}] */
  ingest(records) {
    this.batchesIngested += 1;
    for (const r of records) {
      for (const [signal, value] of Object.entries(r.values)) {
        const key = r.deviceId + '|' + signal;
        let s = this.series.get(key);
        if (!s) {
          s = { t: [], v: [] };
          this.series.set(key, s);
        }
        s.t.push(r.t);
        s.v.push(value);
      }
      if (r.frames) this.framesIngested += r.frames.length;
    }
  }

  /** 时间区间查询 */
  query(deviceId, signal, t0, t1) {
    const s = this.series.get(deviceId + '|' + signal);
    if (!s) return [];
    const out = [];
    for (let i = 0; i < s.t.length; i += 1) {
      if (s.t[i] >= t0 && s.t[i] <= t1) out.push({ t: s.t[i], v: s.v[i] });
    }
    return out;
  }

  latest(deviceId, signal) {
    const s = this.series.get(deviceId + '|' + signal);
    if (!s || !s.t.length) return null;
    return { t: s.t[s.t.length - 1], v: s.v[s.v.length - 1] };
  }

  raiseAlarm(alarm) {
    this.alarms.push(alarm);
  }

  stats() {
    const signals = new Set();
    let points = 0;
    for (const [key, s] of this.series.entries()) {
      signals.add(key.split('|')[1]);
      points += s.t.length;
    }
    return {
      devices: this.devices.size,
      seriesKeys: this.series.size,
      signals: signals.size,
      points,
      frames: this.framesIngested,
      batches: this.batchesIngested,
      alarms: this.alarms.length,
      alarmsCritical: this.alarms.filter((a) => a.severity === 'critical').length,
      alarmsWarning: this.alarms.filter((a) => a.severity === 'warning').length,
    };
  }

  /** 导出为可序列化的对象（用于写入 data/run_result.json） */
  toJSON(limitPerSeries) {
    const lim = limitPerSeries === undefined ? 0 : limitPerSeries;
    const out = {};
    for (const [key, s] of this.series.entries()) {
      const n = lim > 0 ? Math.min(lim, s.t.length) : s.t.length;
      out[key] = { t: s.t.slice(-n), v: s.v.slice(-n).map((v) => Number(v.toFixed(4))) };
    }
    return out;
  }
}

/* =========================================================================
 * 3. 边缘网关：采集 -> 编码 -> 批量上行
 * ========================================================================= */

export class EdgeTelemetryGateway {
  /**
   * @param {object} opts {cloud, devices, sampleRateHz, dropRate, latencyMs, batchSize, seed, clockSkewMs}
   */
  constructor(opts) {
    const o = Object.assign(
      { sampleRateHz: 1, dropRate: 0.01, latencyMs: 120, batchSize: 5, seed: 20240918, clockSkewMs: 0 },
      opts || {},
    );
    this.opt = o;
    this.cloud = o.cloud;
    this.devices = o.devices || [];
    this.rng = createRng(o.seed);
    this.pending = []; // 待送达批次
    this.stats = {
      samplesCollected: 0,
      framesEncoded: 0,
      framesDropped: 0,
      batchesSent: 0,
      batchesLost: 0,
      bytesUplink: 0,
    };
  }

  /** 采集一次整车状态：编码为 CAN 报文 */
  collect(device, t, values) {
    this.stats.samplesCollected += 1;
    const frames = [];
    // 只保留可由 SPN 定义编码的物理量（剔除工况标志等非总线信号）
    const encodable = {};
    for (const d of Object.values(SPN_DEFS)) {
      if (values[d.signal] !== undefined) encodable[d.signal] = values[d.signal];
    }
    for (const pgn of [61444, 65262, 65266, 65265, 65271, 65440]) {
      try {
        frames.push(encodeCanFrame(pgn, device.canSourceAddress, encodable));
      } catch (e) {
        void e;
      }
    }
    this.stats.framesEncoded += frames.length;
    this.stats.bytesUplink += frames.length * 8;
    return { deviceId: device.id, t, values: encodable, frames };
  }

  /** 上报：按批封装，模拟丢包与时延 */
  upload(record, t) {
    this.batch = this.batch || [];
    this.batch.push(record);
    if (this.batch.length >= this.opt.batchSize) {
      const batch = this.batch;
      this.batch = [];
      const lost = this.rng.next() < this.opt.dropRate;
      const deliverAt = t + this.opt.latencyMs / 1000 + this.rng.normal(0, 0.01);
      if (lost) {
        this.stats.batchesLost += 1;
        this.stats.framesDropped += batch.reduce((s, r) => s + r.frames.length, 0);
      } else {
        this.pending.push({ deliverAt, batch });
        this.stats.batchesSent += 1;
      }
    }
  }

  /** 推进网关时钟，投递到期的批次到云端 */
  tick(t) {
    const remain = [];
    for (const p of this.pending) {
      if (p.deliverAt <= t) this.cloud.ingest(p.batch);
      else remain.push(p);
    }
    this.pending = remain;
  }

  flush(t) {
    if (this.batch && this.batch.length) {
      this.pending.push({ deliverAt: t, batch: this.batch });
      this.batch = [];
      this.stats.batchesSent += 1;
    }
    this.tick(t);
  }
}

/* =========================================================================
 * 4. 异常检测（阈值 + 3-sigma + EWMA + 变化率）
 * ========================================================================= */

/** 默认阈值规则（工程整定值） */
export const DEFAULT_RULES = {
  // minDelta：统计类规则（3-sigma / EWMA）额外要求的最小绝对偏差，用于抑制
  //           正常工况波动（如负荷调节引起的转速/油耗周期性变化）导致的误报
  coolantTempC: { warnHigh: 98, critHigh: 105, warnLow: 60, critLow: 50, spn: 110, fmiHigh: 0, fmiLow: 1, unit: '℃', label: '冷却液温度', minDelta: 4, coldStartMask: true },
  hydraulicOilTempC: { warnHigh: 85, critHigh: 95, warnLow: 20, critLow: 10, spn: 9001, fmiHigh: 0, fmiLow: 1, unit: '℃', label: '液压油温度', minDelta: 4, coldStartMask: true },
  engineSpeedRpm: { warnHigh: 2400, critHigh: 2600, warnLow: 600, critLow: 500, spn: 190, fmiHigh: 0, fmiLow: 1, unit: 'rpm', label: '发动机转速', minDelta: 250 },
  fuelRateLph: { rateWindowS: 10, ratePctLimit: 40, spn: 183, fmi: 2, unit: 'L/h', label: '燃油消耗率', minDelta: 8 },
  batteryVoltageV: { warnLow: 24.5, critLow: 23.5, warnHigh: 29.0, critHigh: 30.0, spn: 168, fmiLow: 1, fmiHigh: 0, unit: 'V', label: '蓄电池电压', minDelta: 1.2 },
  defLevelPct: { warnLow: 15, critLow: 5, spn: 9003, fmiLow: 1, unit: '%', label: '尿素液位', minDelta: 10 },
  ptoRpm: { warnHigh: 620, critHigh: 700, spn: 9002, fmiHigh: 0, unit: 'rpm', label: 'PTO 转速', minDelta: 40 },
};

export class AnomalyDetector {
  /**
   * @param {object} opts
   *   windowSizeS    滑动窗口长度 (s)
   *   sigmaK         3-sigma 倍数
   *   ewmaLambda     EWMA 平滑系数
   *   ewmaSigmaK     EWMA 控制限倍数
   *   suppressionS   同类报警抑制窗口 (s)
   *   rules          阈值规则表
   */
  constructor(opts) {
    const o = Object.assign(
      { windowSizeS: 60, sigmaK: 3.0, ewmaLambda: 0.2, ewmaSigmaK: 3.0, suppressionS: 120, minSamples: 20, minSigma: 1e-6, warmupS: 300, coldStartS: 300 },
      opts || {},
    );
    this.opt = o;
    this.rules = o.rules || DEFAULT_RULES;
    this.state = new Map(); // key device|signal -> {win:[], ewma, ewmaVar, lastAlarm:{rule:t}, triggers:{}}
  }

  stateOf(deviceId, signal) {
    const key = deviceId + '|' + signal;
    let s = this.state.get(key);
    if (!s) {
      s = { win: [], ewma: null, ewmaVar: null, lastAlarm: {}, triggers: {}, maxAbsZ: 0 };
      this.state.set(key, s);
    }
    return s;
  }

  /**
   * 推送一个采样值，返回本次触发的报警数组。
   * @param {string} deviceId
   * @param {string} signal
   * @param {number} value
   * @param {number} t 秒
   * @param {object} meta {machineName}
   */
  push(deviceId, signal, value, t, meta) {
    const st = this.stateOf(deviceId, signal);
    const alarms = [];
    const rule = this.rules[signal] || {};
    // 作业状态门控：停机/怠速工况下仅保留绝对阈值（高温、低压等安全相关）判定，
    // 统计类规则（3-sigma / EWMA / 变化率）不判定，避免正常的启停过渡产生误报
    const idle = !!(meta && meta.working === false);
    const emit = (ruleName, severity, message, extra) => {
      st.triggers[ruleName] = (st.triggers[ruleName] || 0) + 1;
      const last = st.lastAlarm[ruleName];
      if (last !== undefined && t - last < this.opt.suppressionS) return; // 抑制窗口内不重复报警
      st.lastAlarm[ruleName] = t;
      const spn = extra && extra.spn !== undefined ? extra.spn : rule.spn;
      const fmi = extra && extra.fmi !== undefined ? extra.fmi : 0;
      const dtc = spn ? { spn, fmi, oc: 0, cm: 0 } : null;
      alarms.push({
        t,
        deviceId,
        machineName: meta && meta.machineName ? meta.machineName : deviceId,
        signal,
        label: rule.label || signal,
        value: Number(value.toFixed(3)),
        rule: ruleName,
        severity,
        message,
        spn: spn || null,
        fmi: spn ? fmi : null,
        fmiText: spn ? FMI_TEXT[fmi] || null : null,
        dtcBytes: dtc ? encodeDtc(dtc) : null,
      });
    };

    // ---- 规则 1：绝对阈值 ----
    if (rule.critHigh !== undefined && value >= rule.critHigh) {
      emit('绝对阈值', 'critical', (rule.label || signal) + ' ' + value.toFixed(1) + (rule.unit || '') + ' 超过严重上限 ' + rule.critHigh + (rule.unit || ''), { fmi: rule.fmiHigh });
    } else if (rule.warnHigh !== undefined && value >= rule.warnHigh) {
      emit('绝对阈值', 'warning', (rule.label || signal) + ' ' + value.toFixed(1) + (rule.unit || '') + ' 超过警告上限 ' + rule.warnHigh + (rule.unit || ''), { fmi: rule.fmiHigh !== undefined ? rule.fmiHigh : 16 });
    }
    // 冷启动抑制：起动暖机阶段（默认 300 s）不判低温类阈值，避免误报
    const coldMasked = rule.coldStartMask === true && t < this.opt.coldStartS;
    if (!coldMasked && rule.critLow !== undefined && value <= rule.critLow) {
      emit('绝对阈值', 'critical', (rule.label || signal) + ' ' + value.toFixed(1) + (rule.unit || '') + ' 低于严重下限 ' + rule.critLow + (rule.unit || ''), { fmi: rule.fmiLow });
    } else if (!coldMasked && rule.warnLow !== undefined && value <= rule.warnLow) {
      emit('绝对阈值', 'warning', (rule.label || signal) + ' ' + value.toFixed(1) + (rule.unit || '') + ' 低于警告下限 ' + rule.warnLow + (rule.unit || ''), { fmi: rule.fmiLow !== undefined ? rule.fmiLow : 17 });
    }

    // ---- 规则 2/3：滑动窗口统计 ----
    st.win.push({ t, v: value });
    while (st.win.length && t - st.win[0].t > this.opt.windowSizeS) st.win.shift();
    if (st.win.length >= this.opt.minSamples) {
      const n = st.win.length;
      let mean = 0;
      for (const p of st.win) mean += p.v;
      mean /= n;
      let varSum = 0;
      for (const p of st.win) varSum += (p.v - mean) * (p.v - mean);
      const sigma = Math.sqrt(varSum / Math.max(1, n - 1));
      const minDelta = rule.minDelta === undefined ? 0 : rule.minDelta;
      const warmedUp = t >= this.opt.warmupS && !idle;
      // ---- 规则 2：3-sigma ----
      if (sigma > this.opt.minSigma) {
        const z = Math.abs(value - mean) / sigma;
        if (z > st.maxAbsZ) st.maxAbsZ = z;
        if (warmedUp && z > this.opt.sigmaK && Math.abs(value - mean) >= minDelta) {
          emit('3-sigma', 'warning', (rule.label || signal) + ' 偏离滑动窗口均值 ' + z.toFixed(1) + ' sigma（窗口 ' + this.opt.windowSizeS + ' s，均值 ' + mean.toFixed(2) + '，标准差 ' + sigma.toFixed(3) + '）');
        }
      }
      // ---- 规则 3：EWMA 控制图 ----
      const lambda = this.opt.ewmaLambda;
      if (st.ewma === null) {
        st.ewma = value;
        st.ewmaVar = sigma * sigma;
      } else {
        const prev = st.ewma;
        st.ewma = lambda * value + (1 - lambda) * prev;
        st.ewmaVar = lambda * (value - prev) * (value - prev) + (1 - lambda) * st.ewmaVar;
        const sEwma = Math.sqrt(Math.max(0, st.ewmaVar) * (lambda / (2 - lambda)));
        if (warmedUp && sEwma > this.opt.minSigma && Math.abs(st.ewma - mean) > this.opt.ewmaSigmaK * sEwma && Math.abs(st.ewma - mean) >= minDelta) {
          emit('EWMA控制图', 'warning', (rule.label || signal) + ' EWMA 统计量 ' + st.ewma.toFixed(2) + ' 超出控制限（均值 ' + mean.toFixed(2) + '，EWMA 标准差 ' + sEwma.toFixed(3) + '，lambda=' + lambda + '）');
        }
      }
    }

    // ---- 规则 4：变化率 ----
    if (rule.ratePctLimit !== undefined && st.win.length >= 2) {
      const winS = rule.rateWindowS || 10;
      let ref = null;
      for (let i = st.win.length - 1; i >= 0; i -= 1) {
        if (t - st.win[i].t >= winS) {
          ref = st.win[i];
          break;
        }
      }
      if (!ref && st.win.length) ref = st.win[0];
      if (ref && Math.abs(ref.v) > 1e-6 && t >= this.opt.warmupS && !idle) {
        const pct = ((value - ref.v) / Math.abs(ref.v)) * 100;
        if (Math.abs(pct) > rule.ratePctLimit) {
          emit('变化率', Math.abs(pct) > rule.ratePctLimit * 1.5 ? 'critical' : 'warning', (rule.label || signal) + ' 在 ' + winS + ' s 内变化 ' + pct.toFixed(1) + '%（' + ref.v.toFixed(2) + ' -> ' + value.toFixed(2) + ' ' + (rule.unit || '') + '）');
        }
      }
    }
    return alarms;
  }

  /** 汇总各信号的统计量（供报告使用） */
  summary() {
    const out = [];
    for (const [key, s] of this.state.entries()) {
      const [deviceId, signal] = key.split('|');
      out.push({ deviceId, signal, samples: s.win.length, maxAbsZ: Number(s.maxAbsZ.toFixed(2)), triggers: { ...s.triggers } });
    }
    return out;
  }
}

/* =========================================================================
 * 5. 端到端遥测场景（含故障注入）
 * ========================================================================= */

/**
 * 生成农机作业工况时序（含故障注入）：
 *   M1 冷却液温度从 12 min 起爬升（节温器卡滞）
 *   M2 转速在 18 min 起出现异常尖峰并伴随燃油率上跳（调速器/喷油异常）
 *   M3 燃油率在 25 min 起突降至近零而转速正常（油路堵塞/传感器故障）
 */
export function generateMachineState(device, tSec, faults, rng) {
  const t = tSec / 60; // min
  // 15 min 作业循环：起步 60 s 升速、作业 11 min、降怠速 90 s（过渡平滑，符合实际操作）
  const cyc = tSec % 900;
  let workFactor;
  if (cyc < 60) workFactor = cyc / 60;
  else if (cyc < 720) workFactor = 1;
  else if (cyc < 810) workFactor = 1 - (cyc - 720) / 90;
  else workFactor = 0;
  const working = workFactor > 0.5;
  const baseLoad = 12 + (62 + 14 * Math.sin(tSec / 37) - 12) * workFactor;
  let engineSpeedRpm = 900 + (2050 - 900) * workFactor + 40 * Math.sin(tSec / 11) * workFactor + rng.normal(0, 12);
  const groundSpeedKph = (8.0 + 0.6 * Math.sin(tSec / 23)) * workFactor + rng.normal(0, 0.15);
  let fuelRateLph = (2.5 + 0.02 * engineSpeedRpm * (baseLoad / 100)) + rng.normal(0, 0.25);
  let coolantTempC = 45 + (88 - 45) * Math.min(1, tSec / 300) + 2.5 * Math.sin(tSec / 120) + rng.normal(0, 0.35);
  let hydraulicOilTempC = 30 + (62 - 30) * workFactor + 3 * Math.sin(tSec / 200) + rng.normal(0, 0.4);
  const batteryVoltageV = 27.8 + 0.2 * Math.sin(tSec / 300) + rng.normal(0, 0.05);
  const defLevelPct = 62 - tSec / 3600;
  const ptoRpm = 540 * workFactor + rng.normal(0, 4);

  const fired = [];
  for (const f of faults) {
    if (tSec < f.startS) continue;
    if (f.endS !== undefined && tSec > f.endS) continue;
    fired.push(f.kind);
    if (f.kind === 'coolant-overheat') {
      // 节温器卡滞：冷却液温度线性爬升
      coolantTempC += Math.min(30, (tSec - f.startS) / 60 * (f.ratePerMin || 2.2));
      hydraulicOilTempC += Math.min(14, (tSec - f.startS) / 60 * 1.1);
    } else if (f.kind === 'rpm-spike') {
      if (Math.floor(tSec / 20) % 3 === 0) engineSpeedRpm += 520 + rng.normal(0, 60);
      fuelRateLph *= 1.55;
    } else if (f.kind === 'fuel-drop') {
      fuelRateLph *= 0.06;
    } else if (f.kind === 'battery-low') {
      // 发电机故障：电压缓降
    }
  }
  return {
    engineSpeedRpm,
    coolantTempC,
    hydraulicOilTempC,
    fuelRateLph,
    groundSpeedKph,
    batteryVoltageV,
    defLevelPct,
    engineLoadPct: working ? baseLoad : 10,
    ptoRpm,
    working,
    _firedFaults: fired,
  };
}

/**
 * 运行完整遥测场景：采样 -> 编码 -> 上行 -> 存储 -> 检测 -> 报警
 * @param {object} opts {machines, durationMin, sampleRateHz, faults, seed}
 */
export function runTelemetryScenario(opts) {
  const o = Object.assign({ durationMin: 30, sampleRateHz: 1, seed: 777 }, opts || {});
  const machines = o.machines;
  const faults = o.faults || {
    M1: [{ kind: 'coolant-overheat', startS: 12 * 60, ratePerMin: 2.2, label: '节温器卡滞导致冷却液温度持续爬升' }],
    M2: [{ kind: 'rpm-spike', startS: 18 * 60, endS: 24 * 60, label: '发动机转速异常尖峰（调速器/喷油异常）' }],
    M3: [{ kind: 'fuel-drop', startS: 25 * 60, label: '燃油消耗率突降至近零（油路堵塞/传感器故障）' }],
  };
  const rng = createRng(o.seed);
  const cloud = new TelemetryCloud();
  for (const m of machines) cloud.registerDevice({ id: m.id, name: m.name, canSourceAddress: m.canSourceAddress });
  const gateway = new EdgeTelemetryGateway({ cloud, devices: machines, sampleRateHz: o.sampleRateHz, seed: o.seed + 1 });
  const detector = new AnomalyDetector({});
  const durationS = o.durationMin * 60;
  const dt = 1 / o.sampleRateHz;
  const alarms = [];
  const dtcFrames = [];
  const faultLog = [];
  const truth = new Map();
  for (const m of machines) truth.set(m.id, { coolantTempC: [], engineSpeedRpm: [], fuelRateLph: [] });

  for (const m of machines) {
    for (const f of faults[m.id] || []) {
      faultLog.push({ deviceId: m.id, machineName: m.name, kind: f.kind, startS: f.startS, endS: f.endS === undefined ? durationS : f.endS, label: f.label });
    }
  }

  for (let t = 0; t <= durationS; t += dt) {
    for (const m of machines) {
      const st = generateMachineState(m, t, faults[m.id] || [], rng);
      const record = gateway.collect(m, t, st);
      gateway.upload(record, t);
      const tr = truth.get(m.id);
      tr.coolantTempC.push(st.coolantTempC);
      tr.engineSpeedRpm.push(st.engineSpeedRpm);
      tr.fuelRateLph.push(st.fuelRateLph);
      for (const signal of Object.keys(DEFAULT_RULES)) {
        if (st[signal] === undefined) continue;
        const fired = detector.push(m.id, signal, st[signal], t, { machineName: m.name, working: st.working });
        for (const a of fired) {
          alarms.push(a);
          cloud.raiseAlarm(a);
          dtcFrames.push({ t, deviceId: m.id, dm1: encodeDm1(m.canSourceAddress, [{ spn: a.spn, fmi: a.fmi, oc: 0, cm: 0 }]) });
        }
      }
    }
    gateway.tick(t);
  }
  gateway.flush(durationS);

  // 报警按时间排序并计算检出时延
  alarms.sort((a, b) => a.t - b.t);
  for (const f of faultLog) {
    const related = alarms.filter((a) => a.deviceId === f.deviceId && a.t >= f.startS - 1);
    const crit = related.filter((a) => a.severity === 'critical');
    f.firstAlarmS = related.length ? related[0].t : null;
    f.firstCriticalS = crit.length ? crit[0].t : null;
    f.detected = related.length > 0;
    f.detectedCritical = crit.length > 0;
    f.latencyS = related.length ? related[0].t - f.startS : null;
    f.alarmCount = related.length;
    f.alarmRules = Array.from(new Set(related.map((a) => a.rule)));
  }

  return {
    cloud,
    gateway,
    detector,
    alarms,
    faultLog,
    dtcFrames,
    params: { durationMin: o.durationMin, sampleRateHz: o.sampleRateHz },
    stats: {
      ...cloud.stats(),
      gateway: { ...gateway.stats },
      detectionRate: faultLog.length ? faultLog.filter((f) => f.detected).length / faultLog.length : 0,
      meanLatencyS: (() => {
        const l = faultLog.filter((f) => f.latencyS !== null).map((f) => f.latencyS);
        return l.length ? l.reduce((a, b) => a + b, 0) / l.length : null;
      })(),
    },
  };
}
