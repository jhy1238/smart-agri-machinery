/**
 * src/planning/dispatch.mjs
 * ---------------------------------------------------------------------------
 * 多机协同调度（构造式贪心 + 局部搜索），并与"朴素轮询分配"基线对比
 *
 * 问题建模（带时间窗与机具能力约束的并行机调度）：
 *   任务集合 T = {1..N}：每个任务有 作业面积、所需作业环节(能力)、时间窗 [e_i, l_i]、位置
 *   农机集合 M = {1..M}：每台农机有 能力集合、作业效率、转移速度、可用时刻、小时成本
 *   决策变量 x_{i,m} ∈ {0,1} 与顺序变量 —— 每台农机执行的任务序列
 *
 * 目标函数（越小越好）：
 *   J = w1 * 完工时间(h) + w2 * 空驶里程(km) + w3 * 等待时间(h)
 *       + w4 * 作业成本(千元) + w5 * 超出时间窗惩罚(h)
 *   权重默认 w1=1.0, w2=0.35, w3=0.20, w4=0.15, w5=5.0
 *
 * 求解流程：
 *   1) 构造式贪心：按时间窗与面积排序任务，逐个插入"使 J 增量最小"的农机与位置；
 *   2) 局部搜索：任务迁移(Or-opt)、机间交换(Swap)、机内逆序(2-opt)，直到无改进或达到迭代上限；
 *   3) 基线：按任务顺序轮询分配给可胜任农机（不做任何排序与改进），用于量化优化收益。
 */

export const DEFAULT_WEIGHTS = { w1: 1.0, w2: 0.35, w3: 0.2, w4: 0.15, w5: 5.0 };

function distance(a, b) {
  return Math.hypot(a.x - b.x, a.y - b.y);
}

/**
 * 对给定分配方案求时间轴与目标函数值。
 * @param {object} assignment {machineId: [taskId, ...]}
 */
export function evaluateSchedule(assignment, ctx) {
  const { tasks, machines, weights } = ctx;
  const taskById = new Map(tasks.map((t) => [t.id, t]));
  const machineById = new Map(machines.map((m) => [m.id, m]));
  const gantt = [];
  let makespanH = 0;
  let transitKm = 0;
  let waitH = 0;
  let costK = 0;
  let tardinessH = 0;
  const perMachine = {};
  for (const m of machines) {
    const seq = assignment[m.id] || [];
    let t = m.availableFromH;
    let pos = m.home;
    for (const taskId of seq) {
      const task = taskById.get(taskId);
      const distM = distance(pos, task.location);
      const transitH = distM / m.transitSpeedMps / 3600; // m / (m/s) = s，再换算为 h
      const arrival = t + transitH;
      const start = Math.max(arrival, task.timeWindowH[0]);
      const wait = start - arrival;
      const dur = taskDurationH(task, m);
      const end = start + dur;
      const late = Math.max(0, end - task.timeWindowH[1]);
      gantt.push({ machineId: m.id, taskId, type: 'transit', startH: t, endH: arrival, distanceKm: distM / 1000 });
      if (wait > 1e-9) gantt.push({ machineId: m.id, taskId, type: 'wait', startH: arrival, endH: start });
      gantt.push({ machineId: m.id, taskId, type: 'work', startH: start, endH: end });
      transitKm += distM / 1000;
      waitH += wait;
      tardinessH += late;
      t = end;
      pos = task.location;
    }
    perMachine[m.id] = { endH: t, busyH: t - m.availableFromH, taskCount: seq.length };
    costK += ((t - m.availableFromH) * m.costPerHour) / 1000;
    if (t > makespanH) makespanH = t;
  }
  const J =
    weights.w1 * makespanH +
    weights.w2 * transitKm +
    weights.w3 * waitH +
    weights.w4 * costK +
    weights.w5 * tardinessH;
  return {
    J,
    makespanH,
    transitKm,
    waitH,
    costK,
    tardinessH,
    perMachine,
    gantt: gantt.sort((a, b) => (a.machineId === b.machineId ? a.startH - b.startH : a.machineId < b.machineId ? -1 : 1)),
  };
}

/** 任务在指定农机上的作业时长（h）：优先使用预计算的机-任务时长表 */
export function taskDurationH(task, machine) {
  if (task.durationHByMachine && task.durationHByMachine[machine.id] !== undefined) {
    return task.durationHByMachine[machine.id];
  }
  const cap = machine.fieldCapacityHaPerH || 1.0;
  return task.areaHa / cap;
}

/** 农机是否胜任任务 */
export function machineCanDo(machine, task) {
  if (!task.capability) return true;
  return machine.capabilities.includes(task.capability);
}

/**
 * 朴素基线：按任务给定顺序，轮询分配给"下一台可胜任农机"，不排序、不改进。
 */
export function baselineNaiveAssignment(tasks, machines, ctx) {
  const assignment = {};
  for (const m of machines) assignment[m.id] = [];
  let cursor = 0;
  const skipped = [];
  for (const task of tasks) {
    let placed = false;
    for (let k = 0; k < machines.length; k += 1) {
      const m = machines[(cursor + k) % machines.length];
      if (!machineCanDo(m, task)) continue;
      assignment[m.id].push(task.id);
      cursor = (cursor + k + 1) % machines.length;
      placed = true;
      break;
    }
    if (!placed) skipped.push(task.id);
  }
  const ev = evaluateSchedule(assignment, ctx);
  return { assignment, ...ev, unassigned: skipped };
}

export class DispatchOptimizer {
  /**
   * @param {object} opts
   *   tasks     任务数组
   *   machines  农机数组
   *   weights   目标函数权重
   *   maxIter   局部搜索最大轮数
   */
  constructor(opts) {
    const o = Object.assign({ weights: DEFAULT_WEIGHTS, maxIter: 60 }, opts || {});
    this.ctx = { tasks: o.tasks, machines: o.machines, weights: { ...DEFAULT_WEIGHTS, ...(o.weights || {}) } };
    this.maxIter = o.maxIter;
    this.tasks = o.tasks;
    this.machines = o.machines;
  }

  /** 构造式贪心：任务按时间窗起点/面积排序，逐个插入使 J 增量最小的位置 */
  greedyConstruct() {
    const { tasks, machines } = this.ctx;
    const assignment = {};
    for (const m of machines) assignment[m.id] = [];
    const ordered = tasks.slice().sort((a, b) => {
      if (a.timeWindowH[0] !== b.timeWindowH[0]) return a.timeWindowH[0] - b.timeWindowH[0];
      return b.areaHa - a.areaHa;
    });
    let best = evaluateSchedule(assignment, this.ctx);
    for (const task of ordered) {
      let bestChoice = null;
      for (const m of machines) {
        if (!machineCanDo(m, task)) continue;
        const seq = assignment[m.id];
        for (let pos = 0; pos <= seq.length; pos += 1) {
          const trial = { ...assignment, [m.id]: seq.slice(0, pos).concat([task.id], seq.slice(pos)) };
          const ev = evaluateSchedule(trial, this.ctx);
          if (!bestChoice || ev.J < bestChoice.ev.J) bestChoice = { m, trial, ev };
        }
      }
      if (!bestChoice) throw new Error('任务 ' + task.id + ' 无可胜任农机（能力约束不满足）');
      assignment[bestChoice.m.id] = bestChoice.trial[bestChoice.m.id];
      best = bestChoice.ev;
    }
    return { assignment, evaluation: best };
  }

  /** 局部搜索：Or-opt 迁移 + 机间交换 + 机内 2-opt 逆序 */
  localSearch(assignment) {
    const { machines } = this.ctx;
    let current = {};
    for (const m of machines) current[m.id] = (assignment[m.id] || []).slice();
    let cur = evaluateSchedule(current, this.ctx);
    const log = [];
    for (let iter = 0; iter < this.maxIter; iter += 1) {
      let improved = false;
      // (a) Or-opt：把某台农机的一个任务迁到另一台农机（或同机其他位置）
      outer: for (const mFrom of machines) {
        for (let i = 0; i < current[mFrom.id].length; i += 1) {
          const taskId = current[mFrom.id][i];
          const task = this.tasks.find((t) => t.id === taskId);
          for (const mTo of machines) {
            if (!machineCanDo(mTo, task)) continue;
            for (let j = 0; j <= current[mTo.id].length; j += 1) {
              if (mTo.id === mFrom.id && (j === i || j === i + 1)) continue;
              const trial = {};
              for (const mm of machines) trial[mm.id] = current[mm.id].slice();
              trial[mFrom.id].splice(i, 1);
              const insertAt = mTo.id === mFrom.id && j > i ? j - 1 : j;
              trial[mTo.id].splice(insertAt, 0, taskId);
              const ev = evaluateSchedule(trial, this.ctx);
              if (ev.J < cur.J - 1e-9) {
                current = trial;
                log.push({ iter, move: 'or-opt', taskId, from: mFrom.id, to: mTo.id, dJ: ev.J - cur.J });
                cur = ev;
                improved = true;
                break outer;
              }
            }
          }
        }
      }
      if (improved) continue;
      // (b) 机间交换
      outer2: for (let a = 0; a < machines.length; a += 1) {
        for (let b = a + 1; b < machines.length; b += 1) {
          const mA = machines[a];
          const mB = machines[b];
          for (let i = 0; i < current[mA.id].length; i += 1) {
            for (let j = 0; j < current[mB.id].length; j += 1) {
              const tA = this.tasks.find((t) => t.id === current[mA.id][i]);
              const tB = this.tasks.find((t) => t.id === current[mB.id][j]);
              if (!machineCanDo(mB, tA) || !machineCanDo(mA, tB)) continue;
              const trial = {};
              for (const mm of machines) trial[mm.id] = current[mm.id].slice();
              trial[mA.id][i] = tB.id;
              trial[mB.id][j] = tA.id;
              const ev = evaluateSchedule(trial, this.ctx);
              if (ev.J < cur.J - 1e-9) {
                current = trial;
                log.push({ iter, move: 'swap', a: tA.id, b: tB.id, dJ: ev.J - cur.J });
                cur = ev;
                improved = true;
                break outer2;
              }
            }
          }
        }
      }
      if (improved) continue;
      // (c) 机内 2-opt 逆序
      outer3: for (const m of machines) {
        const seq = current[m.id];
        for (let i = 0; i < seq.length - 1; i += 1) {
          for (let j = i + 1; j < seq.length; j += 1) {
            const trial = {};
            for (const mm of machines) trial[mm.id] = current[mm.id].slice();
            trial[m.id] = seq.slice(0, i).concat(seq.slice(i, j + 1).reverse(), seq.slice(j + 1));
            const ev = evaluateSchedule(trial, this.ctx);
            if (ev.J < cur.J - 1e-9) {
              current = trial;
              log.push({ iter, move: '2-opt', machine: m.id, dJ: ev.J - cur.J });
              cur = ev;
              improved = true;
              break outer3;
            }
          }
        }
      }
      if (!improved) break;
    }
    return { assignment: current, evaluation: cur, log };
  }

  /** 完整求解：贪心构造 -> 局部搜索 -> 与朴素基线对比 */
  solve() {
    const greedy = this.greedyConstruct();
    const local = this.localSearch(greedy.assignment);
    const baseline = baselineNaiveAssignment(this.tasks, this.machines, this.ctx);
    const pct = (a, b) => (Math.abs(b) > 1e-12 ? ((b - a) / b) * 100 : 0);
    return {
      assignment: local.assignment,
      schedule: local.evaluation,
      greedyJ: greedy.evaluation.J,
      searchLog: local.log,
      baseline: { assignment: baseline.assignment, schedule: baseline, unassigned: baseline.unassigned },
      improvement: {
        objectivePct: pct(local.evaluation.J, baseline.J),
        makespanPct: pct(local.evaluation.makespanH, baseline.makespanH),
        transitPct: pct(local.evaluation.transitKm, baseline.transitKm),
        waitPct: pct(local.evaluation.waitH, baseline.waitH),
        costPct: pct(local.evaluation.costK, baseline.costK),
        tardinessPct: pct(local.evaluation.tardinessH, baseline.tardinessH),
        localSearchGainPct: pct(local.evaluation.J, greedy.evaluation.J),
      },
    };
  }
}

/** 导出甘特图数据（按农机分组，供前端/报表渲染） */
export function extractGantt(evaluation, ctx) {
  const rows = [];
  for (const m of ctx.machines) {
    const items = evaluation.gantt.filter((g) => g.machineId === m.id).sort((a, b) => a.startH - b.startH);
    rows.push({
      machineId: m.id,
      machineName: m.name,
      availableFromH: m.availableFromH,
      endH: evaluation.perMachine[m.id] ? evaluation.perMachine[m.id].endH : m.availableFromH,
      items,
    });
  }
  return rows;
}
