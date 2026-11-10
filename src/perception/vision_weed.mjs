/**
 * src/perception/vision_weed.mjs
 * ---------------------------------------------------------------------------
 * 基于机器视觉的田间杂草识别（超绿指数分割 + 形态学去噪 + 连通域特征分类）
 *
 * 算法流程：
 *   1) 超绿指数   ExG = 2G - R - B          （0~255 量化域，植被显著为正）
 *   2) 阈值分割   mask = ExG > 20           （得到植被/土壤二值图）
 *   3) 形态学去噪 3x3 十字结构元 开运算(先腐蚀后膨胀)去除孤立噪点，闭运算填补叶隙
 *   4) 连通域标记 8 邻域两遍扫描法 + 并查集(Union-Find)等价类合并
 *   5) 特征分类   面积 / 行内位置(距作物行中心线距离) / 长宽比 / 紧凑度
 *                 -> 作物(crop) 或 杂草(weed)
 *   6) 密度成图   1 m x 1 m 栅格统计杂草株数密度(株/m^2)，并生成变量喷药处方图
 *
 * 分类判据（工程参数，可随作物/机型标定）：
 *   - 行内且面积 >= 0.030 m^2  -> 作物（行播作物的行带冠层）
 *   - 其余面积 >= 0.010 m^2 的植被团块 -> 杂草
 *   - 面积 <  0.010 m^2 的团块视为噪声丢弃
 *   注：行内杂草(与作物行带粘连)在单目 RGB 下难以区分，需借助多光谱或深度学习模型，
 *       本原型如实统计其漏检（体现在召回率上），不做"作弊"式真值利用。
 */

/* =========================================================================
 * 1. 超绿指数与分割
 * ========================================================================= */

/** 单像素超绿指数 ExG = 2G - R - B（范围 -510 ~ 510，常用 0~255 量化域） */
export function exgIndex(r, g, b) {
  return 2 * g - r - b;
}

/**
 * 计算整幅影像的 ExG 图。
 * @param {{width:number,height:number,rgb:Uint8Array}} image
 * @returns {Int16Array}
 */
export function computeExgMap(image) {
  const n = image.width * image.height;
  const out = new Int16Array(n);
  for (let i = 0; i < n; i += 1) {
    const k = i * 3;
    out[i] = 2 * image.rgb[k + 1] - image.rgb[k] - image.rgb[k + 2];
  }
  return out;
}

/**
 * Otsu 大津法自动阈值：在 ExG 直方图上最大化类间方差，
 * 相比固定阈值能自适应不同光照/土壤含水率条件（工程上常与固定阈值互为备份）。
 * @returns {number}
 */
export function otsuThreshold(exgMap) {
  const hist = new Int32Array(1024);
  let total = 0;
  for (let i = 0; i < exgMap.length; i += 1) {
    const v = Math.min(1023, Math.max(0, exgMap[i] + 511));
    hist[v] += 1;
    total += 1;
  }
  let sum = 0;
  for (let i = 0; i < 1024; i += 1) sum += i * hist[i];
  let sumB = 0;
  let wB = 0;
  let best = 0;
  let bestVar = -1;
  for (let t = 0; t < 1024; t += 1) {
    wB += hist[t];
    if (wB === 0) continue;
    const wF = total - wB;
    if (wF === 0) break;
    sumB += t * hist[t];
    const mB = sumB / wB;
    const mF = (sum - sumB) / wF;
    const between = wB * wF * (mB - mF) * (mB - mF);
    if (between > bestVar) {
      bestVar = between;
      best = t;
    }
  }
  return best - 511;
}

/**
 * 阈值分割：ExG > threshold 判为植被。
 * @returns {Uint8Array} 0/1 二值掩膜
 */
export function segmentVegetation(exgMap, width, height, threshold) {
  const t = threshold === undefined ? 20 : threshold;
  const mask = new Uint8Array(width * height);
  for (let i = 0; i < mask.length; i += 1) mask[i] = exgMap[i] > t ? 1 : 0;
  return mask;
}

/* =========================================================================
 * 2. 形态学运算（3x3 结构元）
 * ========================================================================= */

const CROSS = [
  [0, 1, 0],
  [1, 1, 1],
  [0, 1, 0],
];
const SQUARE = [
  [1, 1, 1],
  [1, 1, 1],
  [1, 1, 1],
];

function morph(mask, w, h, se, mode) {
  const out = new Uint8Array(w * h);
  for (let y = 0; y < h; y += 1) {
    for (let x = 0; x < w; x += 1) {
      let hit = 0;
      let total = 0;
      for (let dy = -1; dy <= 1; dy += 1) {
        for (let dx = -1; dx <= 1; dx += 1) {
          if (!se[dy + 1][dx + 1]) continue;
          total += 1;
          const yy = y + dy;
          const xx = x + dx;
          const v = yy < 0 || yy >= h || xx < 0 || xx >= w ? 0 : mask[yy * w + xx];
          if (v) hit += 1;
        }
      }
      out[y * w + x] = mode === 'erode' ? (hit === total ? 1 : 0) : hit > 0 ? 1 : 0;
    }
  }
  return out;
}

export function morphErode(mask, w, h, se) {
  return morph(mask, w, h, se || CROSS, 'erode');
}

export function morphDilate(mask, w, h, se) {
  return morph(mask, w, h, se || CROSS, 'dilate');
}

/** 开运算：先腐蚀后膨胀，去除小于结构元的孤立噪点 */
export function morphOpen(mask, w, h, iterations) {
  const it = iterations === undefined ? 1 : iterations;
  let m = mask;
  for (let i = 0; i < it; i += 1) m = morphErode(m, w, h, CROSS);
  for (let i = 0; i < it; i += 1) m = morphDilate(m, w, h, CROSS);
  return m;
}

/** 闭运算：先膨胀后腐蚀，填补叶片间的小孔洞 */
export function morphClose(mask, w, h, iterations) {
  const it = iterations === undefined ? 1 : iterations;
  let m = mask;
  for (let i = 0; i < it; i += 1) m = morphDilate(m, w, h, SQUARE);
  for (let i = 0; i < it; i += 1) m = morphErode(m, w, h, SQUARE);
  return m;
}

/* =========================================================================
 * 2.5 作物行自动检测（行投影峰值法）
 * ========================================================================= */

/**
 * 作物行检测：对植被掩膜做行方向投影，平滑后取峰值得到行中心线。
 * 这是行播作物田间视觉系统的标准做法（Hough 变换的简化等价形式），
 * 得到的行中心线用于划分"行带"（作物）与"行间"（杂草候选）。
 *
 * @param {Uint8Array} mask 植被掩膜
 * @param {object} opts {expectedSpacingM, minPeakRatio, smoothWindowM}
 * @returns {{rowCentersM:number[], rowCentersPx:number[], spacingM:number, peakStrength:number[]}}
 */
export function estimateCropRows(mask, w, h, mPerPx, opts) {
  const o = Object.assign({ expectedSpacingM: 0.6, minPeakRatio: 1.25, smoothWindowM: 0.09 }, opts || {});
  const proj = new Float64Array(h);
  for (let y = 0; y < h; y += 1) {
    let s = 0;
    for (let x = 0; x < w; x += 1) s += mask[y * w + x];
    proj[y] = s;
  }
  const win = Math.max(1, Math.round(o.smoothWindowM / mPerPx));
  const sm = new Float64Array(h);
  for (let y = 0; y < h; y += 1) {
    let s = 0;
    let n = 0;
    for (let k = -win; k <= win; k += 1) {
      const yy = y + k;
      if (yy < 0 || yy >= h) continue;
      s += proj[yy];
      n += 1;
    }
    sm[y] = n ? s / n : 0;
  }
  let mean = 0;
  for (let y = 0; y < h; y += 1) mean += sm[y];
  mean /= h;
  const minSepPx = Math.max(2, Math.round((o.expectedSpacingM * 0.5) / mPerPx));
  const centersPx = [];
  for (let y = 1; y < h - 1; y += 1) {
    if (sm[y] < sm[y - 1] || sm[y] < sm[y + 1]) continue;
    if (sm[y] < mean * o.minPeakRatio) continue;
    if (centersPx.length && y - centersPx[centersPx.length - 1] < minSepPx) {
      // 保留更强者
      if (sm[y] > sm[centersPx[centersPx.length - 1]]) centersPx[centersPx.length - 1] = y;
      continue;
    }
    centersPx.push(y);
  }
  // 亚像素细化：以峰值为中心在 +-win 内取加权质心
  const refined = centersPx.map((y) => {
    let s = 0;
    let sw = 0;
    for (let k = -win; k <= win; k += 1) {
      const yy = y + k;
      if (yy < 0 || yy >= h) continue;
      s += sm[yy] * yy;
      sw += sm[yy];
    }
    return sw > 0 ? s / sw : y;
  });
  const rowCentersM = refined.map((y) => y * mPerPx);
  let spacing = o.expectedSpacingM;
  if (rowCentersM.length > 1) {
    spacing = (rowCentersM[rowCentersM.length - 1] - rowCentersM[0]) / (rowCentersM.length - 1);
  }
  return { rowCentersM, rowCentersPx: refined, spacingM: spacing, peakStrength: centersPx.map((y) => sm[y]) };
}

/* =========================================================================
 * 3. 连通域标记（两遍扫描法 + 并查集）
 * ========================================================================= */

class UnionFind {
  constructor() {
    this.parent = [0];
  }
  add() {
    this.parent.push(this.parent.length);
    return this.parent.length - 1;
  }
  find(x) {
    let r = x;
    while (this.parent[r] !== r) r = this.parent[r];
    // 路径压缩
    let c = x;
    while (this.parent[c] !== r) {
      const nxt = this.parent[c];
      this.parent[c] = r;
      c = nxt;
    }
    return r;
  }
  union(a, b) {
    const ra = this.find(a);
    const rb = this.find(b);
    if (ra !== rb) this.parent[Math.max(ra, rb)] = Math.min(ra, rb);
  }
}

/**
 * 两遍扫描连通域标记。
 * @param {Uint8Array} mask 二值掩膜
 * @param {number} connectivity 4 或 8
 * @returns {{labels:Int32Array,count:number,components:Array}}
 */
export function connectedComponents(mask, w, h, connectivity) {
  const conn = connectivity === undefined ? 8 : connectivity;
  const uf = new UnionFind();
  const labels = new Int32Array(w * h);
  const neigh = conn === 8
    ? [
        [-1, -1],
        [0, -1],
        [1, -1],
        [-1, 0],
      ]
    : [[-1, 0], [0, -1]];
  // 第一遍：临时标号 + 等价对合并
  for (let y = 0; y < h; y += 1) {
    for (let x = 0; x < w; x += 1) {
      const idx = y * w + x;
      if (!mask[idx]) continue;
      let best = 0;
      const found = [];
      for (const [dx, dy] of neigh) {
        const xx = x + dx;
        const yy = y + dy;
        if (xx < 0 || yy < 0 || xx >= w || yy >= h) continue;
        const l = labels[yy * w + xx];
        if (l) found.push(l);
      }
      if (found.length === 0) {
        best = uf.add();
      } else {
        best = Math.min(...found);
        for (const l of found) uf.union(best, l);
      }
      labels[idx] = best;
    }
  }
  // 第二遍：统一为根标号并重排为连续 id
  const remap = new Map();
  const components = [];
  for (let i = 0; i < labels.length; i += 1) {
    if (!labels[i]) continue;
    const root = uf.find(labels[i]);
    let id = remap.get(root);
    if (id === undefined) {
      id = components.length + 1;
      remap.set(root, id);
      components.push({
        id,
        areaPx: 0,
        sumX: 0,
        sumY: 0,
        minX: w,
        minY: h,
        maxX: -1,
        maxY: -1,
      });
    }
    const c = components[id - 1];
    const x = i % w;
    const y = (i - x) / w;
    c.areaPx += 1;
    c.sumX += x;
    c.sumY += y;
    if (x < c.minX) c.minX = x;
    if (x > c.maxX) c.maxX = x;
    if (y < c.minY) c.minY = y;
    if (y > c.maxY) c.maxY = y;
    labels[i] = id;
  }
  for (const c of components) {
    c.centroidPx = { x: c.sumX / c.areaPx, y: c.sumY / c.areaPx };
    c.widthPx = c.maxX - c.minX + 1;
    c.heightPx = c.maxY - c.minY + 1;
  }
  return { labels, count: components.length, components };
}

/* =========================================================================
 * 4. 特征提取与作物/杂草分类
 * ========================================================================= */

/**
 * 连通域特征提取 + 规则分类。
 *
 * 核心特征为"行带外像素占比"：把作物行中心线两侧 +-rowHalfWidthM 定义为行带，
 * 行带内植被视为行播作物冠层，行带外植被视为杂草。相比仅用质心位置判断，
 * 该特征可以正确处理"行间杂草与作物行带粘连"的连通域（此类团块质心可能落在行带内，
 * 但其大部分像素位于行带外）。
 *
 * @param {Array} components connectedComponents 的输出
 * @param {object} ctx
 *   labels        连通域标号图（用于精确统计团块内像素）
 *   width,height  影像尺寸
 *   mPerPx        空间分辨率 (m/px)
 *   rowCentersM   作物行中心线位置（影像坐标系 y，单位 m）
 *   rowHalfWidthM 行带半宽 (m)，默认 0.15
 *   minWeedAreaM2 最小杂草团块面积 (m^2)，默认 0.010
 *   minOutOfBandAreaM2 判为杂草所需的最小行带外面积 (m^2)，默认 0.008
 *   minCropAreaM2 行带内判为作物的最小面积 (m^2)，默认 0.020
 *   exgMap        可选，用于统计团块平均 ExG
 * @returns {Array} 带特征与分类结果的团块列表
 */
export function classifyComponents(components, ctx) {
  const o = Object.assign(
    {
      mPerPx: 0.02,
      rowCentersM: [],
      rowHalfWidthM: 0.15,
      minWeedAreaM2: 0.004,
      minOutOfBandAreaM2: 0.004,
      minCropAreaM2: 0.015,
      exgMap: null,
      labels: null,
      width: 0,
      height: 0,
    },
    ctx || {},
  );
  const out = [];
  for (const c of components) {
    const areaM2 = c.areaPx * o.mPerPx * o.mPerPx;
    const cxM = c.centroidPx.x * o.mPerPx;
    const cyM = c.centroidPx.y * o.mPerPx;
    let rowOffsetM = Infinity;
    let nearestRow = null;
    for (const rc of o.rowCentersM) {
      const d = Math.abs(cyM - rc);
      if (d < rowOffsetM) {
        rowOffsetM = d;
        nearestRow = rc;
      }
    }
    const elong = Math.max(c.widthPx, c.heightPx) / Math.max(1, Math.min(c.widthPx, c.heightPx));
    const compactness = c.areaPx / (c.widthPx * c.heightPx);
    // 逐像素统计行带内/行带外像素、平均 ExG、最大越带距离
    let inBandPx = 0;
    let outOfBandPx = 0;
    let exgSum = 0;
    let maxOutOffsetM = 0;
    if (o.labels) {
      for (let y = c.minY; y <= c.maxY; y += 1) {
        const yM = y * o.mPerPx;
        let nearest = Infinity;
        for (const rc of o.rowCentersM) {
          const d = Math.abs(yM - rc);
          if (d < nearest) nearest = d;
        }
        const isBand = nearest <= o.rowHalfWidthM;
        for (let x = c.minX; x <= c.maxX; x += 1) {
          const idx = y * o.width + x;
          if (o.labels[idx] !== c.id) continue;
          if (isBand) {
            inBandPx += 1;
          } else {
            outOfBandPx += 1;
            if (nearest - o.rowHalfWidthM > maxOutOffsetM) maxOutOffsetM = nearest - o.rowHalfWidthM;
          }
          if (o.exgMap) exgSum += o.exgMap[idx];
        }
      }
    }
    const meanExG = c.areaPx > 0 && o.exgMap ? exgSum / c.areaPx : null;
    const outOfBandRatio = c.areaPx > 0 ? outOfBandPx / c.areaPx : 0;
    const outOfBandAreaM2 = outOfBandPx * o.mPerPx * o.mPerPx;
    const inBandAreaM2 = inBandPx * o.mPerPx * o.mPerPx;
    let className = 'noise';
    let reason = 'area-too-small';
    if (areaM2 >= o.minWeedAreaM2) {
      if (outOfBandAreaM2 >= o.minOutOfBandAreaM2) {
        className = 'weed';
        reason = outOfBandRatio >= 0.5 ? 'off-row-dominant' : 'merged-with-crop-row';
      } else if (inBandAreaM2 >= o.minCropAreaM2) {
        className = 'crop';
        reason = 'in-row-band';
      } else {
        className = 'noise';
        reason = 'too-small-after-row-test';
      }
    }
    out.push({
      id: c.id,
      areaPx: c.areaPx,
      areaM2,
      centroidPx: c.centroidPx,
      centroidM: { x: cxM, y: cyM },
      bboxPx: { minX: c.minX, minY: c.minY, maxX: c.maxX, maxY: c.maxY },
      widthPx: c.widthPx,
      heightPx: c.heightPx,
      elongation: elong,
      compactness,
      rowOffsetM,
      nearestRowM: nearestRow,
      inBandPx,
      outOfBandPx,
      outOfBandRatio,
      outOfBandAreaM2,
      maxOutOffsetM,
      meanExG,
      className,
      reason,
    });
  }
  return out;
}

/* =========================================================================
 * 5. 杂草密度栅格与变量作业处方图
 * ========================================================================= */

/**
 * 杂草密度栅格成图。
 * @param {Array} weeds 杂草团块（classifyComponents 中 className==='weed'）
 * @param {object} opts {widthM, heightM, cellSizeM}
 */
export function weedDensityGrid(weeds, opts) {
  const o = Object.assign({ widthM: 12.8, heightM: 9.6, cellSizeM: 1.0 }, opts || {});
  const cols = Math.ceil(o.widthM / o.cellSizeM);
  const rows = Math.ceil(o.heightM / o.cellSizeM);
  const counts = new Int32Array(cols * rows);
  for (const wd of weeds) {
    const cx = Math.min(cols - 1, Math.max(0, Math.floor(wd.centroidM.x / o.cellSizeM)));
    const cy = Math.min(rows - 1, Math.max(0, Math.floor(wd.centroidM.y / o.cellSizeM)));
    counts[cy * cols + cx] += 1;
  }
  const cellArea = o.cellSizeM * o.cellSizeM;
  const density = Array.from(counts, (c) => c / cellArea);
  const zoneOf = (d) => {
    if (d <= 0) return '无';
    if (d < 0.5) return '轻';
    if (d < 1.5) return '中';
    return '重';
  };
  const zones = density.map(zoneOf);
  const stats = { cols, rows, cellSizeM: o.cellSizeM, cellAreaM2: cellArea, maxDensity: 0, meanDensity: 0 };
  let sum = 0;
  for (const d of density) {
    sum += d;
    if (d > stats.maxDensity) stats.maxDensity = d;
  }
  stats.meanDensity = density.length ? sum / density.length : 0;
  const zoneCount = { 无: 0, 轻: 0, 中: 0, 重: 0 };
  for (const z of zones) zoneCount[z] += 1;
  return { ...stats, counts: Array.from(counts), density, zones, zoneCount };
}

/**
 * 变量作业处方图：按杂草密度生成除草剂施用量(L/ha)。
 * 处方公式：rate = clamp(baseRate + gain * density, 0, maxRate)
 * 与传统均一喷施(全区按 maxRate 或 baseRate 常量)对比可给出药量节省比例。
 */
export function prescriptionMap(grid, opts) {
  const o = Object.assign({ baseRateLPerHa: 1.0, gainLPerHaPerWeed: 1.2, maxRateLPerHa: 4.0, uniformRateLPerHa: 3.0 }, opts || {});
  const rates = grid.density.map((d) => Math.min(o.maxRateLPerHa, Math.max(0, o.baseRateLPerHa + o.gainLPerHaPerWeed * d)));
  const cellHa = grid.cellAreaM2 / 10000;
  const variableTotal = rates.reduce((s, r) => s + r * cellHa, 0);
  const uniformTotal = o.uniformRateLPerHa * cellHa * rates.length;
  return {
    rates: rates.map((r) => Number(r.toFixed(2))),
    variableTotalL: Number(variableTotal.toFixed(4)),
    uniformTotalL: Number(uniformTotal.toFixed(4)),
    savingPct: uniformTotal > 0 ? Number((((uniformTotal - variableTotal) / uniformTotal) * 100).toFixed(2)) : 0,
    params: o,
  };
}

/* =========================================================================
 * 6. 端到端杂草识别
 * ========================================================================= */

/**
 * 完整识别流程 + 与真值对比评估。
 * @param {object} image makeSyntheticFieldImage 的输出（含像素级真值 label）
 */
export function runWeedDetection(image, opts) {
  const o = Object.assign(
    {
      // 'auto' 表示使用 Otsu 自动阈值；也可给定固定 ExG 阈值
      exgThreshold: 'auto',
      minExgThreshold: 0,
      openIterations: 1,
      closeIterations: 1,
      // null 表示按检测到的行距自适应（行距的 20%）
      rowHalfWidthM: null,
      expectedRowSpacingM: 0.6,
      rowSmoothWindowM: 0.09,
      minWeedAreaM2: 0.004,
      minOutOfBandAreaM2: 0.004,
      minCropAreaM2: 0.015,
      gridCellM: 1.0,
      evaluate: true,
    },
    opts || {},
  );
  const w = image.width;
  const h = image.height;
  const mPerPx = image.mPerPx;
  const exgMap = computeExgMap(image);
  const otsu = otsuThreshold(exgMap);
  const threshold = o.exgThreshold === 'auto' ? Math.max(o.minExgThreshold, otsu) : o.exgThreshold;
  const rawMask = segmentVegetation(exgMap, w, h, threshold);
  const opened = morphOpen(rawMask, w, h, o.openIterations);
  const mask = morphClose(opened, w, h, o.closeIterations);
  const cc = connectedComponents(mask, w, h, 8);
  // 作物行中心线由影像自身检测得到（真值仅用于评估行检测精度）
  const rows = estimateCropRows(opened, w, h, mPerPx, {
    expectedSpacingM: o.expectedRowSpacingM,
    smoothWindowM: o.rowSmoothWindowM,
  });
  const rowCentersM = rows.rowCentersM;
  // 行带半宽取行距的 20%（约等于作物冠层半宽），并限制在 [0.08, 0.15] m
  const rowHalfWidthM = o.rowHalfWidthM === null || o.rowHalfWidthM === undefined
    ? Math.min(0.15, Math.max(0.08, rows.spacingM * 0.22))
    : o.rowHalfWidthM;
  const blobs = classifyComponents(cc.components, {
    mPerPx,
    rowCentersM,
    rowHalfWidthM,
    minWeedAreaM2: o.minWeedAreaM2,
    minOutOfBandAreaM2: o.minOutOfBandAreaM2,
    minCropAreaM2: o.minCropAreaM2,
    exgMap,
    labels: cc.labels,
    width: w,
    height: h,
  });
  const weeds = blobs.filter((b) => b.className === 'weed');
  const crops = blobs.filter((b) => b.className === 'crop');
  const grid = weedDensityGrid(weeds, {
    widthM: image.truth.tileWidthM,
    heightM: image.truth.tileHeightM,
    cellSizeM: o.gridCellM,
  });
  const rx = prescriptionMap(grid);

  // 像素级杂草判定图（以连通域为决策单元，并对"与行带粘连"的团块做行带外像素细化）：
  //   - 团块行带外占比 >= 0.85：整块计为杂草目标（典型的行间杂草）；
  //   - 否则（杂草与作物行带粘连）：仅将其行带外像素计为杂草目标，
  //     行带内像素归属不确定，不计入评价（避免把作物冠层误记为杂草目标）。
  const weedMask = new Uint8Array(w * h);
  const outOfBandMask = new Uint8Array(w * h);
  let mergedBlobCount = 0;
  for (const b of weeds) {
    const useWholeBlob = b.outOfBandRatio >= 0.85;
    if (!useWholeBlob) mergedBlobCount += 1;
    for (let y = b.bboxPx.minY; y <= b.bboxPx.maxY; y += 1) {
      const yM = y * mPerPx;
      let nearest = Infinity;
      for (const rc of rowCentersM) {
        const d = Math.abs(yM - rc);
        if (d < nearest) nearest = d;
      }
      const isBand = nearest <= rowHalfWidthM;
      for (let x = b.bboxPx.minX; x <= b.bboxPx.maxX; x += 1) {
        const idx = y * w + x;
        if (cc.labels[idx] !== b.id) continue;
        if (useWholeBlob || !isBand) weedMask[idx] = 1;
        if (!isBand) outOfBandMask[idx] = 1;
      }
    }
  }
  let weedPixelCount = 0;
  let outOfBandPixelCount = 0;
  for (let i = 0; i < weedMask.length; i += 1) {
    if (weedMask[i]) weedPixelCount += 1;
    if (outOfBandMask[i]) outOfBandPixelCount += 1;
  }

  let evaluation = null;
  if (o.evaluate && image.label) {
    // 像素级二分类评估（杂草为正类）：TP=判为杂草且真值为杂草；FP=判为杂草但真值为作物
    // FN=真值为杂草但未被判为杂草（含行内杂草与作物行带粘连的像素）
    let tp = 0;
    let fp = 0;
    let fn = 0;
    for (let i = 0; i < weedMask.length; i += 1) {
      const gt = image.label[i];
      if (weedMask[i]) {
        if (gt === 2) tp += 1;
        else if (gt === 1) fp += 1;
      } else if (gt === 2) {
        fn += 1;
      }
    }
    const precision = tp + fp > 0 ? tp / (tp + fp) : 0;
    const recall = tp + fn > 0 ? tp / (tp + fn) : 0;
    const f1 = precision + recall > 0 ? (2 * precision * recall) / (precision + recall) : 0;
    // 按"行内杂草 / 行间杂草"分组统计召回率：行内杂草与作物行带粘连，
    // 单目可见光条件下本质不可分，需多光谱或深度学习进一步区分。
    const gtWeeds = (image.truth && image.truth.weeds) || [];
    const group = { inRow: { det: 0, tot: 0 }, interRow: { det: 0, tot: 0 }, detPlantInRow: 0, detPlantInterRow: 0 };
    for (let i = 0; i < image.label.length; i += 1) {
      if (image.label[i] !== 2) continue;
      const x = i % w;
      const y = (i - x) / w;
      let best = -1;
      let bestD = Infinity;
      for (let k = 0; k < gtWeeds.length; k += 1) {
        const g = gtWeeds[k];
        const d = (g.x - x * mPerPx) * (g.x - x * mPerPx) + (g.y - y * mPerPx) * (g.y - y * mPerPx);
        if (d < bestD) {
          bestD = d;
          best = k;
        }
      }
      if (best < 0) continue;
      const g = gtWeeds[best];
      const bucket = g.inRow ? group.inRow : group.interRow;
      bucket.tot += 1;
      if (weedMask[i]) bucket.det += 1;
    }
    const recallOf = (b) => (b.tot > 0 ? Number((b.det / b.tot).toFixed(4)) : null);
    // 植株级检出：某株杂草的真值像素被检出 30% 以上即视为该株被检出
    const plantHit = new Array(gtWeeds.length).fill(0);
    const plantTot = new Array(gtWeeds.length).fill(0);
    for (let i = 0; i < image.label.length; i += 1) {
      if (image.label[i] !== 2) continue;
      const x = i % w;
      const y = (i - x) / w;
      let best = -1;
      let bestD = Infinity;
      for (let k = 0; k < gtWeeds.length; k += 1) {
        const g = gtWeeds[k];
        const d = (g.x - x * mPerPx) * (g.x - x * mPerPx) + (g.y - y * mPerPx) * (g.y - y * mPerPx);
        if (d < bestD) {
          bestD = d;
          best = k;
        }
      }
      if (best < 0) continue;
      plantTot[best] += 1;
      if (weedMask[i]) plantHit[best] += 1;
    }
    for (let k = 0; k < gtWeeds.length; k += 1) {
      if (plantTot[k] === 0) continue;
      if (plantHit[k] / plantTot[k] >= 0.3) {
        if (gtWeeds[k].inRow) group.detPlantInRow += 1;
        else group.detPlantInterRow += 1;
      }
    }
    let segTp = 0;
    let segFp = 0;
    let segFn = 0;
    for (let i = 0; i < mask.length; i += 1) {
      const gtVeg = image.label[i] > 0;
      if (mask[i] && gtVeg) segTp += 1;
      else if (mask[i] && !gtVeg) segFp += 1;
      else if (!mask[i] && gtVeg) segFn += 1;
    }
    evaluation = {
      weedPrecision: Number(precision.toFixed(4)),
      weedRecall: Number(recall.toFixed(4)),
      weedF1: Number(f1.toFixed(4)),
      truePosPx: tp,
      falsePosPx: fp,
      falseNegPx: fn,
      recallInRowWeed: recallOf(group.inRow),
      recallInterRowWeed: recallOf(group.interRow),
      plantLevel: {
        gtInRow: gtWeeds.filter((g) => g.inRow).length,
        gtInterRow: gtWeeds.filter((g) => !g.inRow).length,
        detectedInRow: group.detPlantInRow,
        detectedInterRow: group.detPlantInterRow,
        plantRecall: gtWeeds.length ? Number(((group.detPlantInRow + group.detPlantInterRow) / gtWeeds.length).toFixed(4)) : null,
      },
      segmentation: {
        iou: Number((segTp / (segTp + segFp + segFn)).toFixed(4)),
        precision: Number((segTp / Math.max(1, segTp + segFp)).toFixed(4)),
        recall: Number((segTp / Math.max(1, segTp + segFn)).toFixed(4)),
      },
    };
  }

  return {
    imageSize: { width: w, height: h, mPerPx },
    exgThreshold: threshold,
    otsuThreshold: otsu,
    rowDetection: {
      rowCentersM: rowCentersM.map((v) => Number(v.toFixed(4))),
      detectedRows: rowCentersM.length,
      spacingM: Number(rows.spacingM.toFixed(4)),
      rowHalfWidthM: Number(rowHalfWidthM.toFixed(4)),
      // 与真值行中心线的平均绝对偏差（评估行检测精度）
      meanAbsErrorM: (image.truth.rowCentersPx || []).length
        ? Number(
            (
              rowCentersM.reduce((s, rc) => {
                let best = Infinity;
                for (const t of image.truth.rowCentersPx) best = Math.min(best, Math.abs(rc - t * mPerPx));
                return s + best;
              }, 0) / Math.max(1, rowCentersM.length)
            ).toFixed(4),
          )
        : null,
    },
    vegetationPixelRatio: mask.reduce((s, v) => s + v, 0) / mask.length,
    noiseRemovedPx: rawMask.reduce((s, v) => s + v, 0) - opened.reduce((s, v) => s + v, 0),
    componentCount: cc.count,
    blobCount: blobs.length,
    weedPixelCount,
    outOfBandPixelCount,
    mergedWeedBlobCount: mergedBlobCount,
    weedMask,
    outOfBandMask,
    labels: cc.labels,
    blobCount: blobs.length,
    weedCount: weeds.length,
    cropBlobCount: crops.length,
    discardedNoiseBlobs: blobs.filter((b) => b.className === 'noise').length,
    weeds,
    crops,
    grid,
    prescription: rx,
    evaluation,
    truth: image.truth
      ? {
          weedCount: image.truth.weedCount,
          weedDensityPerM2: Number(image.truth.weedDensityPerM2.toFixed(3)),
          weedPixelCount: image.truth.weedPixelCount,
        }
      : null,
  };
}
