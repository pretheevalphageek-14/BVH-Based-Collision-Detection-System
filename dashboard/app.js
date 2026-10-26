// BVH Dynamics Web Simulator & Analytics Companion
(function () {
  'use strict';

  // --- Tab Navigation ---
  const tabs = document.querySelectorAll('.nav-tab');
  const panels = document.querySelectorAll('.tab-content');

  tabs.forEach(tab => {
    tab.addEventListener('click', () => {
      const target = tab.dataset.tab;
      tabs.forEach(t => t.classList.remove('active'));
      panels.forEach(p => p.classList.remove('active'));
      tab.classList.add('active');
      const activePanel = document.getElementById(target);
      if (activePanel) activePanel.classList.add('active');
    });
  });

  // --- Simulation Logic ---
  const canvas = document.getElementById('bvh-canvas');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');

  const sparkCanvas = document.getElementById('telemetry-sparkline');
  const sparkCtx = sparkCanvas ? sparkCanvas.getContext('2d') : null;

  // Set crisp canvas dimensions
  function resizeCanvas() {
    const rect = canvas.getBoundingClientRect();
    canvas.width = rect.width * window.devicePixelRatio;
    canvas.height = rect.height * window.devicePixelRatio;
    ctx.scale(window.devicePixelRatio, window.devicePixelRatio);

    if (sparkCanvas) {
      const sRect = sparkCanvas.getBoundingClientRect();
      sparkCanvas.width = sRect.width * window.devicePixelRatio;
      sparkCanvas.height = sRect.height * window.devicePixelRatio;
      sparkCtx.scale(window.devicePixelRatio, window.devicePixelRatio);
    }
  }
  window.addEventListener('resize', resizeCanvas);

  // Simulation Entities & State
  const DT = 1.0 / 60.0;
  let circles = [];
  let paused = false;
  let stepOnce = false;
  let mode = 'median'; // 'median' | 'sah' | 'refit'
  let restitution = 1.0;
  let frictionEnabled = true;
  let gravityEnabled = false;
  let showTree = true;
  let maxTreeDepth = 5;
  let showVelocity = false;

  let mouseX = 0, mouseY = 0;
  let isRightDown = false;
  let isLeftDown = false;
  let draggedBodies = [];

  // Rolling Performance Metrics
  let lastTime = performance.now();
  let frameCount = 0;
  let fps = 60;
  let fpsTimer = 0;
  let lastBuildUs = 45;
  let lastQueryUs = 18;
  let pairsTested = 0;
  let totalPairsPossible = 0;
  let culledPct = 99.2;
  let bvhNodesCount = 0;
  let totalKineticEnergy = 0;
  let perfHistory = new Array(60).fill(60);

  function makeCircle(x, y) {
    const spd = 140;
    const r = 8 + Math.random() * 10;
    const hue = Math.floor(Math.random() * 360);
    return {
      x, y,
      vx: (Math.random() * 2 - 1) * spd,
      vy: (Math.random() * 2 - 1) * spd,
      r,
      color: `hsl(${hue}, 85%, 62%)`,
      mass: r * 0.2,
      flash: 0
    };
  }

  function spawnCircles(count) {
    const rect = canvas.getBoundingClientRect();
    const w = rect.width;
    const h = rect.height;
    for (let i = 0; i < count; i++) {
      circles.push(makeCircle(40 + Math.random() * (w - 80), 40 + Math.random() * (h - 80)));
    }
  }

  // --- BVH Data Structure (2D) ---
  class AABB {
    constructor(minX, minY, maxX, maxY) {
      this.minX = minX; this.minY = minY;
      this.maxX = maxX; this.maxY = maxY;
    }
    overlaps(other) {
      return this.minX <= other.maxX && this.maxX >= other.minX &&
             this.minY <= other.maxY && this.maxY >= other.minY;
    }
  }

  class BVHNode {
    constructor() {
      this.bounds = null;
      this.left = -1;
      this.right = -1;
      this.objIndex = -1;
    }
  }

  let bvhNodes = [];
  let bvhIndices = [];

  function computeAABB(c, margin = 0) {
    return new AABB(c.x - c.r - margin, c.y - c.r - margin, c.x + c.r + margin, c.y + c.r + margin);
  }

  function combineAABBs(a, b) {
    return new AABB(
      Math.min(a.minX, b.minX), Math.min(a.minY, b.minY),
      Math.max(a.maxX, b.maxX), Math.max(a.maxY, b.maxY)
    );
  }

  function buildBVH() {
    const t0 = performance.now();
    bvhNodes = [];
    bvhIndices = circles.map((_, i) => i);
    if (circles.length === 0) {
      lastBuildUs = 0;
      return;
    }

    function buildRecursive(start, end) {
      const nodeIdx = bvhNodes.length;
      bvhNodes.push(new BVHNode());

      // Compute bounding box
      let nodeBox = computeAABB(circles[bvhIndices[start]]);
      for (let i = start + 1; i < end; i++) {
        nodeBox = combineAABBs(nodeBox, computeAABB(circles[bvhIndices[i]]));
      }
      bvhNodes[nodeIdx].bounds = nodeBox;

      const count = end - start;
      if (count === 1) {
        bvhNodes[nodeIdx].objIndex = bvhIndices[start];
        return nodeIdx;
      }

      // Partition along longest axis
      const extentX = nodeBox.maxX - nodeBox.minX;
      const extentY = nodeBox.maxY - nodeBox.minY;
      const axis = extentX > extentY ? 'x' : 'y';

      let mid = start + Math.floor(count / 2);
      bvhIndices.slice(start, end).sort((a, b) => circles[a][axis] - circles[b][axis]);

      bvhNodes[nodeIdx].left = buildRecursive(start, mid);
      bvhNodes[nodeIdx].right = buildRecursive(mid, end);
      return nodeIdx;
    }

    buildRecursive(0, bvhIndices.length);
    bvhNodesCount = bvhNodes.length;
    lastBuildUs = (performance.now() - t0) * 1000;
  }

  function queryBVH() {
    const t0 = performance.now();
    pairsTested = 0;
    const pairs = [];
    if (bvhNodes.length === 0) {
      lastQueryUs = 0;
      return pairs;
    }

    const stack = [[0, 0]];
    while (stack.length > 0) {
      const [aIdx, bIdx] = stack.pop();
      if (!bvhNodes[aIdx].bounds.overlaps(bvhNodes[bIdx].bounds)) continue;

      const aLeaf = bvhNodes[aIdx].objIndex >= 0;
      const bLeaf = bvhNodes[bIdx].objIndex >= 0;

      if (aLeaf && bLeaf) {
        if (aIdx !== bIdx) {
          const o1 = bvhNodes[aIdx].objIndex;
          const o2 = bvhNodes[bIdx].objIndex;
          if (o1 < o2) {
            pairsTested++;
            pairs.push([o1, o2]);
          }
        }
        continue;
      }

      if (aIdx === bIdx) {
        stack.push([bvhNodes[aIdx].left, bvhNodes[aIdx].left]);
        stack.push([bvhNodes[aIdx].right, bvhNodes[aIdx].right]);
        stack.push([bvhNodes[aIdx].left, bvhNodes[aIdx].right]);
      } else if (aLeaf) {
        stack.push([aIdx, bvhNodes[bIdx].left]);
        stack.push([aIdx, bvhNodes[bIdx].right]);
      } else if (bLeaf) {
        stack.push([bvhNodes[aIdx].left, bIdx]);
        stack.push([bvhNodes[aIdx].right, bIdx]);
      } else {
        stack.push([bvhNodes[aIdx].left, bvhNodes[bIdx].left]);
        stack.push([bvhNodes[aIdx].left, bvhNodes[bIdx].right]);
        stack.push([bvhNodes[aIdx].right, bvhNodes[bIdx].left]);
        stack.push([bvhNodes[aIdx].right, bvhNodes[bIdx].right]);
      }
    }

    lastQueryUs = (performance.now() - t0) * 1000;
    return pairs;
  }

  // --- Physics Resolution ---
  function resolveCollisions(pairs) {
    let hits = 0;
    for (const [i1, i2] of pairs) {
      const c1 = circles[i1];
      const c2 = circles[i2];
      const dx = c2.x - c1.x;
      const dy = c2.y - c1.y;
      const dist2 = dx * dx + dy * dy;
      const radSum = c1.r + c2.r;

      if (dist2 < radSum * radSum && dist2 > 0.0001) {
        hits++;
        c1.flash = 3;
        c2.flash = 3;

        const dist = Math.sqrt(dist2);
        const nx = dx / dist;
        const ny = dy / dist;

        // Relative velocity
        const kx = c1.vx - c2.vx;
        const ky = c1.vy - c2.vy;
        const velAlongNormal = kx * nx + ky * ny;

        if (velAlongNormal > 0) {
          const invM1 = 1.0 / c1.mass;
          const invM2 = 1.0 / c2.mass;
          const impulse = -(1 + restitution) * velAlongNormal / (invM1 + invM2);

          c1.vx += impulse * invM1 * nx;
          c1.vy += impulse * invM1 * ny;
          c2.vx -= impulse * invM2 * nx;
          c2.vy -= impulse * invM2 * ny;

          // Positional de-penetration
          const percent = 0.8;
          const slop = 0.01;
          const penetration = Math.max(0, radSum - dist - slop);
          const correction = (penetration / (invM1 + invM2)) * percent;
          c1.x -= correction * invM1 * nx;
          c1.y -= correction * invM1 * ny;
          c2.x += correction * invM2 * nx;
          c2.y += correction * invM2 * ny;
        }
      }
    }
    return hits;
  }

  function stepSimulation(w, h) {
    totalKineticEnergy = 0;

    // Apply tractor beam from right-click gravity drag
    draggedBodies = [];
    if (isRightDown) {
      for (let i = 0; i < circles.length; i++) {
        const c = circles[i];
        const dx = mouseX - c.x;
        const dy = mouseY - c.y;
        const dist2 = dx * dx + dy * dy;
        if (dist2 < 200 * 200 && dist2 > 1) {
          c.vx += dx * 0.6 * DT;
          c.vy += dy * 0.6 * DT;
          draggedBodies.push(i);
        }
      }
    }

    // Kinematics integration & boundary walls
    for (const c of circles) {
      if (gravityEnabled) c.vy += 300.0 * DT;
      if (frictionEnabled) {
        c.vx *= 0.999;
        c.vy *= 0.999;
      }

      c.x += c.vx * DT;
      c.y += c.vy * DT;

      // Wall rebounds
      if (c.x - c.r < 0) { c.x = c.r; c.vx = -c.vx * 0.9; }
      if (c.x + c.r > w) { c.x = w - c.r; c.vx = -c.vx * 0.9; }
      if (c.y - c.r < 0) { c.y = c.r; c.vy = -c.vy * 0.9; }
      if (c.y + c.r > h) { c.y = h - c.r; c.vy = -c.vy * 0.9; }

      if (c.flash > 0) c.flash--;
      totalKineticEnergy += 0.5 * c.mass * (c.vx * c.vx + c.vy * c.vy);
    }

    // Broad-phase build & query
    buildBVH();
    const candidatePairs = queryBVH();
    const hitCount = resolveCollisions(candidatePairs);

    // Update culling efficiency
    totalPairsPossible = (circles.length * (circles.length - 1)) / 2;
    culledPct = totalPairsPossible > 0 ? (1.0 - pairsTested / totalPairsPossible) * 100 : 0;

    return hitCount;
  }

  // --- Rendering Pipeline ---
  const DEPTH_PALETTE = [
    '#00e5ff', '#10b981', '#f59e0b', '#ec4899', '#8b5cf6', '#38bdf8'
  ];

  function renderTreeOverlay(nodeIdx, depth) {
    if (nodeIdx < 0 || nodeIdx >= bvhNodes.length || depth > maxTreeDepth) return;
    const node = bvhNodes[nodeIdx];
    const b = node.bounds;
    const color = DEPTH_PALETTE[depth % DEPTH_PALETTE.length];

    ctx.strokeStyle = color;
    ctx.lineWidth = 1;
    ctx.strokeRect(b.minX, b.minY, b.maxX - b.minX, b.maxY - b.minY);

    if (node.objIndex < 0) {
      renderTreeOverlay(node.left, depth + 1);
      renderTreeOverlay(node.right, depth + 1);
    }
  }

  function render(hits) {
    const rect = canvas.getBoundingClientRect();
    const w = rect.width;
    const h = rect.height;

    // Clear background
    ctx.fillStyle = '#080c16';
    ctx.fillRect(0, 0, w, h);

    // Matrix background grid
    ctx.strokeStyle = 'rgba(56, 189, 248, 0.05)';
    ctx.lineWidth = 1;
    for (let x = 0; x < w; x += 48) {
      ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, h); ctx.stroke();
    }
    for (let y = 0; y < h; y += 48) {
      ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke();
    }

    // Render tree overlay if enabled
    if (showTree && bvhNodes.length > 0) {
      ctx.globalAlpha = 0.7;
      renderTreeOverlay(0, 0);
      ctx.globalAlpha = 1.0;
    }

    // Render rigid bodies
    for (const c of circles) {
      ctx.beginPath();
      ctx.arc(c.x, c.y, c.r, 0, Math.PI * 2);
      ctx.fillStyle = c.color;
      ctx.fill();

      // 3D Specular reflection glint
      if (c.r >= 6) {
        ctx.beginPath();
        ctx.arc(c.x - c.r * 0.32, c.y - c.r * 0.32, c.r * 0.35, 0, Math.PI * 2);
        ctx.fillStyle = 'rgba(255, 255, 255, 0.45)';
        ctx.fill();
      }

      // Velocity vectors
      if (showVelocity) {
        ctx.beginPath();
        ctx.moveTo(c.x, c.y);
        ctx.lineTo(c.x + c.vx * 0.08, c.y + c.vy * 0.08);
        ctx.strokeStyle = 'rgba(56, 189, 248, 0.6)';
        ctx.stroke();
      }

      // Collision shockwave halo
      if (c.flash > 0) {
        ctx.beginPath();
        ctx.arc(c.x, c.y, c.r + 3, 0, Math.PI * 2);
        ctx.strokeStyle = '#f43f5e';
        ctx.lineWidth = 2;
        ctx.stroke();
      }
    }

    // Tractor beam visualizer
    if (isRightDown) {
      // Concentric rings
      const time = performance.now() * 0.002;
      for (let r = 0; r < 4; r++) {
        const rad = ((time * 30 + r * 50) % 200);
        const alpha = 1.0 - rad / 200;
        ctx.beginPath();
        ctx.arc(mouseX, mouseY, rad, 0, Math.PI * 2);
        ctx.strokeStyle = `rgba(168, 85, 247, ${alpha * 0.6})`;
        ctx.lineWidth = 1.5;
        ctx.stroke();
      }

      // Force beams
      for (const idx of draggedBodies) {
        const c = circles[idx];
        ctx.beginPath();
        ctx.moveTo(mouseX, mouseY);
        ctx.lineTo(c.x, c.y);
        ctx.strokeStyle = 'rgba(192, 132, 252, 0.35)';
        ctx.lineWidth = 1;
        ctx.stroke();
      }

      // Cursor reticle
      ctx.beginPath();
      ctx.moveTo(mouseX - 8, mouseY); ctx.lineTo(mouseX + 8, mouseY);
      ctx.moveTo(mouseX, mouseY - 8); ctx.lineTo(mouseX, mouseY + 8);
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = 1.5;
      ctx.stroke();
    }

    // Render sparkline
    if (sparkCtx) {
      const sRect = sparkCanvas.getBoundingClientRect();
      const sw = sRect.width;
      const sh = sRect.height;
      sparkCtx.clearRect(0, 0, sw, sh);

      const maxVal = Math.max(100, ...perfHistory);
      sparkCtx.beginPath();
      for (let i = 0; i < perfHistory.length; i++) {
        const px = (i / (perfHistory.length - 1)) * sw;
        const py = sh - (perfHistory[i] / maxVal) * (sh - 8) - 4;
        if (i === 0) sparkCtx.moveTo(px, py);
        else sparkCtx.lineTo(px, py);
      }
      sparkCtx.strokeStyle = '#00e5ff';
      sparkCtx.lineWidth = 1.5;
      sparkCtx.stroke();
    }
  }

  // --- Main Animation Loop ---
  function loop(timestamp) {
    const delta = (timestamp - lastTime) / 1000;
    lastTime = timestamp;
    fpsTimer += delta;
    frameCount++;
    if (fpsTimer >= 0.5) {
      fps = Math.round(frameCount / fpsTimer);
      frameCount = 0;
      fpsTimer = 0;
    }

    const rect = canvas.getBoundingClientRect();
    let hits = 0;
    if (!paused || stepOnce) {
      hits = stepSimulation(rect.width, rect.height);
      stepOnce = false;

      // Update rolling telemetry history
      perfHistory.push(lastBuildUs + lastQueryUs);
      if (perfHistory.length > 60) perfHistory.shift();
    }

    render(hits);
    updateUIElements(hits);

    requestAnimationFrame(loop);
  }

  // Update DOM HUD elements
  function updateUIElements(hits) {
    const elFps = document.getElementById('metric-fps');
    if (elFps) elFps.textContent = `${fps} FPS`;

    const elBodies = document.getElementById('metric-bodies');
    if (elBodies) elBodies.textContent = `${circles.length}`;

    const elHits = document.getElementById('metric-hits');
    if (elHits) elHits.textContent = `${hits}`;

    const elBuild = document.getElementById('metric-build');
    if (elBuild) elBuild.textContent = `${lastBuildUs.toFixed(1)} µs`;

    const elQuery = document.getElementById('metric-query');
    if (elQuery) elQuery.textContent = `${lastQueryUs.toFixed(1)} µs`;

    const elCulled = document.getElementById('metric-cull-pct');
    if (elCulled) elCulled.textContent = `${culledPct.toFixed(2)}%`;

    const elCulledBar = document.getElementById('cull-bar-fill');
    if (elCulledBar) elCulledBar.style.width = `${Math.min(100, Math.max(0, culledPct))}%`;

    const elTested = document.getElementById('metric-pairs-tested');
    if (elTested) elTested.textContent = `${pairsTested.toLocaleString()}`;

    const elPossible = document.getElementById('metric-pairs-possible');
    if (elPossible) elPossible.textContent = `${totalPairsPossible.toLocaleString()}`;

    const elNodes = document.getElementById('metric-nodes');
    if (elNodes) elNodes.textContent = `${bvhNodesCount}`;

    const elKe = document.getElementById('metric-ke');
    if (elKe) elKe.textContent = `${Math.round(totalKineticEnergy).toLocaleString()} J`;

    const elDepth = document.getElementById('metric-depth');
    if (elDepth) elDepth.textContent = `${maxTreeDepth}`;
  }

  // --- Mouse & UI Controls ---
  canvas.addEventListener('mousemove', e => {
    const r = canvas.getBoundingClientRect();
    mouseX = e.clientX - r.left;
    mouseY = e.clientY - r.top;
  });

  canvas.addEventListener('mousedown', e => {
    if (e.button === 0) {
      isLeftDown = true;
      circles.push(makeCircle(mouseX, mouseY));
    } else if (e.button === 2) {
      isRightDown = true;
    }
  });

  window.addEventListener('mouseup', e => {
    if (e.button === 0) isLeftDown = false;
    if (e.button === 2) isRightDown = false;
  });

  canvas.addEventListener('contextmenu', e => e.preventDefault());

  // Interactive Buttons
  document.querySelectorAll('[data-mode]').forEach(btn => {
    btn.addEventListener('click', () => {
      document.querySelectorAll('[data-mode]').forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      mode = btn.dataset.mode;
    });
  });

  document.querySelectorAll('[data-restitution]').forEach(btn => {
    btn.addEventListener('click', () => {
      document.querySelectorAll('[data-restitution]').forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      restitution = parseFloat(btn.dataset.restitution);
    });
  });

  const btnPause = document.getElementById('btn-pause');
  if (btnPause) {
    btnPause.addEventListener('click', () => {
      paused = !paused;
      btnPause.textContent = paused ? '▶ RESUME' : '⏸ PAUSE';
      btnPause.classList.toggle('active-amber', paused);
    });
  }

  const btnStep = document.getElementById('btn-step');
  if (btnStep) {
    btnStep.addEventListener('click', () => {
      if (paused) stepOnce = true;
    });
  }

  const btnReset = document.getElementById('btn-reset');
  if (btnReset) {
    btnReset.addEventListener('click', () => {
      circles = [];
      spawnCircles(350);
    });
  }

  const btnClear = document.getElementById('btn-clear');
  if (btnClear) {
    btnClear.addEventListener('click', () => {
      circles = [];
    });
  }

  const btnAdd = document.getElementById('btn-add-50');
  if (btnAdd) {
    btnAdd.addEventListener('click', () => spawnCircles(50));
  }

  const btnSub = document.getElementById('btn-sub-50');
  if (btnSub) {
    btnSub.addEventListener('click', () => {
      if (circles.length >= 50) circles.splice(-50);
      else circles = [];
    });
  }

  const btnTree = document.getElementById('btn-toggle-tree');
  if (btnTree) {
    btnTree.addEventListener('click', () => {
      showTree = !showTree;
      btnTree.classList.toggle('active', showTree);
    });
  }

  const btnDepthInc = document.getElementById('btn-depth-inc');
  if (btnDepthInc) {
    btnDepthInc.addEventListener('click', () => {
      maxTreeDepth = Math.min(12, maxTreeDepth + 1);
    });
  }

  const btnDepthDec = document.getElementById('btn-depth-dec');
  if (btnDepthDec) {
    btnDepthDec.addEventListener('click', () => {
      maxTreeDepth = Math.max(1, maxTreeDepth - 1);
    });
  }

  const btnFriction = document.getElementById('btn-toggle-friction');
  if (btnFriction) {
    btnFriction.addEventListener('click', () => {
      frictionEnabled = !frictionEnabled;
      btnFriction.classList.toggle('active', frictionEnabled);
    });
  }

  const btnGravity = document.getElementById('btn-toggle-gravity');
  if (btnGravity) {
    btnGravity.addEventListener('click', () => {
      gravityEnabled = !gravityEnabled;
      btnGravity.classList.toggle('active', gravityEnabled);
    });
  }

  const btnVel = document.getElementById('btn-toggle-velocity');
  if (btnVel) {
    btnVel.addEventListener('click', () => {
      showVelocity = !showVelocity;
      btnVel.classList.toggle('active', showVelocity);
    });
  }

  // Initialize
  resizeCanvas();
  spawnCircles(350);
  requestAnimationFrame(loop);
})();
