(function () {
  "use strict";

  const PC_RANGE = 153.6;
  const PC_FULL = PC_RANGE * 2;
  const GT_COLOR = "#4cc9f0";
  const PRED_COLOR = "#f9844a";
  const GAUSS_COLORS = ["#4361ee","#90be6d","#845ef7","#f9c74f","#ff6b6b","#20c997"];

  // KGGI keypoint sources (index matches `sources` field: 0=radar,1=camera,2=random,3=past)
  const SOURCE_INFO = [
    { name: "Radar",  color: "#ff6b6b" },
    { name: "Camera", color: "#4dabf7" },
    { name: "Random", color: "#ced4da" },
    { name: "Past",   color: "#b197fc" },
  ];

  const PAST_COLOR = "#7048e8";      // propagated (past) primitive — deep violet
  const TRACK_COLORS = ["#845ef7", "#20c997", "#f783ac"];  // per tracked dynamic object
  const KP_COLOR = "#ffd166";        // single keypoint / Gaussian color (OCSF)
  const ENLARGE_M = 4.8;             // BEV-mask box enlargement (config: enlarged=4.8)
  const ENLARGE_COLOR = "#ff4d4d";   // enlarged GT box (red dashed)
  function enlargedBox(b) { return [b[0], b[1], b[2] + ENLARGE_M, b[3] + ENLARGE_M, b[4]]; }

  function pointInBox(lx, ly, b, pad) {
    pad = pad || 1.0;
    const dx = lx - b[0], dy = ly - b[1];
    const c = Math.cos(-b[4]), s = Math.sin(-b[4]);
    const px = dx * c - dy * s, py = dx * s + dy * c;
    return Math.abs(px) <= b[2] / 2 + pad && Math.abs(py) <= b[3] / 2 + pad;
  }
  // Draw minority sources last so they aren't buried under dense radar points
  const SRC_DRAW_ORDER = [0, 2, 3, 1];
  function srcRank(s) { return SRC_DRAW_ORDER.indexOf(s); }

  let realOCSF = null;
  let realDPTF = null;

  function lidar2screen(lx, ly, W) {
    return { x: (ly + PC_RANGE) / PC_FULL * W, y: (lx + PC_RANGE) / PC_FULL * W };
  }

  function meters2px(m, W) { return m / PC_FULL * W; }

  // lidar2screen swaps axes (forward = up), so a primitive whose heading is θ in
  // the lidar frame appears at screen angle (π/2 − θ). Use this for ellipses.
  function screenAngle(cosR, sinR) { return Math.PI/2 - Math.atan2(sinR, cosR); }

  function drawArrow(ctx, x0, y0, x1, y1, color, lw, W) {
    const angle = Math.atan2(y1-y0, x1-x0);
    const headLen = 7 * (W / 800);
    ctx.beginPath(); ctx.moveTo(x0,y0); ctx.lineTo(x1,y1);
    ctx.strokeStyle = color; ctx.lineWidth = lw; ctx.stroke();
    ctx.beginPath(); ctx.moveTo(x1,y1);
    ctx.lineTo(x1-headLen*Math.cos(angle-0.4), y1-headLen*Math.sin(angle-0.4));
    ctx.lineTo(x1-headLen*Math.cos(angle+0.4), y1-headLen*Math.sin(angle+0.4));
    ctx.closePath(); ctx.fillStyle = color; ctx.fill();
  }

  function drawBevBg(ctx, W) {
    ctx.fillStyle = "#0f0f23"; ctx.fillRect(0,0,W,W);
    const ego = lidar2screen(0, 0, W);

    const laneW = 3.5, roadHL = laneW * 3;
    const p1 = lidar2screen(-PC_RANGE, -roadHL, W);
    const p2 = lidar2screen(PC_RANGE, roadHL, W);
    ctx.fillStyle = "rgba(40,40,55,0.5)";
    ctx.fillRect(p1.x, p1.y, p2.x-p1.x, p2.y-p1.y);

    ctx.strokeStyle = "rgba(255,200,50,0.3)"; ctx.lineWidth = 2; ctx.setLineDash([]);
    ctx.beginPath(); ctx.moveTo(ego.x, 0); ctx.lineTo(ego.x, W); ctx.stroke();

    ctx.strokeStyle = "rgba(255,255,255,0.08)"; ctx.lineWidth = 1;
    [50, 100, 150].forEach(r => {
      const rpx = meters2px(r, W);
      ctx.beginPath(); ctx.arc(ego.x, ego.y, rpx, 0, Math.PI*2); ctx.stroke();
      ctx.fillStyle = "rgba(255,255,255,0.15)";
      ctx.font = `${10*(W/800)}px sans-serif`;
      ctx.fillText(`${r}m`, ego.x + rpx + 3, ego.y - 3);
    });
    ctx.setLineDash([]);

    const egoL = meters2px(8, W), egoHW = meters2px(3.5, W) / 2;
    ctx.fillStyle = "rgba(200,200,230,0.7)";
    ctx.fillRect(ego.x - egoHW, ego.y - egoL/2, egoHW*2, egoL);
    ctx.strokeStyle = "#fff"; ctx.lineWidth = 1.5;
    ctx.strokeRect(ego.x - egoHW, ego.y - egoL/2, egoHW*2, egoL);
    ctx.fillStyle = "rgba(255,255,255,0.5)"; ctx.font = `${9*(W/800)}px sans-serif`;
    ctx.textAlign = "center"; ctx.fillText("EGO", ego.x, ego.y + 3); ctx.textAlign = "start";

    ctx.strokeStyle = "#fff"; ctx.lineWidth = 1.5;
    const arrowTop = ego.y - egoL/2 - meters2px(5, W);
    drawArrow(ctx, ego.x, ego.y - egoL/2 - 2, ego.x, arrowTop, "rgba(255,255,255,0.6)", 1.5, W);
    ctx.fillStyle = "rgba(255,255,255,0.4)"; ctx.font = `${9*(W/800)}px sans-serif`;
    ctx.textAlign = "center"; ctx.fillText("FWD", ego.x, arrowTop - 4); ctx.textAlign = "start";
  }

  function drawBoxOnBev(ctx, W, box, color, opts) {
    const cx = box[0], cy = box[1], l = box[2], w = box[3], yaw = box[4];
    const cosY = Math.cos(yaw), sinY = Math.sin(yaw);
    const hl = l/2, hw = w/2;
    const corners = [
      [-hl,-hw], [hl,-hw], [hl,hw], [-hl,hw]
    ].map(([dx,dy]) => lidar2screen(cx + cosY*dx - sinY*dy, cy + sinY*dx + cosY*dy, W));

    ctx.beginPath();
    corners.forEach((p,i) => i===0 ? ctx.moveTo(p.x,p.y) : ctx.lineTo(p.x,p.y));
    ctx.closePath();

    if (opts.dashed) {
      ctx.setLineDash([6,4]); ctx.strokeStyle = color; ctx.lineWidth = 1; ctx.stroke();
      ctx.setLineDash([]);
    } else {
      ctx.strokeStyle = color; ctx.lineWidth = 1.5; ctx.stroke();
      if (opts.fillAlpha > 0) {
        ctx.fillStyle = color.replace(")", `,${opts.fillAlpha})`).replace("rgb", "rgba");
        ctx.fill();
      }
    }
  }

  function drawVelocityArrow(ctx, W, cx, cy, vx, vy, color) {
    const speed = Math.sqrt(vx*vx + vy*vy);
    if (speed < 1) return;
    const p0 = lidar2screen(cx, cy, W);
    const p1 = lidar2screen(cx + vx*0.5, cy + vy*0.5, W);
    drawArrow(ctx, p0.x, p0.y, p1.x, p1.y, color, 1.5, W);
  }

  function jetColor(t) {
    let r,g,b;
    if (t<0.25) { r=0; g=Math.round(t/0.25*128); b=Math.round(128+t/0.25*127); }
    else if (t<0.5) { const s=(t-0.25)/0.25; r=0; g=Math.round(128+s*127); b=Math.round(255-s*128); }
    else if (t<0.75) { const s=(t-0.5)/0.25; r=Math.round(s*255); g=255; b=Math.round(127-s*127); }
    else { const s=(t-0.75)/0.25; r=255; g=Math.round(255-s*200); b=0; }
    return {r,g,b};
  }

  function drawBevOccupancy(ctx, W, occCells, bevOutSize, alphaMul, yOff, drawBar) {
    if (!occCells || !occCells.length) return;
    alphaMul = alphaMul == null ? 1 : alphaMul;
    yOff = yOff || 0;
    const cs = W / bevOutSize;
    for (const cell of occCells) {
      const [row, col, val] = cell;
      const sx = row * cs;
      const sy = col * cs + yOff;
      const c = jetColor(val);
      ctx.fillStyle = `rgba(${c.r},${c.g},${c.b},${Math.min(val*1.2, 0.85)*alphaMul})`;
      ctx.fillRect(sx, sy, cs+1, cs+1);
    }
    if (drawBar === false) return;

    const barW = 12, barH = W*0.3;
    const barX = W-barW-8, barY = (W-barH)/2;
    for (let i = 0; i < barH; i++) {
      const c = jetColor(1 - i/barH);
      ctx.fillStyle = `rgb(${c.r},${c.g},${c.b})`;
      ctx.fillRect(barX, barY+i, barW, 1);
    }
    ctx.fillStyle = "rgba(255,255,255,0.5)";
    ctx.font = `${9*(W/800)}px sans-serif`;
    ctx.textAlign = "right";
    ctx.fillText("High", barX-4, barY+10);
    ctx.fillText("Low", barX-4, barY+barH);
    ctx.textAlign = "start";
  }

  /* ═══════════════════════════════════════════
     OCSF – Real Data Step-through
     ═══════════════════════════════════════════ */
  let ocsf = { canvas:null, ctx:null, W:0, step:1 };

  const OCSF_DESCS = [
    "Detected objects on the BEV plane (forward view). Cyan = GT box, red dashed = enlarged GT box used as the class-agnostic BEV-mask target, orange = predictions.",
    "KGGI keypoints initialized at estimated object locations. Each dot is a Gaussian center extracted from the model's intermediate representation.",
    "Gaussian primitives with scale (σx, σy) and rotation shown as ellipses. Each primitive carries velocity and learned cross-modal features.",
    "Each Gaussian is splatted as a soft kernel onto the BEV grid and overlapping primitives accumulate into brighter regions. The BEV feature is produced by the Gaussians themselves, so energy appears only around keypoints — not on empty road."
  ];

  function initOCSF() {
    ocsf.canvas = document.getElementById("ocsfCanvas");
    if (!ocsf.canvas) return;
    ocsf.ctx = ocsf.canvas.getContext("2d");
    for (let i = 1; i <= 4; i++) {
      const btn = document.getElementById(`ocsf-step-${i}`);
      if (btn) btn.addEventListener("click", () => setOCSFStep(i));
    }
    resizeOCSF();
    window.addEventListener("resize", resizeOCSF);
  }

  function resizeOCSF() {
    const wrap = ocsf.canvas.parentElement;
    const sz = Math.min(wrap.clientWidth, wrap.clientHeight || wrap.clientWidth);
    const dpr = window.devicePixelRatio || 1;
    ocsf.canvas.width = Math.max(sz*dpr, 800);
    ocsf.canvas.height = ocsf.canvas.width;
    ocsf.canvas.style.width = sz+"px";
    ocsf.canvas.style.height = sz+"px";
    ocsf.W = ocsf.canvas.width;
    drawOCSF();
  }

  function setOCSFStep(s) {
    ocsf.step = s;
    for (let i = 1; i <= 4; i++) {
      const btn = document.getElementById(`ocsf-step-${i}`);
      if (btn) btn.classList.toggle("active", i === s);
    }
    const desc = document.getElementById("ocsf-step-desc");
    if (desc) desc.textContent = OCSF_DESCS[s-1];
    drawOCSF();
  }

  // Splat the displayed Gaussians as soft additive kernels — the BEV feature
  // is generated by the Gaussians themselves, so energy appears only where
  // Gaussians were initialized (i.e. around objects), not on empty road.
  function splatGaussians(ctx, W, g) {
    ctx.save();
    ctx.globalCompositeOperation = "lighter";
    for (let i = 0; i < g.means.length; i++) {
      const p = lidar2screen(g.means[i][0], g.means[i][1], W);
      const [sx, sy] = g.scales[i];
      const r = Math.max(meters2px(Math.max(sx, sy) * 3, W), 5*(W/800));
      const a = 0.35 * Math.min(g.opacities[i], 1);
      const grd = ctx.createRadialGradient(p.x, p.y, 0, p.x, p.y, r);
      grd.addColorStop(0, `rgba(255,170,60,${a})`);
      grd.addColorStop(0.5, `rgba(255,90,40,${a*0.5})`);
      grd.addColorStop(1, "rgba(255,60,30,0)");
      ctx.fillStyle = grd;
      ctx.beginPath(); ctx.arc(p.x, p.y, r, 0, Math.PI*2); ctx.fill();
    }
    ctx.restore();
  }

  function drawLabelChip(ctx, W, x, y, text, color) {
    const fs = 15 * (W / 800);
    ctx.font = `bold ${fs}px sans-serif`;
    const tw = ctx.measureText(text).width;
    ctx.fillStyle = "rgba(0,0,0,0.45)";
    ctx.fillRect(x - 4, y - fs, tw + 8, fs + 6);
    ctx.fillStyle = color;
    ctx.fillText(text, x, y);
    return tw + 14;
  }

  function drawOCSFBoxes(ctx, W, f) {
    // GT (cyan), enlarged GT box used for the BEV mask (red dashed), predictions (orange)
    f.gt_boxes.forEach(b => {
      drawBoxOnBev(ctx, W, enlargedBox(b), ENLARGE_COLOR, {dashed:true, fillAlpha:0});
      drawBoxOnBev(ctx, W, b, GT_COLOR, {dashed:false, fillAlpha:0});
    });
    f.pred_boxes.forEach(b => drawBoxOnBev(ctx, W, b, PRED_COLOR, {dashed:false, fillAlpha:0.1}));
  }

  function drawOCSF() {
    const { ctx, W, step } = ocsf;
    if (!ctx || !realOCSF) return;
    const f = realOCSF.frame;
    const sc = W / 800;
    const g = f.gaussians;

    dptfReset(ctx);
    ctx.fillStyle = "#0f0f23"; ctx.fillRect(0,0,W,W);
    dptfZoom(ctx, W);                       // forward-only view

    if (step === 4) {
      if (g) splatGaussians(ctx, W, g);
      drawOCSFBoxes(ctx, W, f);
    } else {
      drawBevBg(ctx, W);
      drawOCSFBoxes(ctx, W, f);
    }

    if ((step === 2 || step === 3) && g) {
      for (let i = 0; i < g.means.length; i++) {
        const p = lidar2screen(g.means[i][0], g.means[i][1], W);
        if (step === 2) {
          ctx.beginPath(); ctx.arc(p.x, p.y, 3.2*sc, 0, Math.PI*2);
          ctx.fillStyle = KP_COLOR; ctx.fill();
          ctx.strokeStyle = "rgba(0,0,0,0.5)"; ctx.lineWidth = 0.7*sc; ctx.stroke();
        } else {
          const [sx, sy] = g.scales[i];
          const [cosR, sinR] = g.rotations[i];
          ctx_save_draw_ellipse(ctx, p.x, p.y, meters2px(sx*2, W), meters2px(sy*2, W),
                                screenAngle(cosR, sinR), KP_COLOR, Math.min(g.opacities[i], 0.5));
          ctx.beginPath(); ctx.arc(p.x, p.y, 1.4*sc, 0, Math.PI*2);
          ctx.fillStyle = KP_COLOR; ctx.fill();
        }
      }
    }

    dptfReset(ctx);
    let lx = 12 * sc;
    lx += drawLabelChip(ctx, W, lx, 24*sc, "GT", GT_COLOR);
    lx += drawLabelChip(ctx, W, lx, 24*sc, "Enlarged", ENLARGE_COLOR);
    drawLabelChip(ctx, W, lx, 24*sc, "Pred", PRED_COLOR);
  }

  /* ═══════════════════════════════════════════
     DPTF – Real Data Step-through
     ═══════════════════════════════════════════ */
  const DPTF_CAM_COUNT = 20;
  let dptfCamImages = [];
  let dptfCamLoaded = false;

  // canvas ids → role
  const DPTF_CANVAS = ["dptfNoCompCanvas","dptfCompCanvas","dptfFeatCanvas","dptfDetCanvas"];
  const DPTF_DT = 0.7;  // s — propagation interval used to illustrate vel-compensation lag
  const LAG_DT = 1.3;   // s — larger interval for the Gaussian-path drift emphasis

  let dptf = {
    canvases:[], ctxs:[],
    W:0, step:0, running:false, timerId:null,
  };

  function preloadDPTFCam() {
    dptfCamImages = [];
    let loaded = 0;
    for (let i = 0; i < DPTF_CAM_COUNT; i++) {
      const img = new Image();
      img.onload = () => { loaded++; if (loaded === DPTF_CAM_COUNT) dptfCamLoaded = true; };
      img.src = `static/images/dptf_cam/${i}.jpg`;
      dptfCamImages.push(img);
    }
  }

  function updateDPTFCam(step) {
    if (!dptfCamLoaded) return;
    const src = dptfCamImages[step % DPTF_CAM_COUNT].src;
    document.querySelectorAll(".dptf-cam").forEach(el => { el.src = src; });
  }

  function initDPTF() {
    dptf.canvases = DPTF_CANVAS.map(id => document.getElementById(id));
    if (!dptf.canvases[0]) return;
    dptf.ctxs = dptf.canvases.map(c => c ? c.getContext("2d") : null);

    // Controls are duplicated per group (Gaussian / BEV); wire every instance.
    const bind = (sel, fn) => document.querySelectorAll(sel).forEach(b => b.addEventListener("click", fn));
    bind(".dptf-play", playDPTF);
    bind(".dptf-stop", stopDPTF);
    bind(".dptf-step", stepDPTF);
    bind(".dptf-reset", resetDPTF);

    preloadDPTFCam();
    resetDPTF();
    resizeDPTF();
    window.addEventListener("resize", resizeDPTF);
  }

  function resizeDPTF() {
    dptf.canvases.filter(Boolean).forEach(c => {
      const wrap = c.parentElement;
      const sz = Math.min(wrap.clientWidth, wrap.clientHeight || wrap.clientWidth);
      const dpr = window.devicePixelRatio || 1;
      c.width = Math.max(sz*dpr, 600);
      c.height = c.width;
      c.style.width = sz+"px";
      c.style.height = sz+"px";
    });
    dptf.W = dptf.canvases[0].width;
    drawDPTF();
  }

  function resetDPTF() {
    dptf.step = 0; stopDPTF();
    updateLabel();
    updateDPTFCam(0);
    drawDPTF();
  }

  function updateLabel() {
    if (!realDPTF) return;
    const fi = Math.min(dptf.step, realDPTF.num_frames - 1);
    const fr = realDPTF.frames[fi];
    document.querySelectorAll(".dptf-ego").forEach(el => {
      el.textContent = `Ego: ${fr.ego_speed_kmh} km/h`;
    });
  }

  function stepDPTF() {
    if (!realDPTF) return;
    dptf.step = Math.min(dptf.step + 1, realDPTF.num_frames - 1);
    updateLabel();
    updateDPTFCam(dptf.step);
    drawDPTF();
  }

  function playDPTF() {
    if (dptf.running) return;
    dptf.running = true;
    (function loop() {
      stepDPTF();
      if (dptf.running && dptf.step < realDPTF.num_frames - 1) {
        dptf.timerId = setTimeout(loop, 1000);  // 1 fps
      } else {
        dptf.running = false;
      }
    })();
  }
  function stopDPTF() { dptf.running = false; if (dptf.timerId) clearTimeout(dptf.timerId); }

  function dptfBoxes(ctx, W, fr) {
    // GT (cyan dashed) on top of everything, then predictions (orange)
    fr.gt_boxes.forEach(b => drawBoxOnBev(ctx, W, b, GT_COLOR, {dashed:true, fillAlpha:0}));
    fr.pred_boxes.forEach((b,i) => {
      const speed = i < fr.pred_velocities.length
        ? Math.hypot(fr.pred_velocities[i][0], fr.pred_velocities[i][1]) : 0;
      drawBoxOnBev(ctx, W, b, PRED_COLOR, {dashed:false, fillAlpha:Math.min(0.35, speed/30*0.35)});
    });
  }

  function dptfChips(ctx, W) {
    let x = 12 * (W/800);
    x += drawLabelChip(ctx, W, x, 26*(W/800), "GT", GT_COLOR);
    drawLabelChip(ctx, W, x, 26*(W/800), "Pred", PRED_COLOR);
  }

  // Front-only zoom: map the forward region (≈170 m ahead × ±84 m) to the full
  // square canvas so the BEV is larger and the rear is dropped.
  const DPTF_VIEW = { lyMin:-84.3, lyMax:84.3, lxMin:-153.6, lxMax:15 };
  function dptfZoom(ctx, W) {
    const v = DPTF_VIEW;
    const sx0 = (v.lyMin+PC_RANGE)/PC_FULL*W, sx1 = (v.lyMax+PC_RANGE)/PC_FULL*W;
    const sy0 = (v.lxMin+PC_RANGE)/PC_FULL*W, sy1 = (v.lxMax+PC_RANGE)/PC_FULL*W;
    const w = sx1-sx0, h = sy1-sy0;
    ctx.setTransform(W/w, 0, 0, W/h, -sx0*W/w, -sy0*W/h);
  }
  function dptfReset(ctx) { ctx.setTransform(1,0,0,1,0,0); }

  function drawColorbar(ctx, W) {
    const barW = 12, barH = W*0.3, barX = W-barW-8, barY = (W-barH)/2;
    for (let i = 0; i < barH; i++) {
      const c = jetColor(1 - i/barH);
      ctx.fillStyle = `rgb(${c.r},${c.g},${c.b})`; ctx.fillRect(barX, barY+i, barW, 1);
    }
    ctx.fillStyle = "rgba(255,255,255,0.5)"; ctx.font = `${9*(W/800)}px sans-serif`;
    ctx.textAlign = "right"; ctx.fillText("High", barX-4, barY+10);
    ctx.fillText("Low", barX-4, barY+barH); ctx.textAlign = "start";
  }

  // Gaussian-path panel. Only PAST (propagated) primitives are velocity-
  // compensated, so in the "before" panel they are drawn at their lagging
  // position p - v*dt; current-frame primitives are identical in both panels.
  function drawGaussianPanel(ctx, W, fr, before) {
    dptfReset(ctx); ctx.fillStyle = "#0f0f23"; ctx.fillRect(0,0,W,W);
    dptfZoom(ctx, W);
    drawBevBg(ctx, W);
    fr.gt_boxes.forEach(b => drawBoxOnBev(ctx, W, enlargedBox(b), ENLARGE_COLOR, {dashed:true, fillAlpha:0}));
    dptfBoxes(ctx, W, fr);

    const g = fr.gaussians;
    const tracked = fr.tracked || [];
    const s = W/800;
    if (g) {
      // Faint context: the rest of the Gaussian field.
      let drawn = 0;
      for (let i = 0; i < g.means.length && drawn < 150; i++) {
        if (g.opacities[i] < 0.5) continue;
        drawn++;
        const p = lidar2screen(g.means[i][0], g.means[i][1], W);
        ctx.beginPath(); ctx.arc(p.x, p.y, 1.0*s, 0, Math.PI*2);
        ctx.fillStyle = "rgba(150,160,190,0.09)"; ctx.fill();
      }
      // Tracked dynamic objects (1–2), shown in detail. In "before" their
      // propagated Gaussians are NOT velocity-compensated, so they drift well
      // behind the object (dashed connector); in "after" they sit on it.
      for (const t of tracked) {
        const color = TRACK_COLORS[t.tid % TRACK_COLORS.length];
        const boxP = lidar2screen(t.box[0], t.box[1], W);
        drawBoxOnBev(ctx, W, t.box, color, {dashed:false, fillAlpha:0.16});

        const pts = [];
        let mx = 0, my = 0;
        for (const gi of t.gidx) {
          if (gi >= g.means.length) continue;
          let [lx, ly] = g.means[gi];
          const v = g.velocities && g.velocities[gi] ? g.velocities[gi] : [0,0];
          if (before) { lx -= v[0]*LAG_DT; ly -= v[1]*LAG_DT; }
          const p = lidar2screen(lx, ly, W);
          pts.push({ p, lx, ly, v, gi }); mx += p.x; my += p.y;
        }
        if (!pts.length) continue;
        mx /= pts.length; my /= pts.length;

        // "drift" connector from the lagging Gaussians to the actual object
        if (before) {
          ctx.setLineDash([5,4]); ctx.strokeStyle = color; ctx.lineWidth = 1.4*s;
          ctx.beginPath(); ctx.moveTo(mx, my); ctx.lineTo(boxP.x, boxP.y); ctx.stroke();
          ctx.setLineDash([]);
          drawArrow(ctx, mx, my, boxP.x, boxP.y, color, 1.4*s, W);
        }
        for (const { p, lx, ly, v, gi } of pts) {
          const [sx,sy] = g.scales[gi];
          const [cosR,sinR] = g.rotations[gi];
          ctx_save_draw_ellipse(ctx, p.x, p.y, meters2px(sx*1.6, W), meters2px(sy*1.6, W),
                                screenAngle(cosR, sinR), color, 0.9);
          ctx.beginPath(); ctx.arc(p.x, p.y, 3.2*s, 0, Math.PI*2);
          ctx.fillStyle = color; ctx.fill();
          ctx.strokeStyle = "#fff"; ctx.lineWidth = 1.0*s;
          ctx.beginPath(); ctx.arc(p.x, p.y, 5.2*s, 0, Math.PI*2); ctx.stroke();
          drawVelocityArrow(ctx, W, lx, ly, v[0], v[1], "#f9c74f");
        }
      }
    }
    dptfReset(ctx);
    dptfChips(ctx, W);
    // emphasis caption
    ctx.textAlign = "center";
    ctx.font = `bold ${13*s}px sans-serif`;
    ctx.fillStyle = before ? "rgba(255,120,120,0.95)" : "rgba(140,222,150,0.95)";
    ctx.fillText(before ? "✗ primitives drift off the object"
                        : "✓ primitives stay on the object", W/2, W-12*s);
    ctx.textAlign = "start";
  }

  // BEV-path panel. Both panels are ego-motion compensated (static background
  // is aligned). The difference is the velocity head: without it (velComp=
  // false) moving objects' BEV features smear along their motion; with it they
  // stay sharp.
  function drawBevPathPanel(ctx, W, fi, velComp) {
    const fr = realDPTF.frames[fi];
    const size = realDPTF.bev_out_size, cs = W/size;
    dptfReset(ctx); ctx.fillStyle = "#0f0f23"; ctx.fillRect(0,0,W,W);
    dptfZoom(ctx, W);

    if (!velComp) {
      // motion-blur the BEV cells that fall inside fast-moving objects
      const moving = [];
      fr.pred_boxes.forEach((b,i) => {
        const v = fr.pred_velocities[i] || [0,0];
        if (Math.hypot(v[0], v[1]) > 3) moving.push({ b, v });
      });
      const TRAIL = 1.8, K = 10;   // longer, denser motion blur
      for (const cell of fr.bev_occupancy) {
        const [row, col, val] = cell;
        const ly = row/size*PC_FULL - PC_RANGE, lx = col/size*PC_FULL - PC_RANGE;
        let mv = null;
        for (const o of moving) { if (pointInBox(lx, ly, o.b, 1.5)) { mv = o.v; break; } }
        if (!mv) continue;
        const c = jetColor(val);
        for (let k = 1; k <= K; k++) {
          const f = k/K;
          const dsx = (-mv[1]*DPTF_DT*TRAIL*f)/PC_FULL*W, dsy = (-mv[0]*DPTF_DT*TRAIL*f)/PC_FULL*W;
          ctx.fillStyle = `rgba(${c.r},${c.g},${c.b},${Math.min(val*1.2,0.85)*0.85*(1-f*0.8)})`;
          ctx.fillRect(row*cs+dsx, col*cs+dsy, cs+1.5, cs+1.5);
        }
      }
    }
    drawBevOccupancy(ctx, W, fr.bev_occupancy, size, 1, 0, false);
    dptfBoxes(ctx, W, fr);

    dptfReset(ctx);
    drawColorbar(ctx, W);
    dptfChips(ctx, W);
  }

  function drawDPTF() {
    if (!realDPTF || !dptf.ctxs.length) return;
    const fi = Math.min(dptf.step, realDPTF.num_frames - 1);
    const fr = realDPTF.frames[fi];
    const W = dptf.W;

    // Top row — Gaussian path: before / after velocity compensation
    if (dptf.ctxs[0]) drawGaussianPanel(dptf.ctxs[0], W, fr, true);
    if (dptf.ctxs[1]) drawGaussianPanel(dptf.ctxs[1], W, fr, false);
    // Bottom row — BEV path: without / with velocity compensation (both ego-comp)
    if (dptf.ctxs[2]) drawBevPathPanel(dptf.ctxs[2], W, fi, false);
    if (dptf.ctxs[3]) drawBevPathPanel(dptf.ctxs[3], W, fi, true);
  }

  function ctx_save_draw_ellipse(ctx, x, y, rx, ry, angle, color, alpha) {
    ctx.save(); ctx.translate(x, y); ctx.rotate(angle);
    ctx.beginPath(); ctx.ellipse(0, 0, Math.max(rx,1), Math.max(ry,1), 0, 0, Math.PI*2);
    ctx.strokeStyle = color; ctx.lineWidth = 1; ctx.globalAlpha = alpha;
    ctx.stroke(); ctx.globalAlpha = 1; ctx.restore();
  }

  /* ═══ Qualitative Carousel ═══ */
  function initCarousel() {
    const scenes = [
      { src: "static/images/demo_sunny.gif?v=12", label: "Sunny" },
      { src: "static/images/demo_rainy.gif?v=12", label: "Rainy" },
      { src: "static/images/demo_night.gif?v=12", label: "Night" },
    ];
    let idx = 0;
    const img = document.getElementById("qual-carousel-img");
    const lbl = document.getElementById("qual-carousel-label");
    const dots = document.querySelectorAll(".qual-dot");
    if (!img || !lbl) return;

    function show(i) {
      idx = ((i % scenes.length) + scenes.length) % scenes.length;
      img.src = scenes[idx].src;
      lbl.textContent = scenes[idx].label;
      dots.forEach((d,j) => d.style.background = j===idx ? "#4361ee" : "#ccc");
    }

    document.getElementById("qual-prev")?.addEventListener("click", () => show(idx-1));
    document.getElementById("qual-next")?.addEventListener("click", () => show(idx+1));
    dots.forEach(d => d.addEventListener("click", () => show(parseInt(d.dataset.idx))));
  }

  /* ═══ Boot ═══ */
  async function boot() {
    initOCSF();
    initDPTF();
    initCarousel();

    try {
      const [ocsfResp, dptfResp] = await Promise.all([
        fetch("static/data/ocsf_demo.json?v=12"),
        fetch("static/data/dptf_demo.json?v=12"),
      ]);
      if (ocsfResp.ok) {
        realOCSF = await ocsfResp.json();
        setOCSFStep(1);
      }
      if (dptfResp.ok) {
        realDPTF = await dptfResp.json();
        resetDPTF();
      }
    } catch (e) {
      console.warn("Demo data not loaded:", e);
    }
  }

  if (document.readyState === "loading") document.addEventListener("DOMContentLoaded", boot);
  else boot();
})();
