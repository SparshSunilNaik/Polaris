/*
 * Mission map and replay. World geometry follows simulation/worlds/polaris_antarctica.sdf
 * (Gazebo ENU: x east, y north); waypoints follow config/missions/polaris_demo.json
 * (north/east offsets from takeoff). SVG y is -north so north points up.
 */
(() => {
  "use strict";
  const NS = "http://www.w3.org/2000/svg";
  const $ = (id) => document.getElementById(id);
  const reduceMotion = window.matchMedia("(prefers-reduced-motion: reduce)").matches;
  const el = (n, a, p, text) => { const e = document.createElementNS(NS, n); for (const k in a) e.setAttribute(k, a[k]); if (text != null) e.textContent = text; if (p) p.appendChild(e); return e; };
  const P = (east, north) => [east, -north];

  /* ---------- waypoints ---------- */
  const WP = {
    "corridor-staging": { e: 4, n: 0, label: "Corridor staging", note: "required" },
    "rock-observation": { e: 7, n: 2, label: "Rock observation", note: "observation, 2.5 to 3.0 m stand-off", obs: true },
    "station-approach": { e: 9, n: -2, label: "Station approach", note: "optional" },
    "landing-zone-staging": { e: 11.5, n: 0, label: "Landing-zone staging", note: "surrogate for the pad" },
    "replan-staging": { e: 3.4, n: 2.6, label: "Generated staging", note: "replan, position illustrative", generated: true },
  };

  // Stand-off point 2.93 m short of the rock-observation coordinate, approached from corridor staging.
  const standoff = (() => {
    const dx = 7 - 4, dy = 2 - 0, d = Math.hypot(dx, dy);
    return [7 - (dx / d) * 2.93, 2 - (dy / d) * 2.93];
  })();

  /* ---------- retained runs (from docs/presentation_prep/12_FINAL_PROJECT_SUMMARY.txt) ---------- */
  const RUNS = {
    success: {
      wps: ["corridor-staging", "rock-observation", "station-approach", "landing-zone-staging"],
      summary: [["Result", "COMPLETED"], ["Elapsed", "106.3 s"], ["Waypoints", "4 of 4"], ["Recoveries", "1"], ["Avoidances", "4"], ["Collisions", "0 recorded"]],
      frames: [
        { t: 0, pos: [0, 0], yaw: 0, phase: "PREFLIGHT", ev: "Five healthy distinct depth updates; readiness passed" },
        { t: 6, pos: [0, 0], yaw: 0, phase: "TAKEOFF", ev: "Takeoff to 2.5 m" },
        { t: 10, pos: [0, 0], yaw: 0, phase: "WAYPOINT_EXECUTE", wp: "corridor-staging", st: { "corridor-staging": "active" }, ev: "Navigator proposes bounded motion east" },
        { t: 22, pos: [4, 0], yaw: 0, phase: "WAYPOINT_VERIFY", wp: "corridor-staging", ev: "Holding 1.0 s to verify" },
        { t: 25, pos: [4, 0], yaw: 0, phase: "WAYPOINT_COMPLETE", wp: "corridor-staging", st: { "corridor-staging": "done" }, ev: "Corridor staging complete" },
        { t: 27, pos: [4, 0], yaw: 34, phase: "WAYPOINT_EXECUTE", wp: "rock-observation", obs: "APPROACHING", st: { "rock-observation": "active" }, ev: "Turning toward the rock-observation target" },
        { t: 33, pos: standoff, yaw: 34, phase: "WAYPOINT_EXECUTE", wp: "rock-observation", obs: "OBSERVATION_CANDIDATE", ev: "Inside the 2.5 to 3.0 m stand-off window" },
        { t: 37, pos: standoff, yaw: 30, phase: "WAYPOINT_VERIFY", wp: "rock-observation", obs: "OBSERVING", ev: "Dwell started: yaw-only hold, 3 fresh depth frames needed" },
        { t: 42, pos: standoff, yaw: 30, phase: "WAYPOINT_COMPLETE", wp: "rock-observation", obs: "OBSERVATION_COMPLETE", st: { "rock-observation": "done" }, ev: "Observation complete: 2.93 m, −3.9°, 3 fresh frames" },
        { t: 45, pos: standoff, yaw: -45, phase: "WAYPOINT_EXECUTE", wp: "station-approach", obs: "n/a", st: { "station-approach": "active" }, ev: "Optional station approach" },
        { t: 56, pos: [9, -2], yaw: -45, phase: "WAYPOINT_COMPLETE", wp: "station-approach", st: { "station-approach": "done" }, ev: "Station approach complete" },
        { t: 59, pos: [9, -2], yaw: 38, phase: "WAYPOINT_EXECUTE", wp: "landing-zone-staging", st: { "landing-zone-staging": "active" }, ev: "Heading to the surrogate landing-zone staging point" },
        { t: 70, pos: [11.5, 0], yaw: 38, phase: "MISSION_COMPLETE", wp: "landing-zone-staging", st: { "landing-zone-staging": "done" }, ev: "All 4 waypoints complete" },
        { t: 73, pos: [11.5, 0], yaw: 38, phase: "LAND", ev: "Landing at current position" },
        { t: 80, pos: [11.5, 0], yaw: 38, phase: "FINISHED", landed: true, ev: "Landing and disarm confirmed" },
      ],
    },
    failure: {
      wps: ["rock-observation", "replan-staging"],
      summary: [["Result", "ABORTED"], ["Started at", "rock-observation"], ["Replans", "1 of 1"], ["Collisions", "0 recorded"], ["Landing", "confirmed"], ["Disarm", "confirmed"]],
      frames: [
        { t: 0, pos: [0, 0], yaw: 0, phase: "PREFLIGHT", ev: "Readiness passed" },
        { t: 5, pos: [0, 0], yaw: 0, phase: "TAKEOFF", ev: "Takeoff to 2.5 m" },
        { t: 9, pos: [0, 0], yaw: 16, phase: "WAYPOINT_EXECUTE", wp: "rock-observation", obs: "APPROACHING", st: { "rock-observation": "active" }, ev: "Mission starts at rock-observation" },
        { t: 20, pos: [2.2, 1.0], yaw: 16, phase: "WAYPOINT_EXECUTE", wp: "rock-observation", obs: "APPROACHING", ev: "Observation evidence never satisfies every gate" },
        { t: 28, pos: [2.2, 1.0], yaw: 16, phase: "WAYPOINT_FAILED", wp: "rock-observation", obs: "TIMED_OUT", st: { "rock-observation": "failed" }, ev: "Observation timed out" },
        { t: 31, pos: [2.2, 1.0], yaw: 16, phase: "REPLAN", st: { "replan-staging": "active" }, ev: "Bounded replan: one HEALTHIER_VIEWPOINT staging waypoint" },
        { t: 40, pos: [3.4, 2.6], yaw: 40, phase: "WAYPOINT_EXECUTE", wp: "replan-staging", ev: "Trying the generated viewpoint" },
        { t: 49, pos: [3.4, 2.6], yaw: 40, phase: "WAYPOINT_FAILED", wp: "replan-staging", st: { "replan-staging": "failed" }, ev: "Timed out again; the replan budget is used" },
        { t: 52, pos: [3.4, 2.6], yaw: 40, phase: "ABORT", ev: "Mission aborted, failing closed" },
        { t: 56, pos: [3.4, 2.6], yaw: 40, phase: "LAND", ev: "Landing at current position" },
        { t: 62, pos: [3.4, 2.6], yaw: 40, phase: "FINISHED", landed: true, ev: "Landing and disarm confirmed, 0 collisions" },
      ],
    },
  };

  /* ---------- static map ---------- */
  const map = $("map");
  // The view covers the mission area (x -3..22 east, north -8..12); features beyond it get edge pointers.
  const gStatic = el("g", { class: "m-static" }, map);
  for (let i = 0; i < 10; i++) {
    const r = 3 + i * 2.2;
    el("ellipse", { cx: -6, cy: 6, rx: r * 1.3, ry: r, class: "m-contour" }, gStatic);
  }
  el("path", { d: "M-1.2 0H1.2M0 -1.2V1.2", class: "m-takeoff" }, gStatic);
  el("text", { x: 0, y: 2.4, class: "m-label", "text-anchor": "middle" }, gStatic, "Takeoff");
  for (const s of [-1, 1]) el("line", { x1: 5, x2: 23, y1: s * 4.5, y2: s * 4.5, class: "m-wall" }, gStatic);
  el("text", { x: 17.5, y: -5.1, class: "m-label", "text-anchor": "middle" }, gStatic, "Corridor wall");
  el("text", { x: 17.5, y: 5.7, class: "m-label", "text-anchor": "middle" }, gStatic, "Corridor wall");
  const [ox, oy] = P(15, 1.8);
  el("rect", { x: ox - 0.8, y: oy - 0.8, width: 1.6, height: 1.6, class: "m-obstacle" }, gStatic);
  el("text", { x: ox, y: oy - 1.3, class: "m-label", "text-anchor": "middle" }, gStatic, "Offset obstacle");
  for (const [e, n, r] of [[8, 10, 0.9], [12, 13, 1.1], [16, 9, 0.8], [18, 15, 1.0], [11, 17, 0.7]]) {
    const [x, y] = P(e, n);
    el("circle", { cx: x, cy: y, r, class: "m-rock" }, gStatic);
  }
  el("text", { x: 12, y: -9.3, class: "m-label", "text-anchor": "middle" }, gStatic, "Rock landmarks");
  // edge pointers
  el("text", { x: 21.6, y: 0.35, class: "m-edge", "text-anchor": "end" }, gStatic, "Gate and pad →");
  el("text", { x: 12, y: 7.6, class: "m-edge", "text-anchor": "middle" }, gStatic, "Research station ↓");
  el("path", { d: "M-1.6 -9.4 L-1 -11.2 L-0.4 -9.4 Z", class: "m-north" }, gStatic);
  el("text", { x: -1, y: -8.3, class: "m-label", "text-anchor": "middle" }, gStatic, "N");

  const gWp = el("g", {}, map);
  const trail = el("path", { class: "m-trail", d: "" }, map);
  const drone = el("g", { class: "m-drone" }, map);
  el("path", { d: "M0 0 L5 -2.2 A5.4 5.4 0 0 1 5 2.2 Z", class: "m-fov" }, drone);
  for (const [dx, dy] of [[-0.55, -0.55], [0.55, -0.55], [-0.55, 0.55], [0.55, 0.55]]) {
    el("line", { x1: 0, y1: 0, x2: dx, y2: dy, class: "m-arm" }, drone);
    el("circle", { cx: dx, cy: dy, r: 0.38, class: "m-rotor" }, drone);
  }
  el("circle", { cx: 0, cy: 0, r: 0.32, class: "m-body" }, drone);

  /* ---------- replay ---------- */
  let run = "success", t = 0, playing = false, raf = 0, last = 0;
  const scrub = $("scrub"), playBtn = $("play");
  const frames = () => RUNS[run].frames;
  const total = () => frames()[frames().length - 1].t;
  const ST_LABEL = { pending: "pending", active: "active", done: "complete", failed: "failed" };

  function drawWaypoints() {
    gWp.textContent = "";
    RUNS[run].wps.forEach((id, i) => {
      const w = WP[id];
      const [x, y] = P(w.e, w.n);
      const g = el("g", { class: "m-wp", "data-wp": id }, gWp);
      if (w.obs) el("circle", { cx: x, cy: y, r: 2.75, class: "m-standoff" }, g);
      el("rect", { x: x - 0.62, y: y - 0.62, width: 1.24, height: 1.24, transform: `rotate(45 ${x} ${y})`, class: "m-wp-mark" + (w.generated ? " gen" : "") }, g);
      el("text", { x, y: y + 0.3, class: "m-wp-num", "text-anchor": "middle" }, g, w.generated ? "R" : String(i + 1));
    });
    $("wps").innerHTML = RUNS[run].wps.map((id, i) => `<li data-wp="${id}" data-st="pending"><span class="wp-dot" aria-hidden="true">${WP[id].generated ? "R" : i + 1}</span><span class="wp-name">${WP[id].label}</span><span class="wp-note">${WP[id].note}</span><span class="wp-st">pending</span></li>`).join("");
    $("summary").innerHTML = RUNS[run].summary.map(([k, v]) => `<div><dt>${k}</dt><dd>${v}</dd></div>`).join("");
  }

  function render() {
    const fs = frames();
    let i = 0;
    while (i < fs.length - 1 && fs[i + 1].t <= t) i++;
    const a = fs[i], b = fs[Math.min(i + 1, fs.length - 1)];
    const k = b.t > a.t ? Math.min(1, Math.max(0, (t - a.t) / (b.t - a.t))) : 0;
    const s = k * k * (3 - 2 * k);
    const e = a.pos[0] + (b.pos[0] - a.pos[0]) * s;
    const n = a.pos[1] + (b.pos[1] - a.pos[1]) * s;
    const yaw = a.yaw + (b.yaw - a.yaw) * s;
    const [x, y] = P(e, n);
    // Heading is degrees counter-clockwise from east; SVG rotation is clockwise.
    drone.setAttribute("transform", `translate(${x} ${y}) rotate(${-yaw})`);
    drone.classList.toggle("landed", !!fs[i].landed);
    drone.classList.toggle("ground", fs[i].phase === "PREFLIGHT");

    const pts = [];
    for (let j = 0; j <= i; j++) {
      const [px, py] = P(...fs[j].pos);
      const lastPt = pts[pts.length - 1];
      if (!lastPt || lastPt[0] !== px || lastPt[1] !== py) pts.push([px, py]);
    }
    pts.push([x, y]);
    trail.setAttribute("d", pts.map((p, j) => `${j ? "L" : "M"}${p[0].toFixed(2)} ${p[1].toFixed(2)}`).join(""));

    let phase = "PREFLIGHT", wp = "none", obs = "n/a";
    const st = {};
    for (let j = 0; j <= i; j++) {
      const f = fs[j];
      phase = f.phase;
      if (f.wp) wp = WP[f.wp].label;
      if (f.obs) obs = f.obs;
      Object.assign(st, f.st || {});
    }
    $("r-phase").textContent = phase;
    $("r-phase").dataset.kind = /ABORT|FAILED/.test(phase) ? "bad" : /COMPLETE|FINISHED/.test(phase) ? "good" : "";
    $("r-wp").textContent = wp;
    $("r-obs").textContent = obs;
    document.querySelectorAll("#wps li").forEach((li) => {
      const v = st[li.dataset.wp] || "pending";
      li.dataset.st = v;
      li.querySelector(".wp-st").textContent = ST_LABEL[v];
    });
    gWp.querySelectorAll(".m-wp").forEach((g) => g.setAttribute("data-st", st[g.dataset.wp] || "pending"));

    const evList = $("events");
    const shown = fs.slice(0, i + 1);
    if (evList.childElementCount !== shown.length) {
      evList.innerHTML = shown.map((f, j) => `<li class="${j === shown.length - 1 ? "now" : ""}"><span class="ev-phase">${f.phase}</span>${f.ev}</li>`).join("");
      evList.scrollTop = evList.scrollHeight;
    }
    scrub.value = String(Math.round((t / total()) * 1000));
  }

  function setPlaying(p) {
    playing = p;
    playBtn.classList.toggle("is-playing", p);
    playBtn.setAttribute("aria-label", p ? "Pause replay" : "Play replay");
    cancelAnimationFrame(raf);
    if (p) { last = 0; raf = requestAnimationFrame(tick); }
  }
  function tick(now) {
    if (last) t = Math.min(total(), t + ((now - last) / 1000) * 5.5);
    last = now;
    render();
    if (t >= total()) { setPlaying(false); return; }
    raf = requestAnimationFrame(tick);
  }

  playBtn.addEventListener("click", () => { if (t >= total()) t = 0; setPlaying(!playing); });
  scrub.addEventListener("input", () => { setPlaying(false); t = (scrub.value / 1000) * total(); render(); });
  document.querySelectorAll(".run-switch button").forEach((b) => b.addEventListener("click", () => {
    run = b.dataset.run;
    document.querySelectorAll(".run-switch button").forEach((x) => x.setAttribute("aria-pressed", String(x === b)));
    t = 0; $("events").textContent = ""; drawWaypoints(); render();
    if (!reduceMotion) setPlaying(true);
  }));

  drawWaypoints();
  render();

  // Scale bar: 5 m in map units, sized from the rendered map width.
  const scaleBar = $("scale-bar");
  const VIEW_W = map.viewBox.baseVal.width;
  const sizeScale = () => { scaleBar.style.width = `${(5 / VIEW_W) * map.getBoundingClientRect().width}px`; };
  sizeScale();
  new ResizeObserver(sizeScale).observe(map);

  // Fly the completed run once when the map first comes into view.
  if (!reduceMotion) {
    let started = false;
    new IntersectionObserver((es, obs) => {
      if (!started && es[0].isIntersecting) { started = true; obs.disconnect(); setTimeout(() => { if (t === 0) setPlaying(true); }, 500); }
    }, { threshold: 0.35 }).observe(map);
  } else {
    t = total(); render();
  }

  /* ---------- hero contour background ---------- */
  const contours = $("contours");
  const W = 1600, H = 900;
  contours.setAttribute("viewBox", `0 0 ${W} ${H}`);
  for (const [cx, cy, s] of [[1180, 160, 1], [240, 640, 0.8]]) {
    for (let r = 40; r < 900; r += 34) {
      let d = "";
      for (let a = 0; a <= 64; a++) {
        const th = (a / 64) * Math.PI * 2;
        const wob = 1 + 0.07 * Math.sin(th * 3 + r * 0.02) + 0.05 * Math.cos(th * 5 - r * 0.013);
        d += `${a ? "L" : "M"}${(cx + Math.cos(th) * r * 1.35 * wob * s).toFixed(1)} ${(cy + Math.sin(th) * r * wob * s).toFixed(1)}`;
      }
      el("path", { d: d + "Z", class: "contour" }, contours);
    }
  }
})();
