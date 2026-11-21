/*
 * Browser port of ReactiveAvoidanceController.update() from
 * src/polaris/control/reactive_avoidance.py, with config/reactive_avoidance.json.
 * Health inputs other than visual health and depth stability are held healthy.
 */
(() => {
  "use strict";
  const $ = (id) => document.getElementById(id);

  const CFG = {
    forward: 0.4, cautious: 0.16, sidestep: 0.25, retreat: 0.2,
    caution: 0.45, stop: 0.62, emergency: 0.9, clear: 0.35, margin: 0.08,
    blockedN: 2, clearN: 3, retreatN: 3, hold: 2.0, cooldown: 1.0,
    maxEpisode: 8.0, maxReversals: 2,
    minVisual: 0.35, minDepth: 0.4, minStab: 0.75, critStab: 0.4, preferred: "LEFT",
  };

  const summary = (g) => ({
    centre: g[4], left: Math.max(g[0], g[3]), right: Math.max(g[2], g[5]),
    upper: Math.max(g[0], g[1], g[2]), lower: Math.max(g[6], g[7], g[8]),
    forward: Math.max(g[4], 0.5 * g[1]),
  });

  class Controller {
    constructor() { this.reset(); }
    reset() {
      this.s = { mode: "CLEAR", modeAt: 0, lastSeq: 0, blocked: 0, left: 0, right: 0, clear: 0, retreatN: 0,
        dir: "NONE", dirAt: 0, reversals: 0, episode: 0 };
      this.aborted = false;
    }
    transition(mode, now) {
      const s = this.s;
      if (mode === "AVOID_LEFT" || mode === "AVOID_RIGHT" || mode === "RETREAT") { if (!s.episode) s.episode = now; }
      else if (mode === "CLEAR") s.episode = 0;
      if (s.mode !== mode) { s.mode = mode; s.modeAt = now; }
    }
    dec(mode, cmd, reason, now, transition = true) {
      if (transition) this.transition(mode, now);
      if (mode === "ABORT") this.aborted = true;
      return { mode, cmd, reason };
    }
    persistence(seq, r) {
      const s = this.s;
      if (seq === 0 || seq === s.lastSeq) return;
      s.lastSeq = seq;
      s.blocked = r.centre >= CFG.stop ? s.blocked + 1 : 0;
      s.left = r.left >= CFG.stop ? s.left + 1 : 0;
      s.right = r.right >= CFG.stop ? s.right + 1 : 0;
      s.clear = r.centre <= CFG.clear && r.left < CFG.stop && r.right < CFG.stop ? s.clear + 1 : 0;
      s.retreatN = r.centre >= CFG.stop && r.left >= CFG.stop && r.right >= CFG.stop ? s.retreatN + 1 : 0;
    }
    safer(r) {
      if (Math.abs(r.left - r.right) < CFG.margin) {
        if (Math.max(r.left, r.right) >= CFG.stop) return null;
        return CFG.preferred;
      }
      return r.left < r.right ? "LEFT" : "RIGHT";
    }
    sidestep(dir) { return [0, dir === "LEFT" ? -CFG.sidestep : CFG.sidestep]; }
    cmdFor(mode) {
      if (mode === "AVOID_LEFT") return this.sidestep("LEFT");
      if (mode === "AVOID_RIGHT") return this.sidestep("RIGHT");
      if (mode === "RETREAT") return [-CFG.retreat, 0];
      return [0, 0];
    }
    escape(dir, r, now, reason = "Persistent centre obstacle") {
      const s = this.s;
      const dest = dir === "LEFT" ? r.left : r.right;
      if (dest >= CFG.stop) {
        if (!s.episode) s.episode = now;
        return this.dec("HOVER", [0, 0], "Selected escape sector is blocked", now);
      }
      if (s.dir !== "NONE" && s.dir !== dir) {
        const cur = s.dir === "LEFT" ? r.left : r.right;
        if (cur < CFG.stop && now - s.dirAt < CFG.hold) {
          dir = s.dir; reason = "Preferred direction hold prevented reversal";
        } else {
          s.reversals += 1;
          if (s.reversals > CFG.maxReversals) return this.dec("ABORT", [0, 0], "Maximum avoidance reversal count exceeded", now);
        }
      }
      if (s.dir !== dir) { s.dir = dir; s.dirAt = now; }
      return this.dec(dir === "LEFT" ? "AVOID_LEFT" : "AVOID_RIGHT", this.sidestep(dir), reason, now);
    }
    update(grid, seq, visual, stab, now) {
      const s = this.s;
      if (this.aborted) return { mode: "ABORT", cmd: [0, 0], reason: "Aborted: the runtime hovers, then lands" };
      const r = summary(grid);
      this.persistence(seq, r);
      const elapsed = s.episode ? Math.max(0, now - s.episode) : 0;
      if (s.episode && elapsed > CFG.maxEpisode) return this.dec("ABORT", [0, 0], "Maximum avoidance duration exceeded", now);
      if (visual < CFG.minVisual) return this.dec("HOVER", [0, 0], "Visual health is below motion minimum", now);
      if (stab < CFG.critStab) return this.dec("HOVER", [0, 0], "Depth stability is critically low", now);
      if (r.centre >= CFG.emergency) return this.dec("RETREAT", [-CFG.retreat, 0], "Emergency centre risk", now);

      if (s.mode === "AVOID_LEFT" || s.mode === "AVOID_RIGHT" || s.mode === "RETREAT") {
        const chosen = s.mode === "AVOID_LEFT" ? r.left : s.mode === "AVOID_RIGHT" ? r.right : 0;
        if (chosen >= CFG.stop) return this.dec("HOVER", [0, 0], "Chosen escape sector became blocked", now);
        if (s.clear >= CFG.clearN && now - s.dirAt >= CFG.hold && now - s.modeAt >= CFG.cooldown) {
          this.transition("CLEAR", now);
        } else {
          return this.dec(s.mode, this.cmdFor(s.mode), "Holding avoidance direction until clear persistence", now, false);
        }
      }

      if (s.left >= CFG.blockedN && s.right >= CFG.blockedN) {
        if (s.blocked < CFG.blockedN) return this.dec("CAUTION", [CFG.cautious, 0], "Both sides blocked but centre remains clear", now);
        if (!s.episode) s.episode = now;
        if (s.retreatN < CFG.retreatN) return this.dec("HOVER", [0, 0], "Hovering while both-side blockage is confirmed", now);
        return this.dec("RETREAT", [-CFG.retreat, 0], "Both side sectors are persistently blocked", now);
      }
      if (s.blocked >= CFG.blockedN) {
        const dir = this.safer(r);
        if (dir === null) return this.dec("RETREAT", [-CFG.retreat, 0], "No side has a reliable clearance margin", now);
        return this.escape(dir, r, now);
      }
      if (s.left >= CFG.blockedN) return this.escape("RIGHT", r, now, "Persistent left-sector risk");
      if (s.right >= CFG.blockedN) return this.escape("LEFT", r, now, "Persistent right-sector risk");

      if (r.forward >= CFG.caution || stab < CFG.minStab) {
        return this.dec("CAUTION", [CFG.cautious, 0], "Forward risk or perception quality requires reduced speed", now);
      }
      return this.dec("CLEAR", [CFG.forward, 0], "Forward sector clear", now);
    }
  }

  /* ---------- UI ---------- */
  const LEVELS = [0.1, 0.4, 0.5, 0.7, 0.95];
  const NAMES = ["UP_LEFT", "UP", "UP_RIGHT", "LEFT", "CENTRE", "RIGHT", "DOWN_LEFT", "DOWN", "DOWN_RIGHT"];
  const PRESETS = {
    "Open snowfield": [0.1, 0.1, 0.1, 0.1, 0.15, 0.1, 0.4, 0.5, 0.4],
    "Rock dead ahead": [0.1, 0.4, 0.1, 0.2, 0.7, 0.4, 0.4, 0.5, 0.4],
    "Narrow corridor": [0.4, 0.1, 0.4, 0.7, 0.2, 0.7, 0.4, 0.5, 0.4],
    "Boxed in": [0.5, 0.5, 0.5, 0.7, 0.7, 0.7, 0.5, 0.5, 0.5],
    "Wall, very close": [0.4, 0.7, 0.4, 0.5, 0.95, 0.5, 0.5, 0.5, 0.5],
  };
  let grid = PRESETS["Open snowfield"].slice();
  const ctl = new Controller();
  let now = 0, tick = 0, seq = 1, last = 0, raf = 0, visible = false;

  const riskEl = $("risk");
  riskEl.innerHTML = NAMES.map((n, i) => `<button type="button" class="cell" data-i="${i}" aria-label="${n.replace("_", " ").toLowerCase()} risk"><span class="cell-name">${n.replace("_", " ")}</span><span class="cell-val"></span></button>`).join("");
  const cells = [...riskEl.querySelectorAll(".cell")];
  const band = (v) => (v >= CFG.emergency ? "emergency" : v >= CFG.stop ? "stop" : v >= CFG.caution ? "caution" : v > CFG.clear ? "watch" : "clear");
  function paintGrid() {
    cells.forEach((c, i) => {
      c.dataset.band = band(grid[i]);
      c.querySelector(".cell-val").textContent = grid[i].toFixed(2);
      c.setAttribute("aria-label", `${NAMES[i].replace("_", " ").toLowerCase()} risk ${grid[i].toFixed(2)}`);
    });
  }
  cells.forEach((c, i) => {
    c.addEventListener("click", (e) => {
      const idx = LEVELS.findIndex((l) => l >= grid[i] - 1e-6);
      const cur = idx === -1 ? LEVELS.length - 1 : idx;
      const next = e.shiftKey ? Math.max(0, cur - 1) : (cur + 1) % LEVELS.length;
      grid[i] = LEVELS[next];
      paintGrid();
      markPreset(null);
    });
    c.addEventListener("keydown", (e) => {
      const d = { ArrowUp: 0.05, ArrowRight: 0.05, ArrowDown: -0.05, ArrowLeft: -0.05 }[e.key];
      if (d) { e.preventDefault(); grid[i] = Math.min(1, Math.max(0, +(grid[i] + d).toFixed(2))); paintGrid(); markPreset(null); }
    });
  });

  const presetsEl = $("presets");
  Object.keys(PRESETS).forEach((name) => {
    const b = document.createElement("button");
    b.type = "button";
    b.textContent = name;
    b.setAttribute("aria-pressed", String(name === "Open snowfield"));
    b.addEventListener("click", () => { grid = PRESETS[name].slice(); paintGrid(); markPreset(name); });
    presetsEl.appendChild(b);
  });
  function markPreset(name) { presetsEl.querySelectorAll("button").forEach((b) => b.setAttribute("aria-pressed", String(b.textContent === name))); }

  const vis = $("p-visual"), stab = $("p-stab");
  vis.addEventListener("input", () => ($("p-visual-out").textContent = (+vis.value).toFixed(2)));
  stab.addEventListener("input", () => ($("p-stab-out").textContent = (+stab.value).toFixed(2)));

  const fmt = (c) => {
    const parts = [];
    if (c[0] > 0) parts.push(`forward ${c[0].toFixed(2)} m/s`);
    if (c[0] < 0) parts.push(`back ${(-c[0]).toFixed(2)} m/s`);
    if (c[1] < 0) parts.push(`left ${(-c[1]).toFixed(2)} m/s`);
    if (c[1] > 0) parts.push(`right ${c[1].toFixed(2)} m/s`);
    return parts.length ? parts.join(", ") : "zero velocity";
  };

  let lastMode = "";
  function show(d) {
    const modeEl = $("mode");
    modeEl.textContent = d.mode;
    modeEl.dataset.mode = d.mode;
    if (d.mode !== lastMode) { modeEl.classList.remove("flip"); void modeEl.offsetWidth; modeEl.classList.add("flip"); lastMode = d.mode; }
    $("cmd").textContent = fmt(d.cmd);
    $("why").textContent = d.reason;
    const mag = Math.hypot(d.cmd[0], d.cmd[1]);
    const len = mag ? 8 + (mag / 0.4) * 15 : 0;
    $("cv-g").setAttribute("transform", `rotate(${Math.atan2(d.cmd[1], d.cmd[0]) * 180 / Math.PI})`);
    $("cv-line").setAttribute("y2", -len);
    $("cv-head").setAttribute("d", len ? `M-4 ${-len + 5} L0 ${-len} L4 ${-len + 5}` : "M0 0");
    const r = summary(grid);
    $("sectors").innerHTML = [["Centre", r.centre], ["Left", r.left], ["Right", r.right], ["Forward", r.forward]]
      .map(([k, v]) => `<div data-band="${band(v)}"><dt>${k}</dt><dd>${v.toFixed(2)}</dd></div>`).join("");
    const s = ctl.s;
    const ep = s.episode ? Math.max(0, now - s.episode) : 0;
    $("counters").innerHTML = [
      ["Centre blocked", `${s.blocked} / ${CFG.blockedN}`], ["Left blocked", `${s.left} / ${CFG.blockedN}`], ["Right blocked", `${s.right} / ${CFG.blockedN}`],
      ["Clear frames", `${s.clear} / ${CFG.clearN}`], ["Episode", `${ep.toFixed(1)} / ${CFG.maxEpisode} s`], ["Reversals", `${s.reversals} / ${CFG.maxReversals}`],
    ].map(([k, v]) => `<div><dt>${k}</dt><dd>${v}</dd></div>`).join("");
    $("clock").textContent = `t = ${now.toFixed(1)} s, depth frame ${seq}`;
    $("avoid-reset").classList.toggle("attn", ctl.aborted);
  }

  function step() {
    now = +(now + 0.1).toFixed(1);
    tick++;
    if (tick % 3 === 0) seq++; // about 3 depth frames per second against a 10 Hz loop
    show(ctl.update(grid, seq, +vis.value, +stab.value, now));
  }
  function loop(ts) {
    raf = 0;
    if (!visible || document.hidden) { last = 0; return; }
    if (!last) last = ts;
    while (ts - last >= 100) { step(); last += 100; }
    raf = requestAnimationFrame(loop);
  }
  const kick = () => { if (!raf && visible && !document.hidden) raf = requestAnimationFrame(loop); };
  new IntersectionObserver((es) => { visible = es[0].isIntersecting; kick(); }).observe($("avoid"));
  document.addEventListener("visibilitychange", kick);
  $("avoid-reset").addEventListener("click", () => { ctl.reset(); now = 0; tick = 0; seq = 1; show(ctl.update(grid, seq, +vis.value, +stab.value, now)); });

  paintGrid();
  show(ctl.update(grid, seq, +vis.value, +stab.value, now));
})();
