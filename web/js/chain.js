/*
 * Chain of command. Speeds and rules follow config/reactive_avoidance.json and
 * docs/REACTIVE_AVOIDANCE.md: forward 0.40, cautious 0.16, sidestep 0.25,
 * retreat 0.20 m/s, yaw bound 15 deg/s; SafetySupervisor fails closed.
 */
(() => {
  "use strict";
  const $ = (id) => document.getElementById(id);
  const reduceMotion = window.matchMedia("(prefers-reduced-motion: reduce)").matches;
  const sleep = (ms) => new Promise((r) => setTimeout(r, reduceMotion ? 0 : ms));

  const LAYERS = [
    { id: "nav", name: "LocalGoalNavigator", role: "Proposes bounded motion toward the waypoint" },
    { id: "ap", name: "ActivePerceptionManager", role: "May take over with hover or yaw-only recovery" },
    { id: "avoid", name: "ReactiveAvoidanceController", role: "Rewrites motion from the 3×3 risk grid" },
    { id: "safety", name: "SafetySupervisor", role: "Final gate: freshness, health, collision, limits" },
    { id: "mavsdk", name: "MAVSDK Offboard", role: "Sends VelocityBodyYawspeed" },
    { id: "px4", name: "PX4 autopilot", role: "Stabilises and accepts or rejects commands" },
  ];

  // v = [forward, right, yaw]; act: pass | rewrite | override; note shown on the rung.
  const SIT = [
    {
      id: "clear", label: "Clear corridor",
      text: "Nothing to react to. Every layer passes the navigator’s 0.40 m/s forward command unchanged.",
      steps: [
        { v: [0.4, 0, 0], act: "pass", note: "toward waypoint" },
        { v: [0.4, 0, 0], act: "pass", note: "not needed" },
        { v: [0.4, 0, 0], act: "pass", note: "CLEAR" },
        { v: [0.4, 0, 0], act: "pass", note: "PASS_THROUGH" },
        { v: [0.4, 0, 0], act: "pass", note: "sent" },
        { v: [0.4, 0, 0], act: "pass", note: "Offboard accepted" },
      ],
    },
    {
      id: "obstacle", label: "Obstacle ahead",
      text: "Centre risk stays above the 0.62 stop threshold for two new depth frames and the left sector is clearer. Avoidance stops forward motion and sidesteps left at 0.25 m/s, holding that direction for at least 2 s.",
      steps: [
        { v: [0.4, 0, 0], act: "pass", note: "toward waypoint" },
        { v: [0.4, 0, 0], act: "pass", note: "not needed" },
        { v: [0, -0.25, 0], act: "rewrite", note: "AVOID_LEFT" },
        { v: [0, -0.25, 0], act: "pass", note: "AVOID_LEFT allowed" },
        { v: [0, -0.25, 0], act: "pass", note: "sent" },
        { v: [0, -0.25, 0], act: "pass", note: "Offboard accepted" },
      ],
    },
    {
      id: "dwell", label: "Observing the rock",
      text: "Inside the stand-off window, active perception owns the vehicle: translation goes to zero and only a bounded yaw remains while three fresh depth frames are collected.",
      steps: [
        { v: [0.12, 0, 0], act: "pass", note: "creeping toward goal" },
        { v: [0, 0, 6], act: "rewrite", note: "yaw-only dwell" },
        { v: [0, 0, 6], act: "pass", note: "CAUTION, yaw allowed" },
        { v: [0, 0, 6], act: "pass", note: "within 15°/s bound" },
        { v: [0, 0, 6], act: "pass", note: "sent" },
        { v: [0, 0, 6], act: "pass", note: "Offboard accepted" },
      ],
    },
    {
      id: "visual", label: "Glare washes out the camera",
      text: "Visual health drops below 0.35. Avoidance refuses to move on bad perception and asks for a hover; active perception can then try a yaw-only recovery.",
      steps: [
        { v: [0.4, 0, 0], act: "pass", note: "toward waypoint" },
        { v: [0.4, 0, 0], act: "pass", note: "recovery pending" },
        { v: [0, 0, 0], act: "rewrite", note: "HOVER, low visual health" },
        { v: [0, 0, 0], act: "pass", note: "HOVER" },
        { v: [0, 0, 0], act: "pass", note: "zero velocity sent" },
        { v: [0, 0, 0], act: "pass", note: "holding position" },
      ],
    },
    {
      id: "stale", label: "Depth goes stale",
      text: "Required depth is older than its freshness limit. Every layer above still wants to fly, but the Safety Supervisor issues a zero-velocity hard override and recommends landing. It fails closed.",
      steps: [
        { v: [0.4, 0, 0], act: "pass", note: "toward waypoint" },
        { v: [0.4, 0, 0], act: "pass", note: "not needed" },
        { v: [0.4, 0, 0], act: "pass", note: "uses last grid" },
        { v: [0, 0, 0], act: "override", note: "ABORT, stale perception" },
        { v: [0, 0, 0], act: "pass", note: "hover, then land" },
        { v: [0, 0, 0], act: "pass", note: "lands and disarms" },
      ],
    },
    {
      id: "collision", label: "Collision contact",
      text: "Gazebo reports contact. A collision is never confirmed or debated: the supervisor aborts immediately with zero velocity and the runtime lands.",
      steps: [
        { v: [0.4, 0, 0], act: "pass", note: "toward waypoint" },
        { v: [0.4, 0, 0], act: "pass", note: "not needed" },
        { v: [0.16, 0, 0], act: "rewrite", note: "CAUTION" },
        { v: [0, 0, 0], act: "override", note: "ABORT, collision" },
        { v: [0, 0, 0], act: "pass", note: "hover, then land" },
        { v: [0, 0, 0], act: "pass", note: "lands and disarms" },
      ],
    },
    {
      id: "vlm", label: "VLM says “rock, high confidence”",
      vlm: true,
      text: "The vision-language model reports the target. Its result is fused, validated and written to Ego-Context memory, and that is where it stops. The command chain is untouched: the model cannot complete the observation, move the drone or rescue a failing mission.",
      steps: [
        { v: [0.4, 0, 0], act: "pass", note: "toward waypoint" },
        { v: [0.4, 0, 0], act: "pass", note: "not needed" },
        { v: [0.4, 0, 0], act: "pass", note: "CLEAR" },
        { v: [0.4, 0, 0], act: "pass", note: "VLM not an input" },
        { v: [0.4, 0, 0], act: "pass", note: "sent" },
        { v: [0.4, 0, 0], act: "pass", note: "Offboard accepted" },
      ],
    },
  ];

  const ladder = $("ladder");
  ladder.innerHTML = LAYERS.map((l, i) => `
    <li class="rung" data-i="${i}">
      <span class="rung-idx" aria-hidden="true">${i + 1}</span>
      <span class="rung-name">${l.name}<small>${l.role}</small></span>
      <span class="rung-out">
        <svg class="vec" viewBox="-12 -12 24 24" aria-hidden="true"><circle r="10.5" class="vec-ring"/><g class="vec-g"><line class="vec-line" x1="0" y1="0" x2="0" y2="0"/><path class="vec-head" d="M0 0"/></g><circle r="1.8" class="vec-dot"/></svg>
        <span class="rung-val"></span>
        <span class="rung-note"></span>
      </span>
    </li>`).join("");
  const rungs = [...ladder.querySelectorAll(".rung")];

  const fmt = (v) => {
    const parts = [];
    if (v[0] > 0) parts.push(`forward ${v[0].toFixed(2)} m/s`);
    if (v[0] < 0) parts.push(`back ${(-v[0]).toFixed(2)} m/s`);
    if (v[1] < 0) parts.push(`left ${(-v[1]).toFixed(2)} m/s`);
    if (v[1] > 0) parts.push(`right ${v[1].toFixed(2)} m/s`);
    if (v[2]) parts.push(`yaw ${v[2]}°/s`);
    return parts.length ? parts.join(", ") : "zero velocity";
  };
  function drawVec(svg, v) {
    const [f, r, y] = v;
    const mag = Math.hypot(f, r);
    const line = svg.querySelector(".vec-line"), head = svg.querySelector(".vec-head"), g = svg.querySelector(".vec-g");
    if (mag < 1e-6) {
      line.setAttribute("y2", 0); head.setAttribute("d", "M0 0");
      svg.classList.toggle("yaw", y !== 0);
      return;
    }
    svg.classList.remove("yaw");
    const len = 3 + (mag / 0.4) * 6.5;
    const ang = Math.atan2(r, f) * 180 / Math.PI; // forward is up, right is clockwise
    g.setAttribute("transform", `rotate(${ang})`);
    line.setAttribute("y2", -len);
    head.setAttribute("d", `M-3 ${-len + 3} L0 ${-len} L3 ${-len + 3}`);
  }

  let token = 0;
  async function play(s) {
    const my = ++token;
    $("sit-explain").innerHTML = `<p>${s.text}</p>`;
    rungs.forEach((r) => { r.className = "rung"; r.querySelector(".rung-val").textContent = ""; r.querySelector(".rung-note").textContent = ""; });
    const shadow = $("shadow");
    shadow.classList.remove("on", "blocked");
    shadow.querySelectorAll("li").forEach((li) => li.classList.remove("lit"));
    for (let i = 0; i < rungs.length; i++) {
      if (my !== token) return;
      const st = s.steps[i], r = rungs[i];
      r.classList.add("active");
      await sleep(260);
      if (my !== token) return;
      r.classList.remove("active");
      r.classList.add("done", st.act);
      r.querySelector(".rung-val").textContent = fmt(st.v);
      r.querySelector(".rung-note").textContent = st.note;
      drawVec(r.querySelector(".vec"), st.v);
      if (s.vlm && i === 1) runShadow(my);
      await sleep(180);
    }
  }
  async function runShadow(my) {
    const shadow = $("shadow");
    shadow.classList.add("on");
    for (const li of shadow.querySelectorAll("li")) {
      await sleep(380);
      if (my !== token) return;
      li.classList.add("lit");
    }
    await sleep(300);
    if (my === token) shadow.classList.add("blocked");
  }

  const list = $("sit-list");
  SIT.forEach((s, i) => {
    const b = document.createElement("button");
    b.type = "button";
    b.className = "sit";
    b.setAttribute("aria-pressed", "false");
    b.textContent = s.label;
    b.addEventListener("click", () => {
      list.querySelectorAll(".sit").forEach((x) => x.setAttribute("aria-pressed", String(x === b)));
      play(s);
    });
    list.appendChild(b);
    if (i === 0) b.setAttribute("aria-pressed", "true");
  });

  // Show the first situation fully drawn, without animating, until the visitor picks one.
  (function initial() {
    const s = SIT[0];
    $("sit-explain").innerHTML = `<p>${s.text}</p>`;
    rungs.forEach((r, i) => {
      const st = s.steps[i];
      r.className = `rung done ${st.act}`;
      r.querySelector(".rung-val").textContent = fmt(st.v);
      r.querySelector(".rung-note").textContent = st.note;
      drawVec(r.querySelector(".vec"), st.v);
    });
  })();
})();
