/*
 * Observation gates from config/missions/polaris_demo.json (rock-observation),
 * starting at the values recorded in the retained successful mission.
 */
(() => {
  "use strict";
  const $ = (id) => document.getElementById(id);

  const GATES = [
    { id: "dist", name: "Distance to target", unit: "m", min: 1.5, max: 4, step: 0.01, v: 2.93, ok: (x) => x >= 2.5 && x <= 3.0, rule: "2.5 to 3.0 m", geo: true },
    { id: "head", name: "Heading error", unit: "°", min: -30, max: 30, step: 0.1, v: -3.86, ok: (x) => Math.abs(x) <= 12, rule: "within 12°", geo: true },
    { id: "vis", name: "Visual health", unit: "", min: 0, max: 1, step: 0.01, v: 0.84, ok: (x) => x >= 0.35, rule: "at least 0.35" },
    { id: "dh", name: "Depth health", unit: "", min: 0, max: 1, step: 0.01, v: 1.0, ok: (x) => x >= 0.4, rule: "at least 0.40" },
    { id: "stab", name: "Depth stability", unit: "", min: 0, max: 1, step: 0.01, v: 0.98, ok: (x) => x >= 0.75, rule: "at least 0.75" },
    { id: "risk", name: "Centre risk", unit: "", min: 0, max: 1, step: 0.01, v: 0.16, ok: (x) => x <= 0.45, rule: "at most 0.45" },
    { id: "speed", name: "Horizontal speed", unit: "m/s", min: 0, max: 0.6, step: 0.01, v: 0.06, ok: (x) => x <= 0.2, rule: "at most 0.20 m/s" },
    { id: "dwell", name: "Stable dwell", unit: "s", min: 0, max: 2, step: 0.01, v: 0.67, ok: (x) => x >= 0.6, rule: "at least 0.6 s", timing: true },
  ];
  const vals = Object.fromEntries(GATES.map((g) => [g.id, g.v]));
  let fresh = 3, lastFrame = 3;

  const gatesEl = $("gates");
  gatesEl.innerHTML = GATES.map((g) => `
    <div class="gate" data-g="${g.id}">
      <label for="g-${g.id}"><span class="gate-dot" aria-hidden="true"></span>${g.name}<span class="gate-rule">${g.rule}</span></label>
      <input type="range" id="g-${g.id}" min="${g.min}" max="${g.max}" step="${g.step}" value="${g.v}">
      <output for="g-${g.id}" id="o-${g.id}"></output>
    </div>`).join("");

  const fmtVal = (g, x) => (g.unit === "°" ? `${x.toFixed(1)}°` : g.unit === "m" ? `${x.toFixed(2)} m` : g.unit === "m/s" ? `${x.toFixed(2)} m/s` : g.unit === "s" ? `${x.toFixed(2)} s` : x.toFixed(2));

  function evaluate() {
    const fails = GATES.filter((g) => !g.ok(vals[g.id]));
    GATES.forEach((g) => {
      const row = gatesEl.querySelector(`[data-g="${g.id}"]`);
      row.dataset.ok = String(g.ok(vals[g.id]));
      $(`o-${g.id}`).textContent = fmtVal(g, vals[g.id]);
    });
    const geoFail = fails.filter((g) => g.geo);
    const qualityFail = fails.filter((g) => !g.geo && !g.timing);
    const dwellFail = fails.some((g) => g.timing);
    let state, why;
    if (geoFail.length) { state = "APPROACHING"; why = `Not in the stand-off window: ${geoFail.map((g) => g.name.toLowerCase()).join(", ")}.`; }
    else if (qualityFail.length) { state = "OBSERVATION_CANDIDATE"; why = `In position, but the dwell resets: ${qualityFail.map((g) => g.name.toLowerCase()).join(", ")} out of bounds.`; }
    else if (dwellFail || fresh < 3) { state = "OBSERVING"; why = dwellFail && fresh < 3 ? "Dwell too short and not enough fresh frames yet." : dwellFail ? "Every gate holds; waiting for 0.6 s of stable dwell." : `Every gate holds; ${3 - fresh} more fresh depth frame${3 - fresh === 1 ? "" : "s"} needed.`; }
    else { state = "OBSERVATION_COMPLETE"; why = "Every gate held on fresh evidence. Accepted once the command is transmitted with no hard override."; }
    const st = $("obs-state");
    if (st.textContent !== state) { st.classList.remove("pop"); void st.offsetWidth; st.classList.add("pop"); }
    st.textContent = state;
    st.dataset.state = state;
    $("obs-why").textContent = why;

    // stand-off drawing: drone sits west of the target at the chosen distance, nose rotated by heading error
    const d = vals.dist, h = vals.head;
    $("so-drone").setAttribute("transform", `translate(${-Math.min(d, 4.6)} 0) rotate(${h}) rotate(180)`);
    $("standoff").dataset.state = state;

    $("fresh-count").textContent = String(Math.min(fresh, 3));
    $("fresh-dots").innerHTML = [0, 1, 2].map((i) => `<i class="${i < fresh ? "on" : ""}"></i>`).join("");
  }

  GATES.forEach((g) => {
    $(`g-${g.id}`).addEventListener("input", (e) => { vals[g.id] = +e.target.value; evaluate(); });
  });
  const note = $("fresh-note");
  $("fresh-new").addEventListener("click", () => { lastFrame++; fresh = Math.min(fresh + 1, 3); note.textContent = `Frame #${lastFrame}: a new source frame with a later inference timestamp. Counted.`; note.className = "fresh-note good"; evaluate(); });
  $("fresh-dup").addEventListener("click", () => { note.textContent = `Frame #${lastFrame} again: same source sequence, so it is rejected as a duplicate.`; note.className = "fresh-note bad"; note.animate?.([{ transform: "translateX(-4px)" }, { transform: "translateX(4px)" }, { transform: "none" }], { duration: 260 }); evaluate(); });
  $("fresh-reset").addEventListener("click", () => { fresh = 0; note.textContent = "Evidence cleared. Collect three fresh frames."; note.className = "fresh-note"; evaluate(); });

  evaluate();
})();
