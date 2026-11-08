// Builds docs/slides/results_slides.pptx: the "Current results" and "Conclusion and future
// plan" slides for the course deck, in its style (Arial, yellow-highlighted titles).
// Numbers: docs/figures/benchmark_unified.json (RMSE after the first lap, mm; size %).
//
// Usage (pptxgenjs is not a project dependency; install it anywhere and point NODE_PATH at it):
//   npm install --prefix /some/dir pptxgenjs@3.12.0
//   NODE_PATH=/some/dir/node_modules node docs/slides/results_slides.js
const path = require("path");
const fs = require("fs");
const pptxgen = require("pptxgenjs");

const pres = new pptxgen();
pres.layout = "LAYOUT_16x9"; // 10 x 5.625 in
pres.title = "Soft arm: results and plan";

const INK = "1A1A1A";
const MUTED = "5F6368";
const HIGHLIGHT = "FFC94D";
const HEAD = "37474F";
const RULE = "C9CED2";
const BEST = "FFF3CC";
const FONT = "Arial";

function title(slide, text) {
  slide.background = { color: "FFFFFF" };
  slide.addText(text, { x: 0.4, y: 0.3, w: 0.27 * text.length + 0.3, h: 0.62, fontFace: FONT, fontSize: 30, bold: true,
    color: INK, fill: { color: HIGHLIGHT }, margin: [0, 6, 0, 6], valign: "middle", isTextBox: true });
}

// ---------------------------------------------------------------- data from the benchmark
const d = JSON.parse(fs.readFileSync(path.join(__dirname, "..", "figures", "benchmark_unified.json"), "utf8"));
const M = {};
for (const r of d.runs) M[r.short + "|" + r.test] = r.metrics;
const TESTS = ["scored circle", "held-out circle", "MT triangle"];
const ROWS = [
  ["PD", "PD corrected"],
  ["SMC, one-mode (Nikoletta)", "Notebook SMC λ=8"],
  ["Paper NDOSMC (M1)", "Paper NDOSMC"],
  ["Delay-aware SMC, Taylor preview (M2)", "Delay-aware SMC (Taylor preview)"],
  ["MPC, learned reference", "MPC (predicted reference)"],
  ["MPC, reference known in advance*", "MPC, true future ref [diag]"],
];
const rmse = (key, t) => M[key + "|" + t].planar_rmse_after_lap_mm;
const rad = (key) => [M[key + "|scored circle"].radius_min_after_lap_mm, M[key + "|scored circle"].radius_max_after_lap_mm];
const best = TESTS.map((t) => Math.min(...ROWS.slice(0, 5).map(([, k]) => Number(rmse(k, t).toFixed(2)))));
const radErr = (k) => Math.max(...rad(k).map((v) => Math.abs(Number(v.toFixed(1)) - 7.0)));
const bestRad = Math.min(...ROWS.slice(0, 5).map(([, k]) => radErr(k)));

// ---------------------------------------------------------------- slide 1: results
let s = pres.addSlide();
title(s, "Current results");
const th = (t) => ({ text: t, options: { bold: true, color: "FFFFFF", fill: { color: HEAD }, align: "center", valign: "middle" } });
const rows = [[th("Controller"), th("Scored circle"), th("Held-out circle"), th("Triangle"), th("Circle radius (7.0 mm)")]];
ROWS.forEach(([label, key], i) => {
  const ref = i === 5;
  const base = { color: ref ? MUTED : INK, italic: ref, valign: "middle" };
  const row = [{ text: label, options: { ...base, align: "left" } }];
  TESTS.forEach((t, j) => {
    const v = Number(rmse(key, t).toFixed(2));
    const isBest = !ref && v === best[j];
    row.push({ text: v.toFixed(2), options: { ...base, align: "center", bold: isBest, fill: isBest ? { color: BEST } : undefined } });
  });
  const [r0, r1] = rad(key);
  const rBest = !ref && radErr(key) === bestRad;
  row.push({ text: `${r0.toFixed(1)}–${r1.toFixed(1)}`, options: { ...base, align: "center", bold: rBest, fill: rBest ? { color: BEST } : undefined } });
  rows.push(row);
});
s.addTable(rows, { x: 0.4, y: 1.15, w: 6.55, colW: [2.55, 1.0, 1.0, 1.0, 1.0], fontFace: FONT, fontSize: 10.5,
  border: { type: "solid", pt: 0.75, color: RULE }, rowH: [0.42, 0.36, 0.36, 0.36, 0.36, 0.36, 0.36], margin: [2, 4, 2, 4] });
s.addText([
  { text: "RMSE after the first lap, mm: the error once the start-up jump is over (steady tracking). Bold: best per column.", options: { breakLine: true } },
  { text: `The harness reports RMSE over the whole run, which includes the first seconds where the 7 mm circle starts 7 mm from the arm: PD ${M["PD corrected|scored circle"].planar_rmse_mm.toFixed(2)} vs ${M["PD corrected|scored circle"].planar_rmse_after_lap_mm.toFixed(2)}, delay-aware SMC ${M["Delay-aware SMC (Taylor preview)|scored circle"].planar_rmse_mm.toFixed(2)} vs ${M["Delay-aware SMC (Taylor preview)|scored circle"].planar_rmse_after_lap_mm.toFixed(2)}, MPC ${M["MPC (predicted reference)|scored circle"].planar_rmse_mm.toFixed(2)} vs ${M["MPC (predicted reference)|scored circle"].planar_rmse_after_lap_mm.toFixed(2)} mm (whole run vs after lap 1).`, options: { breakLine: true } },
  { text: "Circles: 7 mm at 0.1 Hz (scored), 10 mm at 0.15 Hz (held-out, not used for tuning). Triangle: 7 mm, 10 s per lap. Circle radius: smallest–largest distance of the traced path from its centre, scored circle (reference 7.0 mm; a wide range means the path is not a circle). Simulation, seed 0.", options: { breakLine: true } },
  { text: "* Reference case: the controller is given the future path, which the evaluation does not provide." },
], { x: 0.4, y: 3.85, w: 6.55, h: 1.3, fontFace: FONT, fontSize: 8.5, color: MUTED, valign: "top", margin: 0, isTextBox: true });

s.addText([
  { text: "Prediction matters", options: { bold: true, fontSize: 14, breakLine: true, paraSpaceAfter: 6 } },
  { text: "Commands reach the arm 0.5 s late (+ 0.6 s lag), so the controller must aim where the path will be.", options: { fontSize: 10.5, color: MUTED, breakLine: true, paraSpaceAfter: 6 } },
  { text: "Path known in advance: MPC best", options: { bullet: true, breakLine: true } },
  { text: "Unknown, repeating: MPC learns it after one lap", options: { bullet: true, breakLine: true } },
  { text: "Unknown, non-repeating: Taylor-preview SMC best, but overshoots fast or cornered paths", options: { bullet: true, breakLine: true } },
  { text: "PD, one-mode SMC: no overshoot, but late and small", options: { bullet: true } },
], { x: 7.15, y: 1.15, w: 2.5, h: 2.6, fontFace: FONT, fontSize: 11, color: INK, valign: "top", margin: 0, paraSpaceAfter: 4, isTextBox: true });

// ---------------------------------------------------------------- slide: model check
const MCJ = JSON.parse(fs.readFileSync(path.join(__dirname, "..", "figures", "model_comparison.json"), "utf8"));
const MC = MCJ.results;
const G = Object.fromEntries(Object.entries(MCJ.static_gains_mm_per_psi["x per psi on S3"]).map(([k, v]) => [k, Math.abs(v)]));
s = pres.addSlide();
title(s, "Model check: same excitation");
const MODELS = [["One-mode model (Nikoletta)", "one-mode"], ["One-mode + gravity stiffness", "one-mode+g"],
  ["M2 (task-space, identified)", "M2"], ["M3 (M2 + inertia)", "M3"], ["F (frequency-identified)", "F"]];
const COLS = [["random_steps", "x"], ["random_steps", "y"], ["random_steps", "z"], ["chirp", "x"], ["chirp", "y"], ["chirp", "z"]];
const mcBest = COLS.map(([e, a]) => Math.min(...MODELS.map(([, m]) => (MC[e][m][a] ? Number(MC[e][m][a].rmse_mm.toFixed(2)) : Infinity))));
const mcRows = [[th("Model"), th("Steps x"), th("Steps y"), th("Steps z"), th("Chirp x"), th("Chirp y"), th("Chirp z")]];
for (const [label, m] of MODELS) {
  const row = [{ text: label, options: { align: "left", valign: "middle", color: INK } }];
  COLS.forEach(([e, a], j) => {
    const v = MC[e][m][a];
    if (!v) { row.push({ text: "no z", options: { align: "center", valign: "middle", color: MUTED, italic: true } }); return; }
    const r = Number(v.rmse_mm.toFixed(2)), isBest = r === mcBest[j];
    row.push({ text: r.toFixed(2), options: { align: "center", valign: "middle", color: INK, bold: isBest, fill: isBest ? { color: BEST } : undefined } });
  });
  mcRows.push(row);
}
s.addTable(mcRows, { x: 0.4, y: 1.2, w: 9.2, colW: [3.2, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0], fontFace: FONT, fontSize: 12,
  border: { type: "solid", pt: 0.75, color: RULE }, rowH: 0.38, margin: [2, 4, 2, 4] });
s.addText([
  { text: "Free-run tip error, RMSE in mm: each model gets the same commands as the MuJoCo simulator and runs open loop from rest. 60 s each; random steps (1.5–7.5 psi, held 0.5–3 s) and a phased chirp (0.02 → 1 Hz). Bold: best per column.", options: { breakLine: true } },
  { text: `Static gain (mm/psi, +1 psi on S3): simulator ${G.truth.toFixed(2)}, one-mode ${G["one-mode"].toFixed(2)}, + gravity ${G["one-mode+g"].toFixed(2)}, M2 ${G.M2.toFixed(2)}, M3 ${G.M3.toFixed(2)}, F ${G.F.toFixed(2)}. The one-mode models have no 0.5 s dead time or 0.6 s lag; gravity stiffness fixes most of the gain error but not the timing. M2, M3 and F include both and follow the simulator to about 0.1 mm.` },
], { x: 0.4, y: 3.65, w: 9.2, h: 1.6, fontFace: FONT, fontSize: 11, color: MUTED, valign: "top", margin: 0, isTextBox: true });

// ---------------------------------------------------------------- slide 2: conclusion
s = pres.addSlide();
title(s, "Conclusion and future plan");
const col = (x, head, items) => s.addText([
  { text: head, options: { bold: true, fontSize: 16, breakLine: true, paraSpaceAfter: 8 } },
  ...items.map((t, i) => ({ text: t, options: { bullet: true, breakLine: i < items.length - 1, paraSpaceAfter: 10 } })),
], { x, y: 1.25, w: 4.4, h: 3.6, fontFace: FONT, fontSize: 13, color: INK, valign: "top", margin: 0, isTextBox: true });
const r2 = (k, t) => rmse(k, t).toFixed(2);
col(0.4, "What we learned", [
  `Modelling the 0.5 s delay and 0.6 s lag gives the largest gain: scored circle ${r2("PD corrected", "scored circle")} mm (PD) → ${r2("Delay-aware SMC (Taylor preview)", "scored circle")} mm (delay-aware SMC).`,
  `With the delay handled, predicting the reference decides fast and cornered paths: triangle ${r2("Delay-aware SMC (Taylor preview)", "MT triangle")} (Taylor) / ${r2("MPC (predicted reference)", "MT triangle")} (learned) / ${r2("MPC, true future ref [diag]", "MT triangle")} mm (known).`,
  `No single best controller: known or repeating path → MPC; non-repeating → Taylor-preview SMC (${r2("Delay-aware SMC (Taylor preview)", "recorded circular")} vs ${r2("MPC (predicted reference)", "recorded circular")} mm).`,
]);
col(5.25, "Next steps", [
  "Ask: does the evaluation give the controller the future path, or only the current point?",
  "Reference prediction that works for repeating and non-repeating paths without overshooting corners.",
  "Constrained MPC / MPPI (0–9 psi inside the optimisation); test with a wrong delay estimate.",
]);
s.addText("All results in simulation (make_calibrated_sim); not yet run on the robot.", { x: 0.4, y: 5.0, w: 9.2, h: 0.3,
  fontFace: FONT, fontSize: 9, color: MUTED, margin: 0, isTextBox: true });

pres.writeFile({ fileName: path.join(__dirname, "results_slides.pptx") }).then((f) => console.log("wrote", f));
