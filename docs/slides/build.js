// Builds docs/slides/soft_arm_models_controllers.pptx, the short deck for the
// Nikoletta + Ibraheem discussion (6 slides). Numbers on slide 2 come from
// Nikoletta's slides; the rest from scripts/task_space_model.py,
// scripts/stiffness_vs_pressure.py, scripts/compare_controllers.py and
// scripts/benchmark_unified.py (slide 6).
//
// Usage (pptxgenjs is not a project dependency; install it anywhere and point NODE_PATH at it):
//   npm install --prefix /some/dir pptxgenjs@3.12.0
//   NODE_PATH=/some/dir/node_modules node docs/slides/build.js
const path = require("path");
const pptxgen = require("pptxgenjs");

const pres = new pptxgen();
pres.layout = "LAYOUT_16x9"; // 10 x 5.625 in
pres.title = "Soft arm: models and controllers";

const INK = "1F2A30";
const MUTED = "5D676C";
const ACCENT = "1F6F7A";
const TINT = "E6F1F2";
const BODY = "Calibri";
const MATH = "Cambria";

function header(slide, title, tag) {
  slide.background = { color: "FFFFFF" };
  slide.addText(title, { x: 0.5, y: 0.3, w: 9.0, h: 0.6, fontFace: BODY, fontSize: 28, bold: true, color: INK, margin: 0, isTextBox: true });
  slide.addText(tag, { x: 0.5, y: 0.9, w: 9.0, h: 0.3, fontFace: BODY, fontSize: 12, color: ACCENT, bold: true, margin: 0, isTextBox: true });
}

function bullets(slide, items, opts) {
  slide.addText(
    items.map((t, i) => ({ text: t, options: { bullet: true, breakLine: i < items.length - 1, paraSpaceAfter: 6 } })),
    { x: opts.x, y: opts.y, w: opts.w, h: opts.h, fontFace: BODY, fontSize: 14, color: INK, valign: "top", margin: 0.05, isTextBox: true }
  );
}

function eqCard(slide, lines, opts) {
  const fs = opts.fontSize || 14;
  slide.addShape(pres.shapes.ROUNDED_RECTANGLE, { x: opts.x, y: opts.y, w: opts.w, h: opts.h, fill: { color: TINT }, line: { color: TINT }, rectRadius: 0.08 });
  slide.addText(
    lines.map((t, i) => ({ text: t, options: { breakLine: i < lines.length - 1, paraSpaceAfter: opts.space ?? 8, bold: i === 0 && opts.boldFirst } })),
    { x: opts.x + 0.2, y: opts.y + 0.12, w: opts.w - 0.4, h: opts.h - 0.24, fontFace: opts.face || MATH, fontSize: fs, color: INK, valign: "top", margin: 0, isTextBox: true }
  );
}

// 1 · Nikoletta: model
let s = pres.addSlide();
header(s, "Model: one bending mode of the chain", "NIKOLETTA · MODEL");
bullets(s, [
  "Start from the simulator's 5-link chain, one bending plane (mass matrix from MuJoCo, joint springs and dampers)",
  "Modal analysis gives 5 modes (0.52, 3.35, 8.93, 15.7, 21.5 Hz); keep the first",
  "Project it onto the tip: one second-order equation per axis, same constants for x and y",
  "Pressure assumed to act instantly; gravity not in K",
], { x: 0.5, y: 1.45, w: 5.0, h: 3.6 });
eqCard(s, [
  "M θ̈ + D θ̇ + K θ = τ",
  "K v = ω² M v",
  "ÿ + 9.23 ẏ + 10.60 y = b Δp",
  "|b| = 0.082",
], { x: 5.9, y: 1.45, w: 3.6, h: 2.3 });

// 2 · Nikoletta: controller
s = pres.addSlide();
header(s, "Controller: SMC with a Smith predictor", "NIKOLETTA · CONTROLLER");
bullets(s, [
  "Sliding-mode control on the one-mode model, λ = 12, K = 0.2",
  "Fixing the allocator rotation and the sign of b: 4.37 → 2.31 mm",
  "Disturbance observer (low-pass, τ = 2.5 s): 2.30 mm",
  "Smith predictor with the target advanced by the delay: 1.97 mm (best)",
], { x: 0.5, y: 1.45, w: 5.0, h: 3.6 });
eqCard(s, [
  "s = ė + λ e",
  "Δp = (1/b)[k y + c ẏ + ÿ_d − λ ė − K sat(s/φ)]",
], { x: 5.9, y: 1.45, w: 3.6, h: 1.7 });
s.addText("Numbers from Nikoletta's slides.", { x: 0.5, y: 5.0, w: 9.0, h: 0.3, fontFace: BODY, fontSize: 10, color: MUTED, margin: 0, isTextBox: true });

// 3 · Ibraheem: model
s = pres.addSlide();
header(s, "Model: identified from sensor data (M2)", "IBRAHEEM · MODEL");
bullets(s, [
  "Only hardware signals: tip from motion capture (+0.1 mm noise) and pressure sensors; no simulator parameters",
  "Command waits T, pressure creeps in with lag τ, arm follows: u → p → y",
  "Fitted by output-error least squares on 150 s of random pressure steps",
  "Free-run error on unseen runs: 0.12 mm (noise floor 0.10 mm)",
], { x: 0.5, y: 1.45, w: 5.0, h: 3.6 });
eqCard(s, [
  "τ ṗ = −p + G u(t − T)",
  "ẏ = −Λ y + Λ H p",
  "T = 0.50 s,  τ = 0.60 s",
  "G = diag(1.12, 1.10, 1.16)",
  "Λ = diag(4.77, 4.70, 15.5) s⁻¹",
], { x: 5.9, y: 1.45, w: 3.6, h: 2.8 });

// 4 · Ibraheem: controller
s = pres.addSlide();
header(s, "Controller: delay-aware SMC", "IBRAHEEM · CONTROLLER");
bullets(s, [
  "Smith predictor: model forecast plus the measured model error gives the tip 0.5 s ahead",
  "Reference previewed by the same 0.5 s",
  "SMC law on M2 gives the pressure needed, λ = 8",
  "Lag inverse turns it into the command, clipped to 0–9 psi",
  "Paper's disturbance observer optional: ≤ 0.03 mm",
], { x: 0.5, y: 1.45, w: 4.8, h: 3.6 });
eqCard(s, [
  "ỹ = ŷ(t) + [y(t) − ŷ(t − T)]",
  "σ = ỹ − r̃",
  "p_des = H⁻¹Λ⁻¹[ṙ̃ + Λ ỹ − λ σ − η sat(σ/μ)]",
  "u = G⁻¹(p_des + τ ṗ_des)",
], { x: 5.6, y: 1.45, w: 3.9, h: 2.4, fontSize: 12.5 });

// 5 · Ibraheem: loop diagram, and why no observer is needed
s = pres.addSlide();
header(s, "Delay-aware SMC on M2: the loop", "IBRAHEEM · CONTROLLER");
function blk(x, y, w, h, title, sub, fill) {
  s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x, y, w, h, fill: { color: fill || TINT }, line: { color: ACCENT, width: 1 }, rectRadius: 0.06 });
  s.addText([
    { text: title, options: { bold: true, fontSize: 10.5, fontFace: BODY, breakLine: true } },
    { text: sub, options: { fontSize: 9.5, fontFace: MATH } },
  ], { x, y, w, h, color: INK, align: "center", valign: "middle", margin: 0.03, isTextBox: true });
}
function line(x1, y1, x2, y2, head) {
  s.addShape(pres.shapes.LINE, {
    x: Math.min(x1, x2), y: Math.min(y1, y2), w: Math.abs(x2 - x1), h: Math.abs(y2 - y1),
    flipH: x2 < x1, flipV: y2 < y1, line: { color: INK, width: 1, ...(head ? { endArrowType: "triangle" } : {}) },
  });
}
function label(t, x, y) {
  s.addText(t, { x: x - 0.35, y, w: 0.7, h: 0.22, fontFace: MATH, fontSize: 10.5, color: INK, align: "center", margin: 0, isTextBox: true });
}
const fy = 1.55, fh = 0.75, fm = fy + fh / 2;
label("r", 0.4, fm - 0.11);
line(0.55, fm, 0.7, fm, true);
blk(0.7, fy, 1.1, fh, "Preview", "r + Tṙ + ½T²r̈");
line(1.8, fm, 2.1, fm, true); label("r̃", 1.95, fm - 0.3);
blk(2.1, fy, 0.95, fh, "Error", "σ = ỹ − r̃");
line(3.05, fm, 3.3, fm, true);
blk(3.3, fy, 1.1, fh, "SMC law on M2", "λ = 8, η = 1, μ = 1");
line(4.4, fm, 4.9, fm, true); label("p_des", 4.65, fm - 0.3);
blk(4.9, fy, 1.35, fh, "Lag inverse", "G⁻¹(p_des + τṗ_des)");
line(6.25, fm, 6.45, fm, true);
blk(6.45, fy, 0.65, fh, "Clip", "0–9 psi");
line(7.1, fm, 7.45, fm, true); label("u", 7.3, fm - 0.3);
blk(7.45, fy, 1.45, fh, "Arm", "T = 0.5 s, τ = 0.6 s", "FFFFFF");
line(8.9, fm, 9.6, fm, true); label("y", 9.4, fm - 0.3);
const py = 2.75, ph = 0.75, pm = py + ph / 2;
blk(3.3, py, 3.8, ph, "Smith predictor: M2 with no delay, driven by u", "ỹ_k = ŷ_k + (y_k − ŷ_{k−50}),  50 steps = 0.5 s");
line(7.3, fm, 7.3, pm); line(7.3, pm, 7.1, pm, true);                 // u into predictor
line(9.3, fm, 9.3, 3.75); line(9.3, 3.75, 5.2, 3.75); line(5.2, 3.75, 5.2, py + ph, true); // y into predictor
line(3.3, pm, 2.575, pm); line(2.575, pm, 2.575, fy + fh, true); label("ỹ", 2.35, pm - 0.3); // ỹ to error
eqCard(s, [
  "Why it works without a disturbance observer",
  "• Inertia: the arm is heavily damped (ζ ≈ 0.75 at 1.2 Hz); a model that keeps it (M3) fits no better (0.12 mm)",
  "• M2 free-run error 0.12 mm against 0.10 mm sensor noise: little model error is left",
  "• The predictor adds the measured model error y_k − ŷ_{k−50} at every step",
  "• The switching term η sat(σ/μ) covers what remains; measured: adding the observer changes RMSE by ≤ 0.03 mm",
], { x: 0.5, y: 4.0, w: 9.0, h: 1.45, fontSize: 11, face: BODY, space: 3, boldFirst: true });

// 6 · Results: every controller on the same tests (scripts/benchmark_unified.py)
s = pres.addSlide();
header(s, "Results: same tests, same metric", "RMSE AFTER THE FIRST LAP, mm · make_calibrated_sim, seed 0 · bold = best");
const hdr = (t) => ({ text: t, options: { bold: true, color: "FFFFFF", fill: { color: ACCENT } } });
s.addTable([
  [hdr("Controller"), hdr("Scored circle\n0.1 Hz"), hdr("Triangle\n10 s lap"), hdr("Axial\n0.1 Hz"), hdr("Fast circle\n0.4 Hz"), hdr("Robot\ncircle")],
  ["PD", "3.96", "2.94", "2.77", "7.48", "4.75"],
  ["SMC, one-mode (Nikoletta)", "2.13", "1.81", "1.48", "5.71", "2.95"],
  ["Paper SMC (M1, no observer)", "0.87", "1.46", "0.61", "8.18", "2.62"],
  ["Delay-aware SMC (M2)", "0.09", "1.46", "0.07", "2.35", { text: "1.56", options: { bold: true } }],
  ["Delay-aware SMC + predictor", { text: "0.03", options: { bold: true } }, "1.78", "0.03", "0.55", "2.07"],
  ["Delay-compensated MPC", "0.06", { text: "0.62", options: { bold: true } }, { text: "0.01", options: { bold: true } }, { text: "0.15", options: { bold: true } }, "3.56"],
], { x: 0.4, y: 1.35, w: 9.2, colW: [3.2, 1.2, 1.2, 1.2, 1.2, 1.2], fontFace: BODY, fontSize: 12, color: INK, align: "center", valign: "middle",
     border: { type: "solid", pt: 0.5, color: "D0D6D8" }, rowH: 0.33 });
s.addText("Modelling the 0.5 s delay and 0.6 s lag is what matters; predicting the reference 0.5 s ahead decides fast and cornered motions. On the recorded robot path the reference never repeats, so the history-based predictor falls back to AR(2) and the plain delay-aware SMC is best.", { x: 0.4, y: 4.6, w: 9.2, h: 0.75, fontFace: BODY, fontSize: 11, color: MUTED, margin: 0, isTextBox: true });

pres.writeFile({ fileName: path.join(__dirname, "soft_arm_models_controllers.pptx") }).then((f) => console.log("wrote", f));
