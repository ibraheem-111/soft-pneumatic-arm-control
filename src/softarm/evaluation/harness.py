"""The tracking loop: evaluate_controller's loop for any reference, and thin harness wrappers."""

import numpy as np

from soft_robotic_arm import make_calibrated_sim
from soft_robotic_arm.evaluation import CircularTrackingTask, TrackingResult, evaluate_controller, make_reference

from softarm.constants import BASE_PSI, HZ, LIMIT_PSI, SETTLE_S

SIM_SEED = 0


def track(controller, ref_fn, n_steps: int, seed: int = SIM_SEED) -> dict:
    """evaluate_controller's loop with an arbitrary reference ref_fn(home, k) -> (3,) m.

    Settles SETTLE_S at BASE_PSI on make_calibrated_sim(control_hz=HZ, seed), takes the settled tip
    as home, then steps n_steps; commands are clipped to 0-9 psi. Returns tips, refs (absolute m),
    cmds (psi) and home."""
    sim = make_calibrated_sim(control_hz=int(HZ), seed=seed)
    limit = min(LIMIT_PSI, float(sim.cfg.p_max))
    baseline = np.full(3, BASE_PSI)
    obs = sim.reset()
    for _ in range(int(round(SETTLE_S * HZ))):
        obs = sim.step(baseline)
    home = np.asarray(obs["tip_pos"], float).copy()
    tips = np.empty((n_steps, 3))
    refs = np.empty((n_steps, 3))
    cmds = np.empty((n_steps, 3))
    for k in range(n_steps):
        ref = ref_fn(home, k)
        cmd = np.asarray(controller.compute(float(obs["time"]), obs, ref), float)
        if cmd.shape != (3,) or not np.all(np.isfinite(cmd)):
            raise ValueError("controller must return 3 finite S2-S4 setpoints")
        cmd = np.clip(cmd, 0.0, limit)
        obs = sim.step(cmd)
        tips[k], refs[k], cmds[k] = obs["tip_pos"], ref, cmd
    sim.close()
    return dict(tips=tips, refs=refs, cmds=cmds, home=home)


def track_ref_mm(controller, ref_mm: np.ndarray, n_steps: int, seed: int = SIM_SEED) -> dict:
    """track() with a reference given as mm from the settled tip, sampled at the control rate."""
    return track(controller, lambda home, k: home + ref_mm[k] / 1000.0, n_steps, seed=seed)


def evaluate(controller, task: CircularTrackingTask | None = None, seed: int = 0) -> TrackingResult:
    """The package harness (evaluate_controller) on make_calibrated_sim(seed); default task: the scored one."""
    task = task or CircularTrackingTask()
    sim = make_calibrated_sim(control_hz=task.control_hz, seed=seed)
    try:
        return evaluate_controller(controller, task=task, sim=sim)
    finally:
        sim.close()


def settled_home(task: CircularTrackingTask, seed: int) -> np.ndarray:
    """The settled tip the harness centres the task on, reproduced with an identical settle."""
    sim = make_calibrated_sim(control_hz=task.control_hz, seed=seed)
    obs = sim.reset()
    for _ in range(task.settle_steps):
        obs = sim.step(task.baseline_command(LIMIT_PSI))
    home = np.asarray(obs["tip_pos"], dtype=float).copy()
    sim.close()
    return home


def oracle_for(task: CircularTrackingTask, seed: int):
    """True reference of the task as a function of the sample index: oracle(n) -> absolute m."""
    home = settled_home(task, seed)
    return lambda n: make_reference(home, n / task.control_hz, task)
