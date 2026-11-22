"""Integrated component pointing test for mrpRotation.

A guidance module (Earth Point, Hill Point or Inertial Point) feeds mrpRotation in one task. There are no spacecraft
dynamics: the test checks the reference attitude that the chain commands, and where it puts the selected component.

Each point state runs four scenarios on its component:

1. Component only: the component is misaligned from its nominal axis. No offset, no spin.
2. Offset only: the component is aligned, and the offset moves its boresight off the target.
3. Component and offset together.
4. Component and offset, then a spin about the boresight.

A last test compares three spin axes with the same offset, in Hill Point with the MAC.

The angles are exaggerated so the figures show them: 2 deg misalignment (the real components are within about
0.5 deg) and a 6 deg offset. Run this file directly to save the figures:

    python test_mrpRotation_componentPointing.py [output_directory]
"""

import os
import sys
from dataclasses import dataclass
from typing import Callable, Optional

import matplotlib

if __name__ == "__main__":
    matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pytest
from xmera.architecture import messaging
from xmera.fp32 import celestialTwoBodyPointF32, hillPointF32, inertial3DF32, mrpRotationF32
from xmera.utilities import RigidBodyKinematics as rbk
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros

# Forward Euler makes the spin angle lag the commanded rate by about sigma * Omega * DT / 4, so the step keeps
# Omega * DT small enough for the rate check.
DT = 0.5  # [s] task period and mrpRotation controlPeriod
MISALIGNMENT = np.radians(2.0)  # [rad] exaggerated
OFFSET = np.radians(6.0)  # [rad] about the component x axis
SPIN_RATE = np.radians(1.0)  # [rad/s]
SPIN_TIME = 360.0  # [s] one full turn at SPIN_RATE
STATIC_TIME = 2.0  # [s] three samples, enough for the rate check

ANGLE_TOL = 2e-5  # [rad] float32 attitude chain, one step
SPIN_ANGLE_TOL = 1e-4  # [rad] float32 attitude chain over a full turn of forward-Euler spin
RATE_TOL = 1e-4  # [rad/s] central-difference check of the output rate

# Plot roles: the first three slots of the reference categorical palette, validated for all pairs.
COLOR_BORESIGHT = "#2a78d6"
COLOR_NOMINAL = "#eb6834"
COLOR_ROLL = "#1baf7a"
COLOR_INK = "#0b0b0b"
COLOR_MUTED = "#52514e"
COLOR_GRID = "#e4e3df"

E1, E2, E3 = np.eye(3)


def unit(v):
    v = np.asarray(v, dtype=float)
    return v / np.linalg.norm(v)


def mrp_from_axis_angle(axis, angle):
    return np.tan(angle / 4.0) * unit(axis)


def angle_between(a, b):
    return float(np.arctan2(np.linalg.norm(np.cross(a, b)), np.dot(a, b)))


def transpose_apply(dcms, v):
    """[XN]^T v for each DCM of a (n, 3, 3) stack: X components to N components."""
    return np.einsum("kji,j->ki", dcms, v)


# ---------------------------------------------------------------------------
# Component Table
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Component:
    """A component as the Component Table holds it: a full frame, nominal and measured."""

    name: str
    boresight_C: np.ndarray  # boresight in component axes
    nominal_axis_B: np.ndarray  # body axis that the base frame puts on the target
    nominal_axis_label: str
    sigma_CnB: np.ndarray  # nominal frame, from the ICD
    sigma_CB_misaligned: np.ndarray  # measured frame, 2 deg off the nominal one


def hga():
    """The HGA frame, a stand-in for the ICD frame: the boresight +Z_C on +Y_B and +X_C on +X_B. The measured frame
    tilts the boresight by the misalignment and also turns the frame about the boresight."""
    dcm_CnB = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])
    tilt = rbk.PRV2C(MISALIGNMENT * unit([1.0, -1.0, 0.0]))  # about an axis normal to the boresight
    clock = rbk.PRV2C(MISALIGNMENT * E3)  # about the tilted boresight
    return Component("HGA", E3, E2, r"$+Y_B$", rbk.C2MRP(dcm_CnB), rbk.C2MRP(clock @ tilt @ dcm_CnB))


def mac():
    """The camera frame. The ICD puts the boresight +Z_C on -Y_B and image x on +Z_B."""
    dcm_CnB = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
    dcm_CB = rbk.PRV2C(MISALIGNMENT * unit([1.0, -2.0, 0.5])) @ dcm_CnB
    return Component("MAC", E3, -E2, r"$-Y_B$", rbk.C2MRP(dcm_CnB), rbk.C2MRP(dcm_CB))


# ---------------------------------------------------------------------------
# Point states: guidance module and Base Frame Table entry
# ---------------------------------------------------------------------------

SUN_N = np.zeros(3)  # [m]
EARTH_N = np.array([1.496e11, 0.0, 0.0])  # [m]
SC_CRUISE_N = np.array([1.30e11, 0.45e11, 0.08e11])  # [m] spacecraft in cruise, Earth Point
ORBIT_RADIUS = 50.0e3  # [m] Hill Point orbit about the target body at the origin
ORBIT_RATE = 2.0 * np.pi / 3600.0  # [rad/s]
SIGMA_INERTIAL_RN = np.array([0.1, -0.2, 0.3])  # Inertial Point commanded attitude


def add_earth_point(sim, task_name):
    """celestialTwoBodyPoint with Earth as the primary and the Sun as the secondary. Static geometry."""
    module = celestialTwoBodyPointF32.CelestialTwoBodyPoint()
    module.modelTag = "earthPoint"
    module.celestialBodyAlignmentThreshold = np.radians(1.0)
    nav = messaging.NavTransMsgF32Payload()
    nav.r_BN_N = SC_CRUISE_N
    nav.v_BN_N = np.zeros(3)
    earth = messaging.EphemerisMsgF32Payload()
    earth.r_BdyZero_N = EARTH_N
    earth.v_BdyZero_N = np.zeros(3)
    sun = messaging.EphemerisMsgF32Payload()
    sun.r_BdyZero_N = SUN_N
    sun.v_BdyZero_N = np.zeros(3)
    msgs = [messaging.NavTransMsgF32().write(nav), messaging.EphemerisMsgF32().write(earth),
            messaging.EphemerisMsgF32().write(sun)]
    module.transNavInMsg.subscribeTo(msgs[0])
    module.primaryCelBodyInMsg.subscribeTo(msgs[1])
    module.secondaryCelBodyInMsg.subscribeTo(msgs[2])
    sim.AddModelToTask(task_name, module, ModelPriority=20)
    return module, msgs, lambda t: None


def add_hill_point(sim, task_name):
    """hillPoint about a target body at the origin, with the spacecraft on a circular orbit about it."""
    module = hillPointF32.HillPoint()
    module.modelTag = "hillPoint"
    target = messaging.EphemerisMsgF32Payload()
    target.r_BdyZero_N = np.zeros(3)
    target.v_BdyZero_N = np.zeros(3)
    target_msg = messaging.EphemerisMsgF32().write(target)
    nav_msg = messaging.NavTransMsgF32()
    module.transNavInMsg.subscribeTo(nav_msg)
    module.celBodyInMsg.subscribeTo(target_msg)
    sim.AddModelToTask(task_name, module, ModelPriority=20)

    def write_inputs(t):
        phase = ORBIT_RATE * t
        nav = messaging.NavTransMsgF32Payload()
        nav.r_BN_N = ORBIT_RADIUS * np.array([np.cos(phase), np.sin(phase), 0.0])
        nav.v_BN_N = ORBIT_RADIUS * ORBIT_RATE * np.array([-np.sin(phase), np.cos(phase), 0.0])
        nav_msg.write(nav, macros.sec2nano(t))

    return module, [target_msg, nav_msg], write_inputs


def add_inertial_point(sim, task_name):
    """inertial3D with a fixed commanded attitude."""
    module = inertial3DF32.Inertial3D()
    module.modelTag = "inertialPoint"
    module.sigma_RN = SIGMA_INERTIAL_RN
    sim.AddModelToTask(task_name, module, ModelPriority=20)
    return module, [], lambda t: None


@dataclass(frozen=True)
class PointState:
    name: str
    component: Component
    dcm_RbR0: np.ndarray  # Base Frame Table entry: row i is body axis i in R0 components
    add_guidance: Callable
    # Axes of the target view, seen from the spacecraft toward the target: right x up = -target.
    view_axes: Callable
    view_labels: tuple
    true_target_N: Optional[Callable] = None  # geometric target direction, for the base frame check


def earth_point():
    # +Y_B on Earth (r1), +Z_B away from the Sun (-r2), -X_B on the Earth-Sun plane normal (r3).
    return PointState("Earth Point", hga(), np.array([[0.0, 0.0, -1.0], [1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]),
                      add_earth_point, lambda dcm_R0N: (dcm_R0N[2], dcm_R0N[1]),
                      ("Earth-Sun plane normal [deg]", "toward the Sun [deg]"),
                      lambda t: unit(EARTH_N - SC_CRUISE_N))


def hill_point():
    # -Y_B on the target (-r1), +X_B on the orbit normal (r3), +Z_B along-track (r2).
    return PointState("Hill Point", mac(), np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]),
                      add_hill_point, lambda dcm_R0N: (dcm_R0N[1], dcm_R0N[2]),
                      ("along-track [deg]", "orbit normal [deg]"),
                      lambda t: -unit([np.cos(ORBIT_RATE * t), np.sin(ORBIT_RATE * t), 0.0]))


def inertial_point():
    # The body takes the commanded attitude, so the MAC boresight -Y_B points along -r2.
    return PointState("Inertial Point", mac(), np.eye(3), add_inertial_point,
                      lambda dcm_R0N: (dcm_R0N[2], dcm_R0N[0]), (r"$R_0$ z [deg]", r"$R_0$ x [deg]"))


# ---------------------------------------------------------------------------
# Scenarios
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Scenario:
    title: str
    misaligned: bool
    offset: bool
    spin_axis: Optional[str] = None  # None, "boresight", "target line" or "x axis"
    spin_rate: float = SPIN_RATE


STATE_SCENARIOS = (
    Scenario("Component only", misaligned=True, offset=False),
    Scenario("Offset only", misaligned=False, offset=True),
    Scenario("Component and offset", misaligned=True, offset=True),
    Scenario("Then spin about the boresight", misaligned=True, offset=True, spin_axis="boresight"),
)

SPIN_SCENARIOS = (
    Scenario("Spin about the boresight", misaligned=True, offset=True, spin_axis="boresight"),
    Scenario("Spin about the target line", misaligned=True, offset=True, spin_axis="target line"),
    # About the offset axis, backward: the boresight sweeps from the offset point back across the target.
    Scenario("Spin about the camera x axis", misaligned=True, offset=True, spin_axis="x axis",
             spin_rate=-2.0 * OFFSET / SPIN_TIME),
)


@dataclass
class Run:
    state: PointState
    scenario: Scenario
    times: np.ndarray
    dcm_R0N: np.ndarray  # (n, 3, 3) guidance output
    dcm_RN: np.ndarray  # (n, 3, 3) mrpRotation output
    omega_RN_N: np.ndarray  # (n, 3)
    sigma_CB: np.ndarray
    sigma_RpRc_C: np.ndarray
    spin_axis_C: np.ndarray

    @property
    def component(self):
        return self.state.component

    @property
    def dcm_BC(self):
        return rbk.MRP2C(self.sigma_CB).T

    def target_N(self):
        return transpose_apply(self.dcm_R0N, self.state.dcm_RbR0.T @ self.component.nominal_axis_B)

    def boresight_N(self):
        return transpose_apply(self.dcm_RN, self.dcm_BC @ self.component.boresight_C)

    def nominal_axis_N(self):
        return transpose_apply(self.dcm_RN, self.component.nominal_axis_B)

    def body_z_N(self):
        return transpose_apply(self.dcm_RN, E3)

    def in_R0(self, vectors_N):
        return np.einsum("kij,kj->ki", self.dcm_R0N, vectors_N)

    def spin_axis_R0(self):
        """[RpR0]^T [BC] spin_axis_C, with [RpR0] = [RpRc][RcRb][RbR0]: the base, the component, then the offset."""
        dcm_RcRb = self.dcm_BC @ rbk.MRP2C(self.component.sigma_CnB)
        dcm_RpRc = self.dcm_BC @ rbk.MRP2C(self.sigma_RpRc_C) @ self.dcm_BC.T
        dcm_RpR0 = dcm_RpRc @ dcm_RcRb @ self.state.dcm_RbR0
        return dcm_RpR0.T @ self.dcm_BC @ self.spin_axis_C


def run_scenario(state, scenario):
    """Run the guidance module and mrpRotation for one scenario and record both outputs."""
    component = state.component
    sim = SimulationBaseClass.SimBaseClass()
    task_name = "componentPointingTask"
    process = sim.CreateNewProcess("componentPointingProcess")
    process.addTask(sim.CreateNewTask(task_name, macros.sec2nano(DT)))
    # The input messages must stay alive while the simulation runs.
    guidance, input_msgs, write_inputs = state.add_guidance(sim, task_name)

    sigma_CB = component.sigma_CB_misaligned if scenario.misaligned else component.sigma_CnB
    sigma_RpRc_C = mrp_from_axis_angle(E1, OFFSET) if scenario.offset else np.zeros(3)
    spin_axis_C = {
        None: component.boresight_C,
        "boresight": component.boresight_C,
        # The target is on the boresight before the offset, so this is the target line in component axes after it.
        "target line": rbk.MRP2C(sigma_RpRc_C) @ component.boresight_C,
        "x axis": E1,
    }[scenario.spin_axis]

    rotation = mrpRotationF32.MrpRotation()
    rotation.modelTag = "mrpRotation"
    rotation.sigma_RbR0 = rbk.C2MRP(state.dcm_RbR0)
    rotation.sigma_CnB = component.sigma_CnB
    rotation.sigma_CB = sigma_CB
    rotation.sigma_RpRc_C = sigma_RpRc_C
    rotation.spinAxis_C = spin_axis_C
    rotation.spinRate = scenario.spin_rate if scenario.spin_axis else 0.0
    rotation.controlPeriod = DT
    rotation.attRefInMsg.subscribeTo(guidance.attRefOutMsg)
    sim.AddModelToTask(task_name, rotation, ModelPriority=10)

    guidance_log = guidance.attRefOutMsg.recorder()
    rotation_log = rotation.attRefOutMsg.recorder()
    sim.AddModelToTask(task_name, guidance_log)
    sim.AddModelToTask(task_name, rotation_log)

    duration = SPIN_TIME if scenario.spin_axis else STATIC_TIME
    write_inputs(0.0)
    sim.InitializeSimulation()
    for k in range(int(round(duration / DT)) + 1):
        t = k * DT
        write_inputs(t)
        sim.ConfigureStopTime(macros.sec2nano(t))
        sim.ExecuteSimulation()

    return Run(state, scenario, rotation_log.times() * macros.NANO2SEC,
               np.array([rbk.MRP2C(s) for s in guidance_log.sigma_RN]),
               np.array([rbk.MRP2C(s) for s in rotation_log.sigma_RN]),
               np.array(rotation_log.omega_RN_N), np.array(sigma_CB), sigma_RpRc_C, spin_axis_C)


# ---------------------------------------------------------------------------
# Checks
# ---------------------------------------------------------------------------


def roll_angles(run):
    """Angle [rad] of +Z_B about the boresight, in R0, unwrapped from the first sample."""
    boresight = run.in_R0(run.boresight_N())
    body_z = run.in_R0(run.body_z_N())
    ref_x = unit(body_z[0] - (body_z[0] @ boresight[0]) * boresight[0])
    ref_y = np.cross(boresight[0], ref_x)
    return np.unwrap(np.arctan2(body_z @ ref_y, body_z @ ref_x))


def check_output_rate(run):
    """The output rate matches the central difference of the output attitude."""
    for k in range(1, len(run.times) - 1):
        dcm_dot = (run.dcm_RN[k + 1] - run.dcm_RN[k - 1]) / (2.0 * DT)
        skew = -dcm_dot @ run.dcm_RN[k].T
        omega_R = np.array([skew[2, 1], skew[0, 2], skew[1, 0]])
        np.testing.assert_allclose(run.dcm_RN[k].T @ omega_R, run.omega_RN_N[k], atol=RATE_TOL)


def check_run(run):
    scenario = run.scenario
    target = run.target_N()
    boresight = run.boresight_N()
    nominal = run.nominal_axis_N()
    off_target = np.array([angle_between(b, t) for b, t in zip(boresight, target)])
    misalignment = angle_between(run.component.nominal_axis_B, run.dcm_BC @ run.component.boresight_C)
    if scenario.misaligned:
        assert misalignment > np.radians(1.0), "Test setup: the component must be visibly misaligned"

    if run.state.true_target_N is not None:  # the guidance and the Base Frame Table agree on the target
        for t, target_k in zip(run.times, target):
            assert angle_between(target_k, run.state.true_target_N(t)) < ANGLE_TOL

    if scenario.spin_axis is None:
        # The offset is about the component x axis, normal to the boresight, so it moves the boresight by exactly
        # the offset angle. The nominal axis is off the boresight by the misalignment.
        np.testing.assert_allclose(off_target, OFFSET if scenario.offset else 0.0, atol=ANGLE_TOL)
        nominal_from_boresight = [angle_between(n, b) for n, b in zip(nominal, boresight)]
        np.testing.assert_allclose(nominal_from_boresight, misalignment, atol=ANGLE_TOL)
    elif scenario.spin_axis == "boresight":
        # The spin comes after the offset: the boresight holds the offset point while the spacecraft rolls.
        boresight_R0 = run.in_R0(boresight)
        drift = max(angle_between(b, boresight_R0[0]) for b in boresight_R0)
        assert drift < SPIN_ANGLE_TOL
        np.testing.assert_allclose(off_target, OFFSET, atol=SPIN_ANGLE_TOL)
        roll = roll_angles(run)
        assert abs(roll[-1] - roll[0] - scenario.spin_rate * (run.times[-1] - run.times[0])) < 1e-3
    elif scenario.spin_axis == "target line":
        # The boresight cones around the target at the offset angle.
        np.testing.assert_allclose(off_target, OFFSET, atol=SPIN_ANGLE_TOL)
        boresight_R0 = run.in_R0(boresight)
        cone_diameter = max(angle_between(b, boresight_R0[0]) for b in boresight_R0)
        assert abs(cone_diameter - 2.0 * OFFSET) < 1e-3
    elif scenario.spin_axis == "x axis":
        # The boresight sweeps the great circle normal to the spin axis, which passes through the offset point.
        boresight_R0 = run.in_R0(boresight)
        np.testing.assert_allclose(boresight_R0 @ run.spin_axis_R0(), 0.0, atol=SPIN_ANGLE_TOL)
        sweep = angle_between(boresight_R0[-1], boresight_R0[0])
        assert abs(sweep - abs(scenario.spin_rate) * (run.times[-1] - run.times[0])) < 1e-3

    check_output_rate(run)


# ---------------------------------------------------------------------------
# Figures
# ---------------------------------------------------------------------------


def target_view(run, vectors_N):
    """Angles [deg] of each direction from the target toward the view's right and up axes."""
    target = run.target_N()
    coords = np.empty((len(vectors_N), 2))
    for k, v in enumerate(vectors_N):
        right, up = run.state.view_axes(run.dcm_R0N[k])
        coords[k] = np.degrees([np.arctan2(v @ right, v @ target[k]), np.arctan2(v @ up, v @ target[k])])
    return coords


def image_view(run):
    """Angles [deg] of the target in the component frame: image x and image y from the boresight +Z_C."""
    dcm_CN = np.einsum("ij,kjl->kil", run.dcm_BC.T, run.dcm_RN)
    target_C = np.einsum("kij,kj->ki", dcm_CN, run.target_N())
    return np.degrees(np.column_stack([np.arctan2(target_C[:, 0], target_C[:, 2]),
                                       np.arctan2(target_C[:, 1], target_C[:, 2])]))


def style_axes(ax, half_width):
    ax.set_xlim(-half_width, half_width)
    ax.set_ylim(-half_width, half_width)
    ax.set_aspect("equal")
    ax.grid(color=COLOR_GRID, linewidth=0.6)
    ax.set_axisbelow(True)
    ax.axhline(0.0, color=COLOR_GRID, linewidth=1.0, zorder=0)
    ax.axvline(0.0, color=COLOR_GRID, linewidth=1.0, zorder=0)
    for spine in ax.spines.values():
        spine.set_color(COLOR_GRID)
    ax.tick_params(colors=COLOR_MUTED, labelsize=8)


def draw_roll_arrows(ax, run, boresight_xy, half_width):
    """Short arrows from the boresight toward +Z_B: twelve over a spin, one for a static run."""
    body_z = run.body_z_N()
    boresight = run.boresight_N()
    tips = target_view(run, boresight + 0.01 * body_z)
    length = 0.18 * half_width
    samples = range(0, len(run.times) - 1, (len(run.times) - 1) // 12) if run.scenario.spin_axis else [-1]
    for k in samples:
        direction = unit(tips[k] - boresight_xy[k])
        ax.annotate("", xy=boresight_xy[k] + length * direction, xytext=boresight_xy[k],
                    arrowprops=dict(arrowstyle="-|>", color=COLOR_ROLL, lw=1.5, alpha=0.9), zorder=3)


def draw_target_view(ax, run, half_width=None):
    boresight_xy = target_view(run, run.boresight_N())
    nominal_xy = target_view(run, run.nominal_axis_N())
    if half_width is None:
        half_width = max(3.0, 1.35 * np.abs(np.vstack([boresight_xy, nominal_xy])).max())
    style_axes(ax, half_width)
    ax.plot(nominal_xy[:, 0], nominal_xy[:, 1], color=COLOR_NOMINAL, linewidth=1.5, zorder=2)
    ax.plot(nominal_xy[-1, 0], nominal_xy[-1, 1], "o", markersize=8, markerfacecolor="white",
            markeredgecolor=COLOR_NOMINAL, markeredgewidth=2, zorder=4)
    ax.plot(boresight_xy[:, 0], boresight_xy[:, 1], color=COLOR_BORESIGHT, linewidth=2, zorder=3)
    ax.plot(0.0, 0.0, "*", markersize=19, color=COLOR_INK, markeredgecolor="white", markeredgewidth=1.5, zorder=4)
    ax.plot(boresight_xy[-1, 0], boresight_xy[-1, 1], "o", markersize=9, color=COLOR_BORESIGHT,
            markeredgecolor="white", markeredgewidth=2, zorder=5)
    draw_roll_arrows(ax, run, boresight_xy, half_width)
    return half_width


def summary_text(run):
    off_target = np.degrees([angle_between(b, t) for b, t in zip(run.boresight_N(), run.target_N())])
    nominal_off = np.degrees([angle_between(n, t) for n, t in zip(run.nominal_axis_N(), run.target_N())])
    lines = [f"boresight {off_target[-1]:.2f}° from target",
             f"{run.component.nominal_axis_label} {nominal_off[-1]:.2f}° from target"]
    if run.scenario.spin_axis == "boresight":
        lines.append(f"rolled {np.degrees(roll_angles(run)[-1] - roll_angles(run)[0]):.0f}° about the boresight")
    return "\n".join(lines)


def add_legend(fig, run):
    handles = [
        plt.Line2D([], [], marker="*", linestyle="none", markersize=12, color=COLOR_INK, label="target"),
        plt.Line2D([], [], marker="o", color=COLOR_BORESIGHT, linewidth=2, markersize=7,
                   label=f"{run.component.name} boresight"),
        plt.Line2D([], [], marker="o", color=COLOR_NOMINAL, linewidth=1.5, markersize=7, markerfacecolor="white",
                   markeredgewidth=2, label=f"nominal body axis {run.component.nominal_axis_label}"),
        plt.Line2D([], [], marker=r"$\rightarrow$", linestyle="none", markersize=14, color=COLOR_ROLL,
                   label=r"$+Z_B$ direction (roll)"),
    ]
    fig.legend(handles=handles, loc="lower center", ncol=4, frameon=False, fontsize=9)


def plot_state(runs):
    """One panel per scenario: the sky around the target, seen from the spacecraft."""
    state = runs[0].state
    fig, axes = plt.subplots(1, len(runs), figsize=(4.0 * len(runs), 5.3))
    half_width = max(draw_target_view(ax, run) for ax, run in zip(axes, runs))
    for ax, run in zip(axes, runs):
        ax.clear()
        draw_target_view(ax, run, half_width)
        ax.set_title(run.scenario.title, fontsize=11, color=COLOR_INK)
        ax.set_xlabel(state.view_labels[0], fontsize=9, color=COLOR_MUTED)
        boresight_up = target_view(run, run.boresight_N())[-1, 1] > 0.0
        ax.text(0.03, 0.03 if boresight_up else 0.97, summary_text(run), transform=ax.transAxes, fontsize=8,
                color=COLOR_MUTED, va="bottom" if boresight_up else "top",
                bbox=dict(facecolor="white", edgecolor="none", alpha=0.85))
    axes[0].set_ylabel(state.view_labels[1], fontsize=9, color=COLOR_MUTED)
    fig.suptitle(f"{state.name} with the {state.component.name}", fontsize=13, color=COLOR_INK, x=0.01, ha="left")
    fig.text(0.01, 0.905, "Seen from the spacecraft toward the target. Exaggerated: 2° misalignment, 6° offset about "
             f"the component x axis, {np.degrees(SPIN_RATE):.0f}°/s spin for one turn.", fontsize=9,
             color=COLOR_MUTED)
    add_legend(fig, runs[0])
    fig.tight_layout(rect=(0, 0.12, 1, 0.9))
    return fig


SPIN_CAPTIONS = {
    "boresight": ("Offset, then spin (this design).\nThe boresight holds the offset point;\n"
                  "the target circles the image center."),
    "target line": ("Same as applying the offset after the spin.\nThe boresight cones around the target;\n"
                    "the target stays put in the image."),
    "x axis": "Spin about the offset axis.\nThe boresight sweeps the offset line\nback across the target.",
}


def plot_spin_comparison(runs):
    """Top row: the sky around the target. Bottom row: where the target appears in the MAC image."""
    fig, axes = plt.subplots(2, len(runs), figsize=(4.2 * len(runs), 9.0))
    half_width = max(draw_target_view(ax, run) for ax, run in zip(axes[0], runs))
    for col, run in enumerate(runs):
        top, bottom = axes[0, col], axes[1, col]
        top.clear()
        draw_target_view(top, run, half_width)
        top.set_title(run.scenario.title, fontsize=11, color=COLOR_INK)
        top.set_xlabel(run.state.view_labels[0], fontsize=9, color=COLOR_MUTED)
        top.text(0.03, 0.03, SPIN_CAPTIONS[run.scenario.spin_axis], transform=top.transAxes, fontsize=8,
                 color=COLOR_MUTED, va="bottom", bbox=dict(facecolor="white", edgecolor="none", alpha=0.85))

        image_xy = image_view(run)
        style_axes(bottom, half_width)
        bottom.plot(image_xy[:, 0], image_xy[:, 1], color=COLOR_INK, linewidth=1.5, zorder=2)
        bottom.plot(image_xy[0, 0], image_xy[0, 1], "*", markersize=12, color="white", markeredgecolor=COLOR_INK,
                    markeredgewidth=1.5, zorder=4)
        bottom.plot(image_xy[-1, 0], image_xy[-1, 1], "*", markersize=14, color=COLOR_INK,
                    markeredgecolor="white", markeredgewidth=1.5, zorder=5)
        bottom.plot(0.0, 0.0, "o", markersize=9, color=COLOR_BORESIGHT, markeredgecolor="white",
                    markeredgewidth=2, zorder=3)
        bottom.set_xlabel("image x [deg]", fontsize=9, color=COLOR_MUTED)
    axes[0, 0].set_ylabel("Sky around the target\n" + runs[0].state.view_labels[1], fontsize=9, color=COLOR_MUTED)
    axes[1, 0].set_ylabel("Target in the MAC image\nimage y [deg]", fontsize=9, color=COLOR_MUTED)
    fig.suptitle("Spin and offset: Hill Point with the MAC", fontsize=13, color=COLOR_INK, x=0.01, ha="left")
    fig.text(0.01, 0.945, "Same 6° offset and 2° misalignment in every column; only the spin axis changes. In the "
             "image the boresight is the center, the open star the first sample and the filled star the last.",
             fontsize=9, color=COLOR_MUTED)
    add_legend(fig, runs[0])
    fig.tight_layout(rect=(0, 0.05, 1, 0.94), h_pad=2.5)
    return fig


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

STATES = {"earthPoint": earth_point, "hillPoint": hill_point, "inertialPoint": inertial_point}


def state_runs(state_name):
    state = STATES[state_name]()
    return [run_scenario(state, scenario) for scenario in STATE_SCENARIOS]


def spin_runs():
    state = hill_point()
    return [run_scenario(state, scenario) for scenario in SPIN_SCENARIOS]


@pytest.mark.parametrize("state_name", list(STATES))
def test_component_pointing(show_plots, state_name):
    """Component, offset, both, and a spin about the boresight, in each point state."""
    runs = state_runs(state_name)
    for run in runs:
        check_run(run)
    fig = plot_state(runs)
    if show_plots:
        plt.show()
    plt.close(fig)


def test_spin_and_offset(show_plots):
    """The same offset with three spin axes: the boresight, the target line, and the camera x axis."""
    runs = spin_runs()
    for run in runs:
        check_run(run)
    fig = plot_spin_comparison(runs)
    if show_plots:
        plt.show()
    plt.close(fig)


def save_figures(directory):
    os.makedirs(directory, exist_ok=True)
    figures = {f"componentPointing_{name}.png": plot_state(state_runs(name)) for name in STATES}
    figures["componentPointing_spinAndOffset.png"] = plot_spin_comparison(spin_runs())
    for filename, fig in figures.items():
        path = os.path.join(directory, filename)
        fig.savefig(path, dpi=150)
        plt.close(fig)
        print("saved", path)


if __name__ == "__main__":
    save_figures(sys.argv[1] if len(sys.argv) > 1 else ".")
