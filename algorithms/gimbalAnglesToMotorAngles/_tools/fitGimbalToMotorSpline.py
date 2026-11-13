"""Fit the gimbal-to-motor-steps lookup table with two uniform bicubic B-spline surfaces, one for each motor.

The gimbal-to-motor-steps lookup table gives the motor 1 and motor 2 steps on a uniform gimbal tip (alpha) and tilt
(beta) grid [deg]. The populated cells form a rhombus. The tool maps the gimbal angles to the rotated, normalized
coordinates

    u = psi / a + phi / b
    v = psi / a - phi / b

so that the addressable rhombus is the square |u| <= 1, |v| <= 1. Each motor has its own surface on this square:
n1 = S1(u, v) and n2 = S2(u, v).

The rhombus is centered at zero gimbal angles, and its half-diagonals are a along tip and b along tilt. The
addressable rhombus has the aspect ratio b / a of the populated table cells, and its edges touch the circle of gimbal
angles of the radius --radius plus the margin GIMBAL_MARGIN about zero. The flight module assumes that every request
is inside the circle of radius --radius (thetaMax). The margin keeps the edges away from this circle, so that a
request a little past the circle is still in the rhombus and fp32 rounding of u and v does not put a point of the
circle outside the rhombus. The tool checks that the rhombus lies inside the populated table cells.

S1 and S2 are uniform cubic B-splines with N intervals per axis on [-1, 1], with the knot vector extended by three
knots past each end (not clamped). This gives N + 3 control coefficients per axis for each surface.

The fit uses the gimbal-to-motor-steps table. The gimbal error metrics [deg] use the motor-steps-to-gimbal table,
which the fit never sees: at each fitted table cell inside the rhombus, the fitted steps are mapped back to gimbal
angles by bilinear interpolation of the motor-steps-to-gimbal table, and compared with the gimbal angles of the cell.
The same round trip with the table steps instead of the fitted steps gives the error floor of the two tables.

Usage:
    python fitGimbalToMotorSpline.py --sweep
    python fitGimbalToMotorSpline.py --radius 13.5 --intervals 6 --report
    python fitGimbalToMotorSpline.py --radius 13.5 --intervals 6 --emit ../_tests
"""

import argparse
import os

import numpy as np
from scipy.interpolate import BSpline, NdBSpline, RegularGridInterpolator

SPLINE_DEGREE = 3
DEFAULT_RADIUS = 13.5  # [deg] Default radius of the circle of gimbal angles about zero that the rhombus must contain
# [deg] Margin between the circle of radius --radius and the edges of the rhombus, the same for every radius. A request
# less than this much past the circle is still in the rhombus, so the algorithm does not reject it. The margin also
# covers fp32 rounding: the flight code computes u = fl(fl(psi / a) + fl(phi / b)) from the fp32 values of a and b, and
# the four roundings move the computed |u| and |v| at a point of the circle at most about 1.8e-7 past the exact value,
# while the margin keeps them about GIMBAL_MARGIN / (radius + GIMBAL_MARGIN) below 1. fp32_circle_max_uv checks this.
GIMBAL_MARGIN = 0.1
# [deg] Half-diagonals (a, b) of the rhombus in the rhombus mapping figure. The figure only shows the mapping, so it
# uses this made-up rhombus with different half-diagonals and not the rhombus of the fit.
MAPPING_FIGURE_RHOMBUS = (12.0, 18.0)
TOOL_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_TABLE_DIR = os.path.join(TOOL_DIR, "..", "_tests")


def load_table(path):
    """Load a lookup table CSV. The first row gives the column axis, the first column gives the row axis, and the rest
    gives the table values. A NaN value is outside the measured region.

    Args:
        path: Path to the .csv file.

    Returns:
        Tuple (row_axis, col_axis, values).
    """
    data = np.loadtxt(path, delimiter=",")
    return data[1:, 0], data[0, 1:], data[1:, 1:]


def load_table_pair(path1, path2):
    """Load two lookup table CSVs that share their axes. Returns (row_axis, col_axis, values1, values2)."""
    row_axis, col_axis, values1 = load_table(path1)
    row_axis2, col_axis2, values2 = load_table(path2)
    if not (np.array_equal(row_axis, row_axis2) and np.array_equal(col_axis, col_axis2)):
        raise ValueError(f"{path1} and {path2} must have the same axes.")
    return row_axis, col_axis, values1, values2


class StepsTable:
    """The gimbal-to-motor-steps lookup table, rows along tilt (beta) and columns along tip (alpha)."""

    def __init__(self, table_dir):
        self.tilt_axis, self.tip_axis, self.table1, self.table2 = load_table_pair(  # [deg], [deg], [steps], [steps]
            os.path.join(table_dir, "gimbalToStepsMotor1.csv"), os.path.join(table_dir, "gimbalToStepsMotor2.csv"))
        if not np.array_equal(np.isnan(self.table1), np.isnan(self.table2)):
            raise ValueError("The two motor tables must have the same populated cells.")
        self.step = min(np.median(np.diff(self.tip_axis)), np.median(np.diff(self.tilt_axis)))  # [deg]
        rows, cols = np.where(~np.isnan(self.table1))
        self.tip = self.tip_axis[cols]  # [deg]
        self.tilt = self.tilt_axis[rows]  # [deg]
        self.motor1 = self.table1[rows, cols]  # [steps]
        self.motor2 = self.table2[rows, cols]  # [steps]

    def row_extents(self):
        """Return (tilt, tip_min, tip_max) [deg] for every populated table row."""
        extents = []
        for row, tilt in enumerate(self.tilt_axis):
            cols = np.where(~np.isnan(self.table1[row]))[0]
            if cols.size:
                extents.append((tilt, self.tip_axis[cols[0]], self.tip_axis[cols[-1]]))
        return np.array(extents)


class GimbalTable:
    """The motor-steps-to-gimbal lookup table, rows along motor 2 steps and columns along motor 1 steps."""

    def __init__(self, table_dir):
        motor2_axis, motor1_axis, tip_table, tilt_table = load_table_pair(  # [steps], [steps], [deg], [deg]
            os.path.join(table_dir, "stepsToGimbalTip.csv"), os.path.join(table_dir, "stepsToGimbalTilt.csv"))
        if np.isnan(tip_table).any() or np.isnan(tilt_table).any():
            raise ValueError("The gimbal tables must have no NaN values.")
        grid = (motor2_axis, motor1_axis)
        self.tip = RegularGridInterpolator(grid, tip_table, bounds_error=False, fill_value=np.nan)
        self.tilt = RegularGridInterpolator(grid, tilt_table, bounds_error=False, fill_value=np.nan)

    def gimbal_angles(self, n1, n2):
        """Return (tip, tilt) [deg] for motor steps, by bilinear interpolation of the table. Motor steps outside the
        table give NaN."""
        steps = np.column_stack([n2, n1])
        return self.tip(steps), self.tilt(steps)


class Rhombus:
    """Rhombus |u| <= 1, |v| <= 1 in gimbal angles, centered at zero, with half-diagonals (a, b) [deg]."""

    def __init__(self, a, b):
        self.a = a
        self.b = b

    def to_uv(self, tip, tilt):
        """Map gimbal angles [deg] to the normalized coordinates (u, v)."""
        x = tip / self.a
        y = tilt / self.b
        return x + y, x - y

    def inscribed_radius(self, tip=0.0, tilt=0.0):
        """Return the radius [deg] of the largest circle about the gimbal angles [deg] that is inside the rhombus, or
        zero if the gimbal angles are outside it. The default is the circle about the center. Each edge is a line
        |u| = 1 or |v| = 1, and the distance to it is (1 - |u|) / |grad u| with |grad u| = sqrt(1/a^2 + 1/b^2)."""
        u, v = self.to_uv(tip, tilt)
        return max(0.0, 1.0 - max(abs(u), abs(v))) / np.hypot(1.0 / self.a, 1.0 / self.b)

    def contains(self, tip, tilt, tol=1e-12):
        """Return a mask of the gimbal angles [deg] inside the rhombus."""
        u, v = self.to_uv(tip, tilt)
        return (np.abs(u) <= 1.0 + tol) & (np.abs(v) <= 1.0 + tol)


def inside_margin(gimbal_to_steps_table, rhombus, num_samples=2001):
    """Return the smallest distance [deg] from the rhombus to the edge of the populated table cells, measured along
    tip at each sampled tilt and along tilt at the two tilt vertices. It is negative if the rhombus goes past them.

    Args:
        gimbal_to_steps_table: StepsTable.
        rhombus: Rhombus to check.
        num_samples: Number of tilts at which the rhombus is sampled.
    """
    tilt = np.linspace(-rhombus.b, rhombus.b, num_samples)
    half_width = rhombus.a * (1.0 - np.abs(tilt) / rhombus.b)
    extents = gimbal_to_steps_table.row_extents()
    # Each tilt of the rhombus must lie between two populated rows that both cover the rhombus width there
    row = np.searchsorted(extents[:, 0], tilt)
    below, above = np.clip(row - 1, 0, len(extents) - 1), np.clip(row, 0, len(extents) - 1)
    return np.min(np.minimum.reduce([
        tilt - extents[0, 0],
        extents[-1, 0] - tilt,
        -half_width - np.maximum(extents[below, 1], extents[above, 1]),
        np.minimum(extents[below, 2], extents[above, 2]) - half_width,
    ]))


def circle_rhombus(gimbal_to_steps_table, radius):
    """Return the rhombus about zero with the aspect ratio of the populated table cells whose edges touch the circle of
    gimbal angles of the radius plus GIMBAL_MARGIN.

    The aspect ratio is k = b / a, the tilt range over the tip range of the populated cells. The radius of the largest
    circle in the rhombus is a b / sqrt(a^2 + b^2) = a k / sqrt(1 + k^2), thus a = r' sqrt(1 + k^2) / k and
    b = r' sqrt(1 + k^2), with r' = r + GIMBAL_MARGIN.

    Args:
        gimbal_to_steps_table: StepsTable.
        radius: Radius of the circle [deg].
    """
    required = radius + GIMBAL_MARGIN  # [deg]
    ratio = np.ptp(gimbal_to_steps_table.tilt) / np.ptp(gimbal_to_steps_table.tip)
    return Rhombus(required * np.sqrt(1.0 + ratio**2) / ratio, required * np.sqrt(1.0 + ratio**2))


def fp32_circle_max_uv(rhombus, radius, num_samples=100001):
    """Return the largest |u| or |v| that fp32 calculations give on the circle of gimbal angles of the radius.

    The calculation follows the flight code: the angles and the half-diagonals are fp32 values in radians, and
    x = psi / a, y = phi / b, u = x + y, and v = x - y are fp32 operations. The result must be at most 1.

    Args:
        rhombus: Rhombus [deg].
        radius: Radius of the circle [deg].
        num_samples: Number of points on the circle.
    """
    d2r = np.pi / 180.0
    angle = np.linspace(0.0, 2.0 * np.pi, num_samples)
    tip = np.float32(radius * d2r * np.cos(angle))
    tilt = np.float32(radius * d2r * np.sin(angle))
    x = tip / np.float32(rhombus.a * d2r)
    y = tilt / np.float32(rhombus.b * d2r)
    return float(np.max(np.maximum(np.abs(x + y), np.abs(x - y))))


def knot_vector(intervals):
    """Return the uniform knot vector on [-1, 1], extended by three knots past each end."""
    h = 2.0 / intervals
    return -1.0 + h * (np.arange(intervals + 2 * SPLINE_DEGREE + 1) - SPLINE_DEGREE)


def design_matrix(u, v, knots):
    """Return the tensor-product design matrix. Column i * (N + 3) + j belongs to coefficient c[i][j]."""
    bu = BSpline.design_matrix(np.clip(u, -1.0, 1.0), knots, SPLINE_DEGREE).toarray()
    bv = BSpline.design_matrix(np.clip(v, -1.0, 1.0), knots, SPLINE_DEGREE).toarray()
    return (bu[:, :, None] * bv[:, None, :]).reshape(bu.shape[0], -1)


class SplineFit:
    """Two bicubic surfaces, one for each motor, fitted inside a rhombus: n1 = S1(u, v), n2 = S2(u, v)."""

    def __init__(self, gimbal_to_steps_table, rhombus, intervals, train_mask=None):
        self.rhombus = rhombus
        self.intervals = intervals
        self.knots = knot_vector(intervals)
        self.in_domain = rhombus.contains(gimbal_to_steps_table.tip, gimbal_to_steps_table.tilt)
        mask = self.in_domain if train_mask is None else (self.in_domain & train_mask)
        u, v = rhombus.to_uv(gimbal_to_steps_table.tip[mask], gimbal_to_steps_table.tilt[mask])
        # Both motors sample at the same (u, v), so one least-squares solve fits both surfaces
        matrix = design_matrix(u, v, self.knots)
        target = np.column_stack([gimbal_to_steps_table.motor1[mask], gimbal_to_steps_table.motor2[mask]])
        coeffs, _, self.rank, singular_values = np.linalg.lstsq(matrix, target, rcond=None)
        self.condition = singular_values[0] / singular_values[self.rank - 1]
        self.equations_per_coeff = matrix.shape[0] / matrix.shape[1]
        n = intervals + SPLINE_DEGREE
        self.coeffs1 = coeffs[:, 0].reshape(n, n)  # [steps], first index along u
        self.coeffs2 = coeffs[:, 1].reshape(n, n)  # [steps], first index along u

    def evaluate(self, coeffs, u, v):
        """Evaluate a surface [steps] with scipy's tensor-product evaluator (independent of the design matrix)."""
        spline = NdBSpline((self.knots, self.knots), coeffs, SPLINE_DEGREE)
        return spline(np.column_stack([np.clip(u, -1.0, 1.0), np.clip(v, -1.0, 1.0)]))

    def motor_steps(self, tip, tilt):
        """Return (n1, n2) [steps], not rounded, for gimbal angles [deg] inside the rhombus."""
        u, v = self.rhombus.to_uv(tip, tilt)
        return self.evaluate(self.coeffs1, u, v), self.evaluate(self.coeffs2, u, v)

    def errors(self, gimbal_to_steps_table, mask=None):
        """Return the fit errors [steps] of both motors at the steps table cells in the domain (and in mask)."""
        sel = self.in_domain if mask is None else (self.in_domain & mask)
        n1, n2 = self.motor_steps(gimbal_to_steps_table.tip[sel], gimbal_to_steps_table.tilt[sel])
        return np.concatenate([n1 - gimbal_to_steps_table.motor1[sel], n2 - gimbal_to_steps_table.motor2[sel]])

    def gimbal_errors(self, gimbal_to_steps_table, steps_to_gimbal_table, mask=None):
        """Return the (tip, tilt) errors [deg] at the steps table cells in the domain (and in mask). The fitted steps
        are mapped back to gimbal angles with the motor-steps-to-gimbal table. Fitted steps outside that table give
        NaN."""
        sel = self.in_domain if mask is None else (self.in_domain & mask)
        n1, n2 = self.motor_steps(gimbal_to_steps_table.tip[sel], gimbal_to_steps_table.tilt[sel])
        tip, tilt = steps_to_gimbal_table.gimbal_angles(n1, n2)
        return tip - gimbal_to_steps_table.tip[sel], tilt - gimbal_to_steps_table.tilt[sel]


def table_gimbal_errors(gimbal_to_steps_table, steps_to_gimbal_table, rhombus):
    """Return the (tip, tilt) errors [deg] of the table steps mapped back to gimbal angles at the steps table cells in
    the rhombus. These errors are the floor of gimbal errors: the two tables and the interpolation do not agree better.
    """
    sel = rhombus.contains(gimbal_to_steps_table.tip, gimbal_to_steps_table.tilt)
    tip, tilt = steps_to_gimbal_table.gimbal_angles(gimbal_to_steps_table.motor1[sel],
                                                    gimbal_to_steps_table.motor2[sel])
    return tip - gimbal_to_steps_table.tip[sel], tilt - gimbal_to_steps_table.tilt[sel]


def holdout_errors(gimbal_to_steps_table, steps_to_gimbal_table, rhombus, intervals, fraction=0.1, num_samples=10):
    """Fit without a random `fraction` of the in-domain cells and return the gimbal angle errors [deg] at those cells.

    The tool repeats this for `num_samples` random samples and returns the errors of all the samples together. An
    error is NaN where the fitted steps are outside the motor-steps-to-gimbal table.
    """
    errs = []
    for seed in range(num_samples):
        rng = np.random.default_rng(seed)
        held = rng.random(gimbal_to_steps_table.tip.size) < fraction
        fit = SplineFit(gimbal_to_steps_table, rhombus, intervals, train_mask=~held)
        errs.append(np.hypot(*fit.gimbal_errors(gimbal_to_steps_table, steps_to_gimbal_table, mask=held)))
    return np.concatenate(errs)


def summarize(gimbal_to_steps_table, steps_to_gimbal_table, rhombus, intervals):
    """Return a dict with the fit metrics [steps] and the gimbal errors [deg] for one rhombus and interval count."""
    fit = SplineFit(gimbal_to_steps_table, rhombus, intervals)
    err = fit.errors(gimbal_to_steps_table)
    tip_err, tilt_err = fit.gimbal_errors(gimbal_to_steps_table, steps_to_gimbal_table)
    gimbal_err = np.hypot(tip_err, tilt_err)
    floor = np.hypot(*table_gimbal_errors(gimbal_to_steps_table, steps_to_gimbal_table, rhombus))
    hold = holdout_errors(gimbal_to_steps_table, steps_to_gimbal_table, rhombus, intervals)
    coeffs = np.concatenate([fit.coeffs1, fit.coeffs2])
    return {
        "fit": fit,
        "fit_max": np.max(np.abs(err)),
        "fit_rms": np.sqrt(np.mean(err**2)),
        "tip_max": np.max(np.abs(tip_err)),
        "tilt_max": np.max(np.abs(tilt_err)),
        "max": np.max(gimbal_err),
        "rms": np.sqrt(np.mean(gimbal_err**2)),
        "p99": np.percentile(gimbal_err, 99),
        "floor_max": np.max(floor),
        "floor_rms": np.sqrt(np.mean(floor**2)),
        "holdout_max": np.nanmax(hold),
        "holdout_rms": np.sqrt(np.nanmean(hold**2)),
        "holdout_outside": np.count_nonzero(np.isnan(hold)),
        "condition": fit.condition,
        "rank": fit.rank,
        "equations_per_coeff": fit.equations_per_coeff,
        "coeff_range": (coeffs.min(), coeffs.max()),
        "coeffs": coeffs.size,
        "bytes": 4 * coeffs.size,
    }


def sweep(gimbal_to_steps_table, steps_to_gimbal_table, rhombus, intervals_list):
    """Print a markdown table of the fit metrics over the interval counts for the rhombus.

    The gimbal columns [deg] map the fitted steps back to gimbal angles with the motor-steps-to-gimbal table. The
    total gimbal error is the root sum square of the tip and tilt errors. The table floor columns [deg] do the same
    with the table steps. The holdout columns [deg] give the gimbal errors at cells that the fit did not use, and the
    number of these cells whose fitted steps are outside the motor-steps-to-gimbal table (not in the maximum and RMS).
    The fit columns [steps] compare the fit with the gimbal-to-motor-steps table that it was fitted to. The equation
    count and the condition number are for one surface.

    Args:
        rhombus: Addressable rhombus [deg].
        intervals_list: List of interval counts N.
    """
    print("| N | coeffs | bytes | eqs/coeff | rank | cond(A) | coeff range [steps] | tip max [deg] | tilt max [deg] "
          "| gimbal max [deg] | gimbal rms [deg] | gimbal p99 [deg] | floor max [deg] | floor rms [deg] "
          "| fit max [steps] | fit rms [steps] | holdout max [deg] | holdout rms [deg] | holdout outside |")
    print("|" + "---|" * 19)
    for intervals in intervals_list:
        s = summarize(gimbal_to_steps_table, steps_to_gimbal_table, rhombus, intervals)
        print(f"| {intervals} | {s['coeffs']} | {s['bytes']} "
              f"| {s['equations_per_coeff']:.1f} | {s['rank']} | {s['condition']:.3g} "
              f"| [{s['coeff_range'][0]:.0f}, {s['coeff_range'][1]:.0f}] "
              f"| {s['tip_max']:.4f} | {s['tilt_max']:.4f} | {s['max']:.4f} | {s['rms']:.4f} | {s['p99']:.4f} "
              f"| {s['floor_max']:.4f} | {s['floor_rms']:.4f} | {s['fit_max']:.1f} | {s['fit_rms']:.2f} "
              f"| {s['holdout_max']:.4f} | {s['holdout_rms']:.4f} | {s['holdout_outside']} |")


def emit(gimbal_to_steps_table, rhombus, radius, intervals, out_dir):
    """Write the configuration: the coefficients [steps] of the two motors and the domain [rad]. The domain file has
    the half-diagonals a and b and the radius of the circle. The Python tests of the module check this configuration
    against the tables, and the scenarios load it."""
    fit = SplineFit(gimbal_to_steps_table, rhombus, intervals)
    d2r = np.pi / 180.0
    for motor, coeffs in ((1, fit.coeffs1), (2, fit.coeffs2)):
        np.savetxt(os.path.join(out_dir, f"splineCoefficientsMotor{motor}.csv"), coeffs.ravel()[None, :],
                   delimiter=",", fmt="%.9e")
    np.savetxt(os.path.join(out_dir, "splineDomain.csv"),
               np.array([[rhombus.a, rhombus.b, radius]]) * d2r, delimiter=",", fmt="%.9e")
    err = fit.errors(gimbal_to_steps_table)
    print(f"Wrote the configuration to {out_dir}: N={intervals}, coeffs={fit.coeffs1.size} per motor, "
          f"max fit error vs table={np.max(np.abs(err)):.3f} steps")


def save_figure(fig, out_dir, name):
    """Write the figure to out_dir as name.pdf for the LaTeX documentation and name.svg for the HTML documentation."""
    for extension in ("pdf", "svg"):
        fig.savefig(os.path.join(out_dir, f"{name}.{extension}"))


def mapping_figure(rhombus, out_dir=None, show=False):
    """Plot how the gimbal angles map to the normalized coordinates in three steps.

    1. The rhombus in gimbal angles (psi, phi) [deg].
    2. Normalize: x = psi / a, y = phi / b gives the unit rhombus |x| + |y| <= 1.
    3. Rotate by 45 deg and scale by sqrt(2): (v, u) = (x - y, x + y) gives the square |u| <= 1, |v| <= 1. The
       square is drawn with v on the horizontal axis and u on the vertical axis, so this step is a pure rotation.

    Args:
        rhombus: Rhombus [deg] to draw in the first panel.
        out_dir: Directory to write rhombusMapping.pdf and .svg to, or None to skip writing it.
        show: Open the figure in an interactive window.
    """
    import matplotlib

    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import ConnectionPatch

    # Every curve is defined in (u, v) and mapped to each panel, so the three panels show the same points
    def to_panels(u, v):
        x, y = (u + v) / 2.0, (u - v) / 2.0
        gimbal = (rhombus.a * x, rhombus.b * y)
        return [gimbal, (x, y), (v, u)]

    s = np.linspace(-1.0, 1.0, 2)
    one = np.ones_like(s)
    edges = [  # (u, v) of the edge, color, label
        ((one, s), "tab:red", r"$u = +1$"),
        ((s, one), "tab:blue", r"$v = +1$"),
        ((-one, s), "tab:green", r"$u = -1$"),
        ((s, -one), "tab:orange", r"$v = -1$"),
    ]
    vertices = [((1.0, 1.0), r"$A$ (+tip)"), ((1.0, -1.0), r"$B$ (+tilt)"), ((-1.0, -1.0), r"$C$ (-tip)"),
                ((-1.0, 1.0), r"$D$ (-tilt)")]
    grid = [-0.5, 0.0, 0.5]
    point = (0.5, -0.25)  # (u, v) of the example point P

    fig, axes = plt.subplots(1, 3, figsize=(13.0, 5.0))
    titles = ["1. Gimbal angles", "2. Normalize", "3. Rotate 45 deg and scale"]
    labels = [(r"Gimbal tip angle $\psi$ [deg]", r"Gimbal tilt angle $\phi$ [deg]"), (r"$x$", r"$y$"),
              (r"$v = x - y$", r"$u = x + y$")]
    for panel, ax in enumerate(axes):
        for level in grid:
            for uv in ((level * one, s), (s, level * one)):
                ax.plot(*to_panels(*uv)[panel], color="0.75", linewidth=0.7, linestyle="--", zorder=1)
        for uv, color, label in edges:
            ax.plot(*to_panels(*uv)[panel], color=color, linewidth=2.5, label=label, zorder=2)
        cx, cy = to_panels(np.zeros(1), np.zeros(1))[panel]
        for (u, v), name in vertices:
            px, py = to_panels(np.array([u]), np.array([v]))[panel]
            ax.plot(px, py, "ko", markersize=4, zorder=3)
            # Place the name outside the shape, away from the center
            dx, dy = np.sign(np.round(px[0] - cx[0], 6)), np.sign(np.round(py[0] - cy[0], 6))
            ax.annotate(name, (px[0], py[0]), xytext=(7 * dx, 7 * dy), textcoords="offset points", fontsize=9,
                        ha={1.0: "left", 0.0: "center", -1.0: "right"}[dx],
                        va={1.0: "bottom", 0.0: "center", -1.0: "top"}[dy])
        px, py = to_panels(np.array([point[0]]), np.array([point[1]]))[panel]
        ax.plot(px, py, marker="*", color="k", markersize=11, zorder=4)
        ax.annotate(r"$P$", (px[0], py[0]), xytext=(6, 4), textcoords="offset points", fontsize=10)
        ax.plot(cx, cy, "k+", markersize=9, zorder=3)
        ax.set_title(titles[panel], fontsize=11)
        ax.set_xlabel(labels[panel][0])
        ax.set_ylabel(labels[panel][1])
        ax.set_aspect("equal")
        ax.axhline(cy[0], color="0.4", linewidth=0.5, zorder=0)
        ax.axvline(cx[0], color="0.4", linewidth=0.5, zorder=0)

    # Half-diagonals of the gimbal rhombus
    r = rhombus
    ax = axes[0]
    ax.annotate("", (r.a, -0.2 * r.b), (0.0, -0.2 * r.b), arrowprops={"arrowstyle": "<->", "color": "k"})
    ax.text(r.a / 2.0, -0.22 * r.b, r"$a$", ha="center", va="top", fontsize=11)
    ax.plot([-1.15 * r.a, 0.0], [r.b] * 2, color="0.4", linewidth=0.5, linestyle=":")
    ax.annotate("", (-1.1 * r.a, r.b), (-1.1 * r.a, 0.0), arrowprops={"arrowstyle": "<->", "color": "k"})
    ax.text(-1.2 * r.a, r.b / 2.0, r"$b$", ha="right", va="center", fontsize=11)
    for ax, (cx, cy), lim in zip(axes, to_panels(np.zeros(1), np.zeros(1)), [1.3 * max(r.a, r.b), 1.6, 1.45]):
        ax.set_xlim(cx[0] - lim, cx[0] + lim)
        ax.set_ylim(cy[0] - lim, cy[0] + lim)
    axes[1].set_xticks([-1, 0, 1])
    axes[1].set_yticks([-1, 0, 1])
    axes[2].set_xticks([-1, 0, 1])
    axes[2].set_yticks([-1, 0, 1])
    fig.legend(*axes[2].get_legend_handles_labels(), loc="lower center", ncol=4, fontsize=9, handlelength=2.0)

    fig.tight_layout(rect=(0.0, 0.06, 1.0, 0.9), w_pad=4.0)
    steps = [r"$x = \psi/a$" "\n" r"$y = \phi/b$",
             r"$u = x + y$" "\n" r"$v = x - y$"]
    for i, text in enumerate(steps):
        arrow = ConnectionPatch((1.03, 1.08), (-0.03, 1.08), "axes fraction", "axes fraction", axesA=axes[i],
                                axesB=axes[i + 1], arrowstyle="-|>", mutation_scale=15, linewidth=1.5)
        fig.add_artist(arrow)
        x0 = axes[i].get_position().x1
        x1 = axes[i + 1].get_position().x0
        y = axes[i].get_position().y1 + 0.08 * axes[i].get_position().height
        fig.text((x0 + x1) / 2.0, y + 0.015, text, ha="center", va="bottom", fontsize=10)

    if out_dir:
        save_figure(fig, out_dir, "rhombusMapping")
        print(f"Wrote rhombusMapping.pdf and .svg to {out_dir}")
    if show:
        plt.show()
    else:
        plt.close(fig)


def basis_figure(out_dir=None, show=False):
    """Plot the four cubic B-spline basis functions that are not zero on one interval k.

    The left panel shows the four basis functions of the coefficients c_k to c_{k+3} over the knots, with interval k
    shaded. The right panel shows the parts of these functions on interval k as the weights B_0(t) to B_3(t).

    Args:
        out_dir: Directory to write basisFunctions.pdf and .svg to, or None to skip writing it.
        show: Open the figure in an interactive window.
    """
    import matplotlib

    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    colors = ["tab:purple", "tab:blue", "tab:green", "tab:orange"]
    # The knots are at the integers, in units of h, with interval k from 0 to 1
    fig, (left, right) = plt.subplots(1, 2, figsize=(12.0, 4.5), gridspec_kw={"width_ratios": [1.6, 1.0]})
    left.axvspan(0.0, 1.0, color="0.9", zorder=0)
    left.text(0.5, 0.75, f"interval $k$", ha="center", fontsize=10)
    for p, color in enumerate(colors):
        # The basis function of c_{k+p} is not zero from knot k + p - 3 to knot k + p + 1
        bump = BSpline.basis_element(np.arange(p - 3, p + 2, dtype=float), extrapolate=False)
        x = np.linspace(p - 3.0, p + 1.0, 400)
        left.plot(x, bump(x), color=color, linewidth=1.2, alpha=0.6)
        on_interval = np.linspace(0.0, 1.0, 100)
        left.plot(on_interval, bump(on_interval), color=color, linewidth=3.0)
        left.annotate(fr"$c_{{k+{p}}}$" if p else r"$c_k$", (p - 1.0, 2.0 / 3.0), xytext=(0, 5),
                      textcoords="offset points", ha="center", color=color, fontsize=11)
    left.set_xticks(range(-3, 5))
    left.set_xticklabels([r"$k$" if j == 0 else fr"$k{j:+d}$" for j in range(-3, 5)])
    left.set_xlim(-3.2, 4.2)
    left.set_ylim(0.0, 0.82)
    left.set_xlabel(r"Knot (spacing $h$)")
    left.set_ylabel("Basis function value")
    left.set_title(r"Basis functions of $c_k$ to $c_{k+3}$", fontsize=11)

    t = np.linspace(0.0, 1.0, 200)
    weights = [(1.0 - t) ** 3 / 6.0, (3.0 * t**3 - 6.0 * t**2 + 4.0) / 6.0,
               (-3.0 * t**3 + 3.0 * t**2 + 3.0 * t + 1.0) / 6.0, t**3 / 6.0]
    for p, (weight, color) in enumerate(zip(weights, colors)):
        right.plot(t, weight, color=color, linewidth=2.5, label=fr"$B_{p}(t)$, weight of " +
                   (fr"$c_{{k+{p}}}$" if p else r"$c_k$"))
    right.plot(t, sum(weights), "k--", linewidth=1.0, label="Sum of the weights")
    right.set_yticks([0.0, 1.0 / 6.0, 4.0 / 6.0, 1.0])
    right.set_yticklabels(["0", "1/6", "4/6", "1"])
    right.grid(axis="y", color="0.85", linewidth=0.7)
    right.set_xlim(0.0, 1.0)
    right.set_ylim(0.0, 1.08)
    right.set_xlabel(r"Local coordinate $t$ in interval $k$")
    right.set_ylabel("Weight")
    right.set_title(r"Weights on interval $k$", fontsize=11)
    right.legend(loc="upper center", bbox_to_anchor=(0.5, 0.88), ncol=2, fontsize=8)

    fig.tight_layout()
    if out_dir:
        save_figure(fig, out_dir, "basisFunctions")
        print(f"Wrote basisFunctions.pdf and .svg to {out_dir}")
    if show:
        plt.show()
    else:
        plt.close(fig)


def figures(gimbal_to_steps_table, steps_to_gimbal_table, rhombus, radius, intervals, out_dir=None, show=False):
    """Plot the domain and motor 1 table, the gimbal angle error map, and the 3D motor 1 fit surface colored by the
    gimbal angle error.

    Args:
        radius: Radius of the circle that the rhombus contains [deg].
        out_dir: Directory to write the PDF figures to, or None to skip writing them.
        show: Open the figures in interactive windows.
    """
    import matplotlib

    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    mapping_figure(Rhombus(*MAPPING_FIGURE_RHOMBUS), out_dir, show)
    basis_figure(out_dir, show)
    fit = SplineFit(gimbal_to_steps_table, rhombus, intervals)

    def finish(fig, name):
        fig.tight_layout()
        if out_dir:
            save_figure(fig, out_dir, name)
        if not show:
            plt.close(fig)

    def outline(r):
        tip = np.array([r.a, 0.0, -r.a, 0.0, r.a])
        tilt = np.array([0.0, r.b, 0.0, -r.b, 0.0])
        return tip, tilt

    def circle():
        angle = np.linspace(0.0, 2.0 * np.pi, 361)
        return radius * np.cos(angle), radius * np.sin(angle)

    # The table cells are faded and the overlays use colors that are not in the viridis colormap, so the overlays stand
    # out from the cells
    point_alpha = 0.45

    def plot_overlays(ax):
        ax.plot(*outline(rhombus), color="black", linewidth=2.5, label="Addressable rhombus", zorder=3)
        ax.plot(*circle(), color="#e6007e", linewidth=2.5,
                label=f"Required circle (radius {radius:g} deg about zero)", zorder=4)
        ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.1), ncol=2, fontsize=8, frameon=False)

    fig, ax = plt.subplots(figsize=(6.0, 6.5))
    points = ax.scatter(gimbal_to_steps_table.tip, gimbal_to_steps_table.tilt, c=gimbal_to_steps_table.motor1, s=4,
                        cmap="viridis", alpha=point_alpha)
    plot_overlays(ax)
    ax.set_xlabel("Gimbal tip angle [deg]")
    ax.set_ylabel("Gimbal tilt angle [deg]")
    ax.set_aspect("equal")
    fig.colorbar(points, ax=ax, label="Measured motor 1 steps [steps]")
    finish(fig, "gimbalDomain")

    sel = fit.in_domain
    n1, _ = fit.motor_steps(gimbal_to_steps_table.tip[sel], gimbal_to_steps_table.tilt[sel])
    err = np.hypot(*fit.gimbal_errors(gimbal_to_steps_table, steps_to_gimbal_table))
    limit = np.max(err)
    fig, ax = plt.subplots(figsize=(6.0, 6.5))
    points = ax.scatter(gimbal_to_steps_table.tip[sel], gimbal_to_steps_table.tilt[sel], c=err, s=6, cmap="viridis",
                        vmin=0.0, vmax=limit, alpha=point_alpha)
    plot_overlays(ax)
    ax.set_xlabel("Gimbal tip angle [deg]")
    ax.set_ylabel("Gimbal tilt angle [deg]")
    ax.set_aspect("equal")
    fig.colorbar(points, ax=ax, label="Gimbal angle error [deg]")
    finish(fig, "fitError")

    # Spline surface on the in-domain table cells, each triangle colored by the mean error of its three vertices
    from matplotlib.tri import Triangulation

    tip = gimbal_to_steps_table.tip[sel]
    tilt = gimbal_to_steps_table.tilt[sel]
    motor1 = gimbal_to_steps_table.motor1[sel]
    tri = Triangulation(tip, tilt)
    fig = plt.figure(figsize=(8.0, 6.5))
    ax = fig.add_subplot(projection="3d")
    surface = ax.plot_trisurf(tri, n1, cmap="viridis", linewidth=0.0, antialiased=False)
    surface.set_array(err[tri.triangles].mean(axis=1))
    surface.set_clim(0.0, limit)
    ax.scatter(tip, tilt, motor1, c="k", s=0.5, alpha=0.4, depthshade=False, label="Lookup table (truth)")
    ax.set_xlabel("Gimbal tip angle [deg]")
    ax.set_ylabel("Gimbal tilt angle [deg]")
    ax.set_zlabel("Motor 1 steps [steps]")
    ax.legend(loc="upper left", fontsize=8)
    fig.colorbar(surface, ax=ax, shrink=0.7, pad=0.1, label="Gimbal angle error [deg]")
    finish(fig, "fitSurface3d")
    if out_dir:
        print(f"Wrote figures to {out_dir}")
    if show:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--table-dir", default=DEFAULT_TABLE_DIR,
                        help="directory of the lookup table CSVs: gimbalToStepsMotor1.csv and gimbalToStepsMotor2.csv "
                             "(fitted), stepsToGimbalTip.csv and stepsToGimbalTilt.csv (gimbal error metrics)")
    parser.add_argument("--radius", type=float, default=DEFAULT_RADIUS,
                        help="radius [deg] of the circle of gimbal angles about zero that the rhombus must contain")
    parser.add_argument("--intervals", type=int, default=6, help="spline intervals per axis (N)")
    parser.add_argument("--sweep", action="store_true", help="print the study over N")
    parser.add_argument("--report", action="store_true", help="print the metrics for N")
    parser.add_argument("--emit", metavar="DIR", help="write the configuration for N")
    parser.add_argument("--figures", metavar="DIR", help="write the documentation figures for N")
    parser.add_argument("--show", action="store_true", help="open the figures in interactive windows")
    args = parser.parse_args()

    gimbal_to_steps_table = StepsTable(args.table_dir)
    steps_to_gimbal_table = GimbalTable(args.table_dir)
    rhombus = circle_rhombus(gimbal_to_steps_table, args.radius)
    margin = inside_margin(gimbal_to_steps_table, rhombus)
    max_uv = fp32_circle_max_uv(rhombus, args.radius)
    print(f"Rhombus about the circle of radius {args.radius:g} deg plus the margin {GIMBAL_MARGIN:g} deg: "
          f"half-diagonals tip {rhombus.a:.6f} deg, tilt "
          f"{rhombus.b:.6f} deg. Distance to the edge of the populated table cells: {margin:.4f} deg. "
          f"Largest fp32 |u| or |v| on the circle: 1 - {1.0 - max_uv:.3g}.")
    if margin < 0.0:
        raise SystemExit(f"The rhombus goes {-margin:.4f} deg past the populated table cells. Use a smaller --radius.")
    if max_uv > 1.0:
        raise SystemExit("fp32 rounding puts a point of the circle outside the rhombus. Increase GIMBAL_MARGIN.")

    if args.sweep:
        sweep(gimbal_to_steps_table, steps_to_gimbal_table, rhombus, list(range(4, 22, 2)))
    if args.report:
        sweep(gimbal_to_steps_table, steps_to_gimbal_table, rhombus, [args.intervals])
    if args.emit:
        emit(gimbal_to_steps_table, rhombus, args.radius, args.intervals, args.emit)
    if args.figures or args.show:
        figures(gimbal_to_steps_table, steps_to_gimbal_table, rhombus, args.radius, args.intervals, args.figures,
                args.show)


if __name__ == "__main__":
    main()
