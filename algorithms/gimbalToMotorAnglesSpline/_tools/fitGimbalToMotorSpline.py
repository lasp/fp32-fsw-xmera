"""Fit the gimbal-to-motor-steps lookup table with two uniform bicubic B-spline surfaces, one for each motor.

The gimbal-to-motor-steps lookup table gives the motor 1 and motor 2 steps on a uniform gimbal tip (alpha) and tilt
(beta) grid [deg]. The populated cells form a rhombus. The tool maps the gimbal angles to the rotated, normalized
coordinates

    u = (psi - psi0) / a + (phi - phi0) / b
    v = (psi - psi0) / a - (phi - phi0) / b

so that the addressable rhombus is the square |u| <= 1, |v| <= 1. Each motor has its own surface on this square:
n1 = S1(u, v) and n2 = S2(u, v).

The rhombus is centered at the gimbal angles (psi0, phi0), and its half-diagonals are a along tip and b along tilt.
The tool selects the center and the half-diagonals together: the addressable rhombus is the rhombus of largest area,
2 a b, that is inscribed in the populated table cells. The tool reports how much of the required circle of radius
REQUIRED_RADIUS about zero gimbal angles the rhombus contains.

S1 and S2 are uniform cubic B-splines with N intervals per axis on [-1, 1], with the knot vector extended by three
knots past each end (not clamped). This gives N + 3 control coefficients per axis for each surface.

The fit uses the gimbal-to-motor-steps table. The gimbal error metrics [deg] use the motor-steps-to-gimbal table,
which the fit never sees: at each fitted table cell inside the rhombus, the fitted steps are mapped back to gimbal
angles by bilinear interpolation of the motor-steps-to-gimbal table, and compared with the gimbal angles of the cell.
The same round trip with the table steps instead of the fitted steps gives the error floor of the two tables.

Usage:
    python fitGimbalToMotorSpline.py --sweep
    python fitGimbalToMotorSpline.py --intervals 12 --report
    python fitGimbalToMotorSpline.py --intervals 12 --emit ../_tests
"""

import argparse
import os

import numpy as np
from scipy.interpolate import BSpline, NdBSpline, RegularGridInterpolator
from scipy.optimize import differential_evolution

SPLINE_DEGREE = 3
REQUIRED_RADIUS = 13.5  # [deg] Radius of the required circle of gimbal angles about zero
TOOL_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_TABLE_DIR = os.path.join(TOOL_DIR, "..", "_tests")


def load_sheets(path):
    """Load every sheet of a lookup table workbook. Blank cells are outside the measured region and become NaN.

    Args:
        path: Path to the .xlsx file.

    Returns:
        Dict of sheet name to 2D array.
    """
    import openpyxl

    workbook = openpyxl.load_workbook(path, read_only=True, data_only=True)
    sheets = {sheet.title: np.array([[np.nan if x is None else float(x) for x in row]
                                     for row in sheet.iter_rows(values_only=True)]) for sheet in workbook.worksheets}
    workbook.close()
    return sheets


def grid_axes(row_grid, col_grid):
    """Return the axis vectors of a grid whose first sheet changes down the rows and second across the columns."""
    if not (np.all(col_grid == col_grid[0]) and np.all(row_grid == row_grid[:, :1])):
        raise ValueError("The grid sheets must be constant along their other axis.")
    return row_grid[:, 0], col_grid[0]


class StepsTable:
    """The gimbal-to-motor-steps lookup table, rows along tilt (beta) and columns along tip (alpha)."""

    def __init__(self, path):
        sheets = load_sheets(path)
        self.tilt_axis, self.tip_axis = grid_axes(sheets["Beta Grid"], sheets["Alpha Grid"])  # [deg]
        self.table1 = sheets["M1"]  # [steps]
        self.table2 = sheets["M2"]  # [steps]
        if not np.array_equal(np.isnan(self.table1), np.isnan(self.table2)):
            raise ValueError("The two motor sheets must have the same populated cells.")
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

    def __init__(self, path):
        sheets = load_sheets(path)
        motor2_axis, motor1_axis = grid_axes(sheets["M2 Grid"], sheets["M1 Grid"])  # [steps]
        tip_table, tilt_table = sheets["Alpha"], sheets["Beta"]  # [deg]
        if np.isnan(tip_table).any() or np.isnan(tilt_table).any():
            raise ValueError("The gimbal sheets must have no empty cells.")
        grid = (motor2_axis, motor1_axis)
        self.tip = RegularGridInterpolator(grid, tip_table, bounds_error=False, fill_value=np.nan)
        self.tilt = RegularGridInterpolator(grid, tilt_table, bounds_error=False, fill_value=np.nan)

    def gimbal_angles(self, n1, n2):
        """Return (tip, tilt) [deg] for motor steps, by bilinear interpolation of the table. Motor steps outside the
        table give NaN."""
        steps = np.column_stack([n2, n1])
        return self.tip(steps), self.tilt(steps)


class Rhombus:
    """Rhombus |u| <= 1, |v| <= 1 in gimbal angles, with center (psi0, phi0) and half-diagonals (a, b) [deg]."""

    def __init__(self, psi0, phi0, a, b):
        self.psi0 = psi0
        self.phi0 = phi0
        self.a = a
        self.b = b

    def to_uv(self, tip, tilt):
        """Map gimbal angles [deg] to the normalized coordinates (u, v)."""
        x = (tip - self.psi0) / self.a
        y = (tilt - self.phi0) / self.b
        return x + y, x - y

    def inscribed_radius(self, tip=None, tilt=None):
        """Return the radius [deg] of the largest circle about the gimbal angles [deg] that is inside the rhombus, or
        zero if the gimbal angles are outside it. The default is the circle about the center. Each edge is a line
        |u| = 1 or |v| = 1, and the distance to it is (1 - |u|) / |grad u| with |grad u| = sqrt(1/a^2 + 1/b^2)."""
        if tip is None:
            tip, tilt = self.psi0, self.phi0
        u, v = self.to_uv(tip, tilt)
        return max(0.0, 1.0 - max(abs(u), abs(v))) / np.hypot(1.0 / self.a, 1.0 / self.b)

    def contains(self, tip, tilt, tol=1e-12):
        """Return a mask of the gimbal angles [deg] inside the rhombus."""
        u, v = self.to_uv(tip, tilt)
        return (np.abs(u) <= 1.0 + tol) & (np.abs(v) <= 1.0 + tol)


def inside_margin(tables, rhombus, num_samples=2001):
    """Return the smallest distance [deg] from the rhombus to the edge of the populated table cells, measured along
    tip at each sampled tilt and along tilt at the two tilt vertices. It is negative if the rhombus goes past them.

    Args:
        tables: StepsTable.
        rhombus: Rhombus to check.
        num_samples: Number of tilts at which the rhombus is sampled.
    """
    tilt = rhombus.phi0 + np.linspace(-rhombus.b, rhombus.b, num_samples)
    half_width = rhombus.a * (1.0 - np.abs(tilt - rhombus.phi0) / rhombus.b)
    extents = tables.row_extents()
    # Each tilt of the rhombus must lie between two populated rows that both cover the rhombus width there
    row = np.searchsorted(extents[:, 0], tilt)
    below, above = np.clip(row - 1, 0, len(extents) - 1), np.clip(row, 0, len(extents) - 1)
    return np.min(np.minimum.reduce([
        tilt - extents[0, 0],
        extents[-1, 0] - tilt,
        rhombus.psi0 - half_width - np.maximum(extents[below, 1], extents[above, 1]),
        np.minimum(extents[below, 2], extents[above, 2]) - (rhombus.psi0 + half_width),
    ]))


def largest_inscribed_rhombus(tables, seed=0):
    """Return the rhombus of largest area that is inscribed in the populated table cells.

    The half-diagonals are a = s cos(theta) and b = s sin(theta). For a center (psi0, phi0) and a shape theta, the
    largest feasible size s comes from bisection, and the area is 2 a b = s^2 sin(2 theta). The edge of the populated
    cells is a staircase, so the area has many local maxima: a global search (differential evolution, with a fixed
    seed) finds the center and the shape.

    Args:
        tables: StepsTable.
        seed: Seed of the global search.
    """

    def make(psi0, phi0, theta, size):
        return Rhombus(psi0, phi0, size * np.cos(theta), size * np.sin(theta))

    def largest_size(psi0, phi0, theta):
        low, high = 0.0, 2.0 * np.hypot(np.ptp(tables.tip), np.ptp(tables.tilt))  # [deg]
        if inside_margin(tables, make(psi0, phi0, theta, 1e-6)) < 0.0:
            return 0.0
        for _ in range(50):
            middle = 0.5 * (low + high)
            if inside_margin(tables, make(psi0, phi0, theta, middle)) >= 0.0:
                low = middle
            else:
                high = middle
        return low

    def negative_area(params):
        return -largest_size(*params) ** 2 * np.sin(2.0 * params[2])

    span = tables.step * 4.0  # [deg] The center is searched within four table steps of the middle of the cells
    middle_tip = 0.5 * (tables.tip.min() + tables.tip.max())  # [deg]
    middle_tilt = 0.5 * (tables.tilt.min() + tables.tilt.max())  # [deg]
    bounds = [(middle_tip - span, middle_tip + span),  # psi0 [deg]
              (middle_tilt - span, middle_tilt + span),  # phi0 [deg]
              (0.1, 0.5 * np.pi - 0.1)]  # theta [rad]
    result = differential_evolution(negative_area, bounds, seed=seed, polish=False, tol=1e-8, maxiter=300)
    return make(*result.x, largest_size(*result.x))


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

    def __init__(self, tables, rhombus, intervals, train_mask=None):
        self.rhombus = rhombus
        self.intervals = intervals
        self.knots = knot_vector(intervals)
        self.in_domain = rhombus.contains(tables.tip, tables.tilt)
        mask = self.in_domain if train_mask is None else (self.in_domain & train_mask)
        u, v = rhombus.to_uv(tables.tip[mask], tables.tilt[mask])
        # Both motors sample at the same (u, v), so one least-squares solve fits both surfaces
        matrix = design_matrix(u, v, self.knots)
        target = np.column_stack([tables.motor1[mask], tables.motor2[mask]])
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

    def errors(self, tables, mask=None):
        """Return the fit errors [steps] of both motors at the steps table cells in the domain (and in mask)."""
        sel = self.in_domain if mask is None else (self.in_domain & mask)
        n1, n2 = self.motor_steps(tables.tip[sel], tables.tilt[sel])
        return np.concatenate([n1 - tables.motor1[sel], n2 - tables.motor2[sel]])

    def gimbal_errors(self, tables, gimbal, mask=None):
        """Return the (tip, tilt) errors [deg] at the steps table cells in the domain (and in mask). The fitted steps
        are mapped back to gimbal angles with the motor-steps-to-gimbal table. Fitted steps outside that table give
        NaN."""
        sel = self.in_domain if mask is None else (self.in_domain & mask)
        n1, n2 = self.motor_steps(tables.tip[sel], tables.tilt[sel])
        tip, tilt = gimbal.gimbal_angles(n1, n2)
        return tip - tables.tip[sel], tilt - tables.tilt[sel]


def table_gimbal_errors(tables, gimbal, rhombus):
    """Return the (tip, tilt) errors [deg] of the table steps mapped back to gimbal angles at the steps table cells in
    the rhombus. These errors are the floor of gimbal errors: the two tables and the interpolation do not agree better.
    """
    sel = rhombus.contains(tables.tip, tables.tilt)
    tip, tilt = gimbal.gimbal_angles(tables.motor1[sel], tables.motor2[sel])
    return tip - tables.tip[sel], tilt - tables.tilt[sel]


def holdout_errors(tables, gimbal, rhombus, intervals, fraction=0.1, num_samples=10):
    """Fit without a random `fraction` of the in-domain cells and return the gimbal angle errors [deg] at those cells.

    The tool repeats this for `num_samples` random samples and returns the errors of all the samples together. An
    error is NaN where the fitted steps are outside the motor-steps-to-gimbal table.
    """
    errs = []
    for seed in range(num_samples):
        rng = np.random.default_rng(seed)
        held = rng.random(tables.tip.size) < fraction
        fit = SplineFit(tables, rhombus, intervals, train_mask=~held)
        errs.append(np.hypot(*fit.gimbal_errors(tables, gimbal, mask=held)))
    return np.concatenate(errs)


def summarize(tables, gimbal, rhombus, intervals):
    """Return a dict with the fit metrics [steps] and the gimbal errors [deg] for one rhombus and interval count."""
    fit = SplineFit(tables, rhombus, intervals)
    err = fit.errors(tables)
    tip_err, tilt_err = fit.gimbal_errors(tables, gimbal)
    gimbal_err = np.hypot(tip_err, tilt_err)
    floor = np.hypot(*table_gimbal_errors(tables, gimbal, rhombus))
    hold = holdout_errors(tables, gimbal, rhombus, intervals)
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


def sweep(tables, gimbal, rhombus, intervals_list):
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
        s = summarize(tables, gimbal, rhombus, intervals)
        print(f"| {intervals} | {s['coeffs']} | {s['bytes']} "
              f"| {s['equations_per_coeff']:.1f} | {s['rank']} | {s['condition']:.3g} "
              f"| [{s['coeff_range'][0]:.0f}, {s['coeff_range'][1]:.0f}] "
              f"| {s['tip_max']:.4f} | {s['tilt_max']:.4f} | {s['max']:.4f} | {s['rms']:.4f} | {s['p99']:.4f} "
              f"| {s['floor_max']:.4f} | {s['floor_rms']:.4f} | {s['fit_max']:.1f} | {s['fit_rms']:.2f} "
              f"| {s['holdout_max']:.4f} | {s['holdout_rms']:.4f} | {s['holdout_outside']} |")


def emit(tables, rhombus, intervals, out_dir, num_reference, seed):
    """Write the coefficient [steps], domain [rad], reference and table-point fixtures for the tests."""
    fit = SplineFit(tables, rhombus, intervals)
    d2r = np.pi / 180.0
    for motor, coeffs in ((1, fit.coeffs1), (2, fit.coeffs2)):
        np.savetxt(os.path.join(out_dir, f"splineCoefficientsMotor{motor}.csv"), coeffs.ravel()[None, :],
                   delimiter=",", fmt="%.9e")
    np.savetxt(os.path.join(out_dir, "splineDomain.csv"),
               np.array([[rhombus.psi0, rhombus.phi0, rhombus.a, rhombus.b]]) * d2r, delimiter=",", fmt="%.9e")

    # Random gimbal angles inside the rhombus, plus the four vertices
    rng = np.random.default_rng(seed)
    uv = rng.uniform(-1.0, 1.0, size=(num_reference, 2))
    uv = np.vstack([uv, [[1.0, 1.0], [1.0, -1.0], [-1.0, 1.0], [-1.0, -1.0]]])
    tip = rhombus.psi0 + rhombus.a * (uv[:, 0] + uv[:, 1]) / 2.0
    tilt = rhombus.phi0 + rhombus.b * (uv[:, 0] - uv[:, 1]) / 2.0
    n1, n2 = fit.motor_steps(tip, tilt)
    np.savetxt(os.path.join(out_dir, "splineReference.csv"), np.column_stack([tip * d2r, tilt * d2r, n1, n2]),
               delimiter=",", fmt="%.9e", header="tip [rad], tilt [rad], n1 [steps], n2 [steps]")

    sel = fit.in_domain
    np.savetxt(os.path.join(out_dir, "tablePointsInDomain.csv"),
               np.column_stack([tables.tip[sel] * d2r, tables.tilt[sel] * d2r, tables.motor1[sel],
                                tables.motor2[sel]]),
               delimiter=",", fmt="%.9e", header="tip [rad], tilt [rad], n1 [steps], n2 [steps]")
    err = fit.errors(tables)
    print(f"Wrote fixtures to {out_dir}: N={intervals}, coeffs={fit.coeffs1.size} per motor, "
          f"max fit error vs table={np.max(np.abs(err)):.3f} steps")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--steps-table",
                        default=os.path.join(DEFAULT_TABLE_DIR, "table_gimbal_angles_to_motor_steps.xlsx"),
                        help="gimbal-to-motor-steps lookup table that the spline is fitted to")
    parser.add_argument("--gimbal-table",
                        default=os.path.join(DEFAULT_TABLE_DIR, "table_motor_steps_to_gimbal_angles.xlsx"),
                        help="motor-steps-to-gimbal lookup table that maps the fitted steps back to gimbal angles")
    parser.add_argument("--intervals", type=int, default=12, help="spline intervals per axis (N)")
    parser.add_argument("--sweep", action="store_true", help="print the study over N")
    parser.add_argument("--report", action="store_true", help="print the metrics for N")
    parser.add_argument("--emit", metavar="DIR", help="write the test fixtures for N")
    parser.add_argument("--num-reference", type=int, default=200, help="number of random reference points")
    parser.add_argument("--seed", type=int, default=2026)
    args = parser.parse_args()

    tables = StepsTable(args.steps_table)
    gimbal = GimbalTable(args.gimbal_table)
    rhombus = largest_inscribed_rhombus(tables)
    print(f"Largest inscribed rhombus: center ({rhombus.psi0:.4f}, {rhombus.phi0:.4f}) deg, half-diagonals tip "
          f"{rhombus.a:.4f} deg, tilt {rhombus.b:.4f} deg. It contains the circle of radius "
          f"{rhombus.inscribed_radius(0.0, 0.0):.4f} deg about zero (required {REQUIRED_RADIUS:g} deg).")

    if args.sweep:
        sweep(tables, gimbal, rhombus, list(range(4, 22, 2)))
    if args.report:
        sweep(tables, gimbal, rhombus, [args.intervals])
    if args.emit:
        emit(tables, rhombus, args.intervals, args.emit, args.num_reference, args.seed)


if __name__ == "__main__":
    main()
