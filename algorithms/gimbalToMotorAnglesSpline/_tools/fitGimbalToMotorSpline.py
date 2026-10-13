"""Fit the measured gimbal-to-motor angle tables with one shared uniform bicubic B-spline surface.

The measured tables give the motor 1 and motor 2 angles [deg] on a 0.5 deg gimbal tip/tilt grid. The populated
cells form a rhombus. The tool maps the gimbal angles to the rotated, normalized coordinates

    u = (psi - psi0) / a + (phi - phi0) / b
    v = (psi - psi0) / a - (phi - phi0) / b

so that the addressable rhombus is the square |u| <= 1, |v| <= 1. Motor 2 is the mirror of motor 1 about the
tilt center, so one surface S serves both motors: theta1 = S(u, v), theta2 = S(v, u).

S is a uniform cubic B-spline with N intervals per axis on [-1, 1], with the knot vector extended by three knots past
each end (not clamped). This gives N + 3 control coefficients per axis.

The surface is steep near the +tip edges of the measured rhombus. A band at those edges (or at all edges) is removed
from the addressable rhombus before the fit, so the spline never needs to follow the steep region.

Usage:
    python fitGimbalToMotorSpline.py --sweep
    python fitGimbalToMotorSpline.py --intervals 16 --band 1.0 --report
    python fitGimbalToMotorSpline.py --intervals 16 --band 1.0 --emit ../_tests
"""

import argparse
import os

import numpy as np
from scipy.interpolate import BSpline, bisplev

SPLINE_DEGREE = 3
TOOL_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_TABLE_DIR = os.path.join(TOOL_DIR, "..", "_tests")


def load_table(path):
    """Load a measured table CSV. Blank cells are outside the measured rhombus and become NaN.

    Args:
        path: Path to the CSV file [deg].

    Returns:
        2D array of motor angles [deg], rows along tilt and columns along tip.
    """
    with open(path) as f:
        rows = [line.rstrip("\n").split(",") for line in f if line.strip()]
    return np.array([[float(x) if x else np.nan for x in row] for row in rows])


class MeasuredTables:
    """The two measured tables and the gimbal angles of their populated cells."""

    def __init__(self, table1, table2, step, tip_col_offset, tilt_row_offset):
        if table1.shape != table2.shape or not np.array_equal(np.isnan(table1), np.isnan(table2)):
            raise ValueError("The two tables must have the same shape and the same populated cells.")
        self.table1 = table1
        self.table2 = table2
        self.step = step
        self.tip_col_offset = tip_col_offset
        self.tilt_row_offset = tilt_row_offset
        rows, cols = np.where(~np.isnan(table1))
        self.tip = (cols - tip_col_offset) * step  # [deg]
        self.tilt = (rows - tilt_row_offset) * step  # [deg]
        self.motor1 = table1[rows, cols]  # [deg]
        self.motor2 = table2[rows, cols]  # [deg]

    def row_extents(self):
        """Return (tilt, tip_min, tip_max) [deg] for every populated table row."""
        extents = []
        for row in range(self.table1.shape[0]):
            cols = np.where(~np.isnan(self.table1[row]))[0]
            if cols.size:
                extents.append(((row - self.tilt_row_offset) * self.step,
                                (cols[0] - self.tip_col_offset) * self.step,
                                (cols[-1] - self.tip_col_offset) * self.step))
        return np.array(extents)

    def symmetry_error(self):
        """Return the max |theta2(psi, phi) - theta1(psi, -phi)| [deg] over the cells where both exist."""
        mirrored = self.table1[::-1, :]
        if self.table1.shape[0] != 2 * self.tilt_row_offset + 1:
            raise ValueError("The tilt grid must be symmetric about zero tilt to check the motor symmetry.")
        return np.nanmax(np.abs(self.table2 - mirrored))


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

    def contains(self, tip, tilt, tol=1e-12):
        """Return a mask of the gimbal angles [deg] inside the rhombus."""
        u, v = self.to_uv(tip, tilt)
        return (np.abs(u) <= 1.0 + tol) & (np.abs(v) <= 1.0 + tol)

    def area(self):
        """Return the rhombus area [deg^2]."""
        return 2.0 * self.a * self.b


def inscribed_rhombus(tables, tol):
    """Find the largest rhombus, centered at zero tilt, that lies inside the populated table rows.

    The tilt center is fixed at zero because the motor symmetry is a reflection about zero tilt. The tip center and
    the tilt half-diagonal are searched on a grid. For each pair the largest feasible tip half-diagonal is computed.

    Args:
        tables: MeasuredTables.
        tol: [deg] Allowed overhang of the rhombus past the populated cells of a row.

    Returns:
        Rhombus with the largest area.
    """
    extents = tables.row_extents()
    tilt_rows, tip_lo, tip_hi = extents[:, 0], extents[:, 1], extents[:, 2]
    max_tilt = np.max(np.abs(tilt_rows))
    best = None
    for psi0 in np.arange(tip_lo.min() / 4.0, tip_hi.max() / 4.0, tables.step / 10.0):
        for b in np.arange(0.8 * max_tilt, max_tilt + tol + 1e-9, tables.step / 10.0):
            frac = 1.0 - np.abs(tilt_rows) / b
            inside = frac > 0.0
            room = np.minimum(psi0 - tip_lo, tip_hi - psi0) + tol
            if np.any(room[inside] < 0.0):
                continue
            a = np.min(room[inside] / frac[inside])
            candidate = Rhombus(psi0, 0.0, a, b)
            if best is None or candidate.area() > best.area():
                best = candidate
    return best


def shrink(rhombus, band, mode):
    """Remove a band of tip width `band` [deg] (measured at the tilt center) from the rhombus edges.

    Args:
        rhombus: Rhombus to shrink.
        band: [deg] Band width along tip at the tilt center.
        mode: "plus-tip" removes the band from the two +tip edges only (u <= 1 - m, v <= 1 - m).
              "all" removes it from all four edges (|u|, |v| <= 1 - m).

    Returns:
        The shrunk Rhombus. It is still a rhombus, so the u <-> v symmetry is kept.
    """
    m = band / rhombus.a
    if mode == "plus-tip":
        scale = 1.0 - m / 2.0
        return Rhombus(rhombus.psi0 - band / 2.0, rhombus.phi0, rhombus.a * scale, rhombus.b * scale)
    if mode == "all":
        return Rhombus(rhombus.psi0, rhombus.phi0, rhombus.a * (1.0 - m), rhombus.b * (1.0 - m))
    raise ValueError(f"Unknown band mode {mode}")


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
    """One shared bicubic surface fitted to both motors inside a rhombus."""

    def __init__(self, tables, rhombus, intervals, train_mask=None):
        self.rhombus = rhombus
        self.intervals = intervals
        self.knots = knot_vector(intervals)
        self.in_domain = rhombus.contains(tables.tip, tables.tilt)
        mask = self.in_domain if train_mask is None else (self.in_domain & train_mask)
        u, v = rhombus.to_uv(tables.tip[mask], tables.tilt[mask])
        # Motor 1 samples S at (u, v) and motor 2 samples S at (v, u)
        matrix = np.vstack([design_matrix(u, v, self.knots), design_matrix(v, u, self.knots)])
        target = np.concatenate([tables.motor1[mask], tables.motor2[mask]])
        coeffs, *_ = np.linalg.lstsq(matrix, target, rcond=None)
        n = intervals + SPLINE_DEGREE
        self.coeffs = coeffs.reshape(n, n)  # [deg], first index along u

    def evaluate(self, u, v):
        """Evaluate S(u, v) [deg] with scipy's FITPACK evaluator (independent of the design matrix)."""
        tck = (self.knots, self.knots, self.coeffs.ravel(), SPLINE_DEGREE, SPLINE_DEGREE)
        return np.array([bisplev(ui, vi, tck) for ui, vi in zip(u, v)])

    def motor_angles(self, tip, tilt):
        """Return (theta1, theta2) [deg] for gimbal angles [deg] inside the rhombus."""
        u, v = self.rhombus.to_uv(tip, tilt)
        return self.evaluate(u, v), self.evaluate(v, u)

    def errors(self, tables, mask=None):
        """Return the fit errors [deg] of both motors at the table cells in the domain (and in mask)."""
        sel = self.in_domain if mask is None else (self.in_domain & mask)
        theta1, theta2 = self.motor_angles(tables.tip[sel], tables.tilt[sel])
        return np.concatenate([theta1 - tables.motor1[sel], theta2 - tables.motor2[sel]])


def bilinear_proxy_error(tables, rhombus):
    """Estimate the bilinear interpolation error [deg] of the 0.5 deg table inside the rhombus.

    The tool interpolates the cell centers of the 2 * step subgrid from their four corners and compares with the
    measured value. Bilinear error scales with the square of the grid step, so the result is divided by four.
    """
    errs = []
    for table in (tables.table1, tables.table2):
        rows, cols = table.shape
        for i in range(0, rows - 2):
            for j in range(0, cols - 2):
                corners = table[[i, i, i + 2, i + 2], [j, j + 2, j, j + 2]]
                center = table[i + 1, j + 1]
                if np.any(np.isnan(corners)) or np.isnan(center):
                    continue
                tip = (np.array([j, j + 2]) - tables.tip_col_offset) * tables.step
                tilt = (np.array([i, i + 2]) - tables.tilt_row_offset) * tables.step
                if not np.all(rhombus.contains(np.repeat(tip, 2), np.tile(tilt, 2))):
                    continue
                errs.append(abs(corners.mean() - center) / 4.0)
    return np.array(errs)


def holdout_errors(tables, rhombus, intervals, fraction=0.1, seed=0):
    """Fit without a random `fraction` of the in-domain cells and return the errors [deg] at those cells."""
    rng = np.random.default_rng(seed)
    held = rng.random(tables.tip.size) < fraction
    fit = SplineFit(tables, rhombus, intervals, train_mask=~held)
    return fit.errors(tables, mask=held)


def summarize(tables, full, rhombus, intervals):
    """Return a dict with the fit metrics for one rhombus and interval count."""
    fit = SplineFit(tables, rhombus, intervals)
    err = fit.errors(tables)
    hold = holdout_errors(tables, rhombus, intervals)
    bil = bilinear_proxy_error(tables, rhombus)
    n = intervals + SPLINE_DEGREE
    return {
        "fit": fit,
        "max": np.max(np.abs(err)),
        "rms": np.sqrt(np.mean(err**2)),
        "holdout_max": np.max(np.abs(hold)),
        "bilinear_max": bil.max() if bil.size else float("nan"),
        "coeffs": n * n,
        "bytes": 4 * n * n,
        "area_kept": rhombus.area() / full.area(),
        "tip_range": (rhombus.psi0 - rhombus.a, rhombus.psi0 + rhombus.a),
        "tilt_range": (rhombus.phi0 - rhombus.b, rhombus.phi0 + rhombus.b),
        "cells": int(np.sum(fit.in_domain)),
    }


def sweep(tables, full, bands, intervals_list, modes):
    """Print a markdown table of the fit metrics over the band widths, interval counts and band modes."""
    print("| mode | band [deg] | N | coeffs | bytes | fit max [deg] | fit rms [deg] | holdout max [deg] "
          "| bilinear max [deg] | area kept | tip range [deg] | tilt range [deg] |")
    print("|---|---|---|---|---|---|---|---|---|---|---|---|")
    for mode in modes:
        for band in bands:
            rhombus = shrink(full, band, mode)
            for intervals in intervals_list:
                s = summarize(tables, full, rhombus, intervals)
                print(f"| {mode} | {band:.2f} | {intervals} | {s['coeffs']} | {s['bytes']} | {s['max']:.4f} "
                      f"| {s['rms']:.4f} | {s['holdout_max']:.4f} | {s['bilinear_max']:.4f} "
                      f"| {s['area_kept']:.3f} | [{s['tip_range'][0]:.2f}, {s['tip_range'][1]:.2f}] "
                      f"| [{s['tilt_range'][0]:.2f}, {s['tilt_range'][1]:.2f}] |")


def emit(tables, rhombus, intervals, out_dir, num_reference, seed):
    """Write the coefficient, domain, reference and table-point fixtures [rad] for the C++ and Python tests."""
    fit = SplineFit(tables, rhombus, intervals)
    d2r = np.pi / 180.0
    np.savetxt(os.path.join(out_dir, "splineCoefficients.csv"), (fit.coeffs.ravel() * d2r)[None, :],
               delimiter=",", fmt="%.9e")
    np.savetxt(os.path.join(out_dir, "splineDomain.csv"),
               np.array([[rhombus.psi0, rhombus.phi0, rhombus.a, rhombus.b]]) * d2r, delimiter=",", fmt="%.9e")

    # Random gimbal angles inside the rhombus, plus the four vertices
    rng = np.random.default_rng(seed)
    uv = rng.uniform(-1.0, 1.0, size=(num_reference, 2))
    uv = np.vstack([uv, [[1.0, 1.0], [1.0, -1.0], [-1.0, 1.0], [-1.0, -1.0]]])
    tip = rhombus.psi0 + rhombus.a * (uv[:, 0] + uv[:, 1]) / 2.0
    tilt = rhombus.phi0 + rhombus.b * (uv[:, 0] - uv[:, 1]) / 2.0
    theta1, theta2 = fit.motor_angles(tip, tilt)
    np.savetxt(os.path.join(out_dir, "splineReference.csv"),
               np.column_stack([tip, tilt, theta1, theta2]) * d2r, delimiter=",", fmt="%.9e",
               header="tip [rad], tilt [rad], theta1 [rad], theta2 [rad]")

    sel = fit.in_domain
    np.savetxt(os.path.join(out_dir, "tablePointsInDomain.csv"),
               np.column_stack([tables.tip[sel], tables.tilt[sel], tables.motor1[sel], tables.motor2[sel]]) * d2r,
               delimiter=",", fmt="%.9e", header="tip [rad], tilt [rad], theta1 [rad], theta2 [rad]")
    err = fit.errors(tables)
    print(f"Wrote fixtures to {out_dir}: N={intervals}, coeffs={fit.coeffs.size}, "
          f"max fit error vs table={np.max(np.abs(err)):.5f} deg")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--table1", default=os.path.join(DEFAULT_TABLE_DIR, "gimbal_to_motor1_angle.csv"))
    parser.add_argument("--table2", default=os.path.join(DEFAULT_TABLE_DIR, "gimbal_to_motor2_angle.csv"))
    parser.add_argument("--step", type=float, default=0.5, help="[deg] table grid step")
    parser.add_argument("--tip-col-offset", type=int, default=37, help="table column of zero tip")
    parser.add_argument("--tilt-row-offset", type=int, default=54, help="table row of zero tilt")
    parser.add_argument("--inscribe-tol", type=float, default=0.25,
                        help="[deg] allowed overhang of the rhombus past the populated cells of a row")
    parser.add_argument("--band", type=float, default=1.0, help="[deg] band width removed from the edges")
    parser.add_argument("--band-mode", choices=["plus-tip", "all"], default="plus-tip")
    parser.add_argument("--intervals", type=int, default=16, help="spline intervals per axis (N)")
    parser.add_argument("--sweep", action="store_true", help="print the band x N x mode study")
    parser.add_argument("--report", action="store_true", help="print the metrics for --band and --intervals")
    parser.add_argument("--emit", metavar="DIR", help="write the test fixtures for --band and --intervals")
    parser.add_argument("--num-reference", type=int, default=200, help="number of random reference points")
    parser.add_argument("--seed", type=int, default=2026)
    args = parser.parse_args()

    tables = MeasuredTables(load_table(args.table1), load_table(args.table2), args.step, args.tip_col_offset,
                            args.tilt_row_offset)
    full = inscribed_rhombus(tables, args.inscribe_tol)
    print(f"Motor symmetry error max |theta2(psi,phi) - theta1(psi,-phi)| = {tables.symmetry_error():.5f} deg")
    print(f"Inscribed rhombus: psi0={full.psi0:.3f} phi0={full.phi0:.3f} a={full.a:.3f} b={full.b:.3f} deg")
    print(f"All-cells bilinear proxy max error = {bilinear_proxy_error(tables, full).max():.4f} deg")

    if args.sweep:
        sweep(tables, full, [0.0, 0.5, 1.0, 1.5, 2.0, 3.0], [8, 12, 16, 20, 24], ["plus-tip", "all"])
    rhombus = shrink(full, args.band, args.band_mode)
    if args.report:
        sweep(tables, full, [args.band], [args.intervals], [args.band_mode])
    if args.emit:
        emit(tables, rhombus, args.intervals, args.emit, args.num_reference, args.seed)


if __name__ == "__main__":
    main()
