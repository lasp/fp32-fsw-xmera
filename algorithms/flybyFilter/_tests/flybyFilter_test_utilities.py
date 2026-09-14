# SPDX-License-Identifier: ISC
# Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#
# Plotting helpers for the flybyFilter Python integration tests (only used when show_plots=True).
#
# Unlike the inertialFilter and sunlineFilter equivalents, every entry point takes show_plots and
# guards on it internally, so the callers stay free of `if show_plots:` blocks. Matplotlib is
# imported lazily-ish: a missing backend disables plotting rather than failing the test run.

import numpy as np

try:
    import matplotlib.pyplot as plt
    _HAVE_MPL = True
except Exception:  # pragma: no cover - plotting is optional
    _HAVE_MPL = False

m2km = 1.0 / 1000.0

STATE_LABELS = ['pos x (m)', 'pos y (m)', 'pos z (m)', 'vel x (m/s)', 'vel y (m/s)', 'vel z (m/s)']


def _enabled(show_plots):
    return _HAVE_MPL and show_plots


def _seconds(nanos):
    return np.asarray(nanos) * 1E-9


def states(x, testName, show_plots):
    """Per-component state (or state-error) traces. `x` is [time_ns, s0..s5] per row."""
    if not _enabled(show_plots):
        return
    t = _seconds(x[:, 0])
    num_states = x.shape[1] - 1

    plt.figure(figsize=(10, 10))
    for k in range(num_states):
        plt.subplot(3, 2, k + 1)
        plt.plot(t, x[:, k + 1], "b")
        plt.xlabel('t (s)')
        plt.title(STATE_LABELS[k] + ' ' + testName)
        plt.grid()
    plt.tight_layout()
    plt.show()
    plt.close()


def energy(t, energy_series, testName, show_plots):
    """Relative drift of the two-body specific orbital energy; a flat trace means the propagation
    conserved energy."""
    if not _enabled(show_plots):
        return
    conserved = (energy_series - energy_series[0]) / energy_series[0]
    plt.figure(figsize=(10, 10))
    plt.plot(t, conserved, "b", label='Energy')
    plt.legend(loc='lower right')
    plt.xlabel('t (s)')
    plt.title('Relative energy drift ' + testName)
    plt.grid()
    plt.show()
    plt.close()


def state_covar(x, Pflat, testName, show_plots):
    """State traces with +/-3 sigma envelopes taken from the flattened covariance rows."""
    if not _enabled(show_plots):
        return
    num_states = x.shape[1] - 1
    t = _seconds(Pflat[:, 0])
    P = np.zeros([Pflat.shape[0], num_states, num_states])
    for i in range(Pflat.shape[0]):
        P[i, :, :] = Pflat[i, 1:(num_states * num_states + 1)].reshape([num_states, num_states])

    plt.figure(figsize=(10, 10))
    for k in range(num_states):
        sigma = np.sqrt(np.abs(P[:, k, k]))
        plt.subplot(3, 2, k + 1)
        plt.plot(t, x[:, k + 1], "b")
        plt.plot(t, x[:, k + 1] + 3 * sigma, 'r--')
        plt.plot(t, x[:, k + 1] - 3 * sigma, 'r--')
        plt.xlabel('t (s)')
        plt.title(STATE_LABELS[k] + ' ' + testName)
        plt.grid()
    plt.tight_layout()
    plt.show()
    plt.close()


def covar_trace(Pflat, num_states, testName, show_plots):
    """Total uncertainty over time -- the quantity the covariance assertions actually check."""
    if not _enabled(show_plots):
        return
    t = _seconds(Pflat[:, 0])
    trace = np.array([
        np.trace(Pflat[i, 1:(num_states * num_states + 1)].reshape([num_states, num_states]))
        for i in range(Pflat.shape[0])
    ])

    plt.figure(figsize=(10, 6))
    plt.semilogy(t, np.abs(trace), "b")
    plt.xlabel('t (s)')
    plt.ylabel('trace(P)')
    plt.title('Covariance trace ' + testName)
    plt.grid()
    plt.show()
    plt.close()


def post_fit_residuals(Res, noise, testName, show_plots):
    """Residual scatter against a +/-3 sigma band.

    Rows where the filter took no measurement are written as exact zeros by the adapter; they are
    replaced with NaN so the gaps are left blank instead of drawing a spurious line at zero.
    """
    if not _enabled(show_plots):
        return
    residuals = np.array(Res, dtype=float, copy=True)
    t = _seconds(residuals[:, 0])
    blank = np.abs(residuals[:, 1:]) < 1E-10
    residuals[:, 1:][blank] = np.nan

    plt.figure(figsize=(10, 10))
    for j in range(residuals.shape[1] - 1):
        plt.subplot(3, 1, j + 1)
        plt.plot(t, residuals[:, j + 1], "b.", label='Residual')
        plt.plot(t, 3 * noise * np.ones_like(t), 'r--', label='3 sigma')
        plt.plot(t, -3 * noise * np.ones_like(t), 'r--')
        if j == 0:
            plt.legend(loc='lower right')
        plt.ylim([-10 * noise, 10 * noise])
        plt.xlabel('t (s)')
        plt.title('Heading component ' + str(j + 1) + ' ' + testName)
        plt.grid()
    plt.tight_layout()
    plt.show()
    plt.close()


def two_orbits(r_true, r_est, show_plots):
    """Truth and estimated trajectories in the inertial frame, with the central body at the origin."""
    if not _enabled(show_plots):
        return
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.set_xlabel('$R_x$, km')
    ax.set_ylabel('$R_y$, km')
    ax.set_zlabel('$R_z$, km')
    ax.set_title('Spacecraft Orbits')
    ax.plot(r_true[:, 1] * m2km, r_true[:, 2] * m2km, r_true[:, 3] * m2km, 'dodgerblue', label="True orbit")
    ax.plot(r_est[:, 1] * m2km, r_est[:, 2] * m2km, r_est[:, 3] * m2km, 'salmon', label="Estimated orbit")
    ax.scatter(0, 0, 0, color='r')
    ax.legend()
    plt.show()
    plt.close()


def outlier_rejection(meas, truth, valid, testName, show_plots):
    """Plot the measurement stream against truth, coloured by whether the filter used each sample.
    meas and truth are [t_ns, m1..mN]; valid is a bool mask over the rows of meas."""
    if not (_HAVE_MPL and show_plots):
        return
    numComponents = len(meas[0, :]) - 1
    numColumns = 1 if numComponents <= 3 else 2
    numRows = -(-numComponents // numColumns)
    t = meas[:, 0] * 1E-9
    tTruth = truth[:, 0] * 1E-9
    accepted = np.asarray(valid, dtype=bool)
    rejected = ~accepted

    plt.figure(figsize=(10, 10))
    for j in range(numComponents):
        plt.subplot(numRows, numColumns, j + 1)
        plt.plot(tTruth, truth[:, j + 1], 'k', label='Truth')
        plt.plot(t[accepted], meas[accepted, j + 1], '.', color='dodgerblue', label='Used')
        plt.plot(t[rejected], meas[rejected, j + 1], 'x', color='crimson', label='Rejected')
        if j == 0:
            plt.legend(loc='lower right')
        plt.title('Meas comp ' + str(j + 1) + ' ' + testName)
        plt.grid()
    plt.tight_layout()
    plt.show()
    plt.close()


def error_recovery(err, outlierSteps, ambient, testName, show_plots):
    """Plot the error norm against its ambient band, marking each injected outlier.
    err is [t_ns, |error|]; outlierSteps indexes its rows."""
    if not (_HAVE_MPL and show_plots):
        return
    t = err[:, 0] * 1E-9
    steps = [k for k in outlierSteps if 0 <= k < len(err)]

    plt.figure(figsize=(10, 6))
    plt.semilogy(t, err[:, 1], color='dodgerblue', label='|error|')
    plt.semilogy(t, np.full(len(t), ambient), 'k', label='ambient')
    plt.semilogy(t, np.full(len(t), 2 * ambient), '--', color='grey', label='2x ambient')
    for n, k in enumerate(steps):
        plt.axvline(t[k], color='crimson', alpha=0.5, label='outlier injected' if n == 0 else None)
    plt.legend(loc='upper right')
    plt.xlabel('t(s)')
    plt.ylabel('|error|')
    plt.title(testName + ': perturbation and recovery')
    plt.grid()
    plt.tight_layout()
    plt.show()
    plt.close()
