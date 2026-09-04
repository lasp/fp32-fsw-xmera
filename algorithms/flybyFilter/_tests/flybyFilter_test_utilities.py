# SPDX-License-Identifier: ISC
# Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#
# Plotting helpers for the flybyFilter Python integration test (only used when show_plots=True).

import numpy as np

try:
    import matplotlib.pyplot as plt
    _HAVE_MPL = True
except Exception:  # pragma: no cover - plotting is optional
    _HAVE_MPL = False

m2km = 1.0 / 1000.0


def energy(t, energy_series, testName, show_plots):
    if not (_HAVE_MPL and show_plots):
        return
    conserved = (energy_series - energy_series[0]) / energy_series[0]
    plt.figure(figsize=(10, 10))
    plt.plot(t, conserved, "b", label='Energy')
    plt.legend(loc='lower right')
    plt.title('Energy ' + testName)
    plt.grid()
    plt.show()
    plt.close()


def state_covar(x, Pflat, testName, show_plots):
    if not (_HAVE_MPL and show_plots):
        return
    numStates = len(x[0, :]) - 1
    P = np.zeros([len(Pflat[:, 0]), numStates, numStates])
    t = np.zeros(len(Pflat[:, 0]))
    for i in range(len(Pflat[:, 0])):
        t[i] = x[i, 0] * 1E-9
        P[i, :, :] = Pflat[i, 1:(numStates * numStates + 1)].reshape([numStates, numStates])

    labels = ['pos x (m)', 'pos y (m)', 'pos z (m)', 'vel x (m/s)', 'vel y (m/s)', 'vel z (m/s)']
    plt.figure(figsize=(10, 10))
    for k in range(numStates):
        plt.subplot(3, 2, k + 1)
        plt.plot(t, x[:, k + 1], "b")
        plt.plot(t, x[:, k + 1] + 3 * np.sqrt(P[:, k, k]), 'r--')
        plt.plot(t, x[:, k + 1] - 3 * np.sqrt(P[:, k, k]), 'r--')
        plt.title(labels[k] + ' ' + testName)
        plt.grid()
    plt.show()
    plt.close()


def post_fit_residuals(Res, noise, testName, show_plots):
    if not (_HAVE_MPL and show_plots):
        return
    t = Res[:, 0] * 1E-9
    plt.figure(figsize=(10, 10))
    for j in range(3):
        plt.subplot(3, 1, j + 1)
        plt.plot(t, Res[:, j + 1], "b.", label='Residual')
        plt.plot(t, 3 * noise * np.ones_like(t), 'r--')
        plt.plot(t, -3 * noise * np.ones_like(t), 'r--')
        plt.ylim([-10 * noise, 10 * noise])
        plt.title('Meas comp ' + str(j + 1) + ' ' + testName)
        plt.grid()
    plt.show()
    plt.close()


def two_orbits(r_true, r_est, show_plots):
    if not (_HAVE_MPL and show_plots):
        return
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.set_xlabel('$R_x$, km')
    ax.set_ylabel('$R_y$, km')
    ax.set_zlabel('$R_z$, km')
    ax.plot(r_true[:, 1] * m2km, r_true[:, 2] * m2km, r_true[:, 3] * m2km, 'dodgerblue', label="True orbit")
    ax.plot(r_est[:, 1] * m2km, r_est[:, 2] * m2km, r_est[:, 3] * m2km, 'salmon', label="Estimated orbit")
    ax.scatter(0, 0, 0, color='r')
    ax.set_title('Spacecraft Orbits')
    ax.legend()
    plt.show()
    plt.close()


def outlier_rejection(meas, truth, valid, testName, show_plots):
    """Plot the measurement stream fed to the filter, coloured by whether the filter kept it.

    meas  : [t_ns, m1, ... mN] the measurements actually written to the module
    truth : [t_ns, x1, ... xN] the truth the clean measurements follow
    valid : bool array aligned with the rows of meas; True where the filter applied the measurement

    The y-limits are deliberately left to auto-scale: the outliers sitting far off the truth curve
    is the point of the figure.
    """
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
    """Plot the estimate error against its ambient band, marking each injected outlier, so the
    spike and the decay back are readable directly.

    The tuning read is the height of each spike relative to the band (how much of the outlier the
    Kalman gain let through) and its width (how long the estimate took to settle).
    """
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
