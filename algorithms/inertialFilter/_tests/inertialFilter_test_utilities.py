import inspect
import os
import numpy as np

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('fswAlgorithms')

import matplotlib as mpl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Ellipse

color_x = 'dodgerblue'
color_y = 'salmon'
color_z = 'lightgreen'
m2km = 1.0 / 1000.0


def states(x, testName):
    numStates = len(x[0, :]) - 1

    t = np.zeros(len(x[:, 0]))
    for i in range(len(t)):
        t[i] = x[i, 0] * 1E-9

    plt.figure(num=None, figsize=(10, 10), dpi=80, facecolor='w', edgecolor='k')
    plt.subplot(321)
    plt.plot(t, x[:, 1], "b", label='Error Filter')
    plt.legend(loc='lower right')
    plt.title('First MRP component')
    plt.grid()

    plt.subplot(322)
    plt.plot(t, x[:, 4], "b")
    plt.title('First rate component (rad/s)')
    plt.grid()

    plt.subplot(323)
    plt.plot(t, x[:, 2], "b")
    plt.title('Second MRP component')
    plt.grid()

    plt.subplot(324)
    plt.plot(t, x[:, 5], "b")
    plt.xlabel('t(s)')
    plt.title('Second rate component (rad/s)')
    plt.grid()

    plt.subplot(325)
    plt.plot(t, x[:, 3], "b")
    plt.xlabel('t(s)')
    plt.title('Third MRP component')
    plt.grid()

    plt.subplot(326)
    plt.plot(t, x[:, 6], "b")
    plt.xlabel('t(s)')
    plt.title('Third rate component (rad/s)')
    plt.grid()

    return plt


def energy(t, energy, testName):
    conserved = np.zeros(len(t))
    for i in range(len(t)):
        conserved[i] = (energy[i] - energy[0]) / energy[0]

    plt.figure(num=None, figsize=(10, 10), dpi=80, facecolor='w', edgecolor='k')
    plt.plot(t, conserved, "b", label='Energy')
    plt.legend(loc='lower right')
    plt.title('Energy ' + testName)
    plt.grid()

    return plt


def state_covar(x, Pflat, testName):
    numStates = len(x[0, :]) - 1

    P = np.zeros([len(Pflat[:, 0]), numStates, numStates])
    t = np.zeros(len(Pflat[:, 0]))
    for i in range(len(Pflat[:, 0])):
        t[i] = x[i, 0] * 1E-9
        P[i, :, :] = Pflat[i, 1:(numStates * numStates + 1)].reshape([numStates, numStates])

    plt.figure(num=None, figsize=(10, 10), dpi=80, facecolor='w', edgecolor='k')
    plt.subplot(321)
    plt.plot(t, x[:, 1], "b", label='Error Filter')
    plt.plot(t, x[:, 1] + 3 * np.sqrt(P[:, 0, 0]), 'r--', label='Covar Filter')
    plt.plot(t, x[:, 1] - 3 * np.sqrt(P[:, 0, 0]), 'r--')
    plt.legend(loc='lower right')
    plt.title('First MRP component')
    plt.grid()

    plt.subplot(322)
    plt.plot(t, x[:, 4], "b")
    plt.plot(t, x[:, 4] + 3 * np.sqrt(P[:, 3, 3]), 'r--')
    plt.plot(t, x[:, 4] - 3 * np.sqrt(P[:, 3, 3]), 'r--')
    plt.title('First rate component (rad/s)')
    plt.grid()

    plt.subplot(323)
    plt.plot(t, x[:, 2], "b")
    plt.plot(t, x[:, 2] + 3 * np.sqrt(P[:, 1, 1]), 'r--')
    plt.plot(t, x[:, 2] - 3 * np.sqrt(P[:, 1, 1]), 'r--')
    plt.title('Second MRP component')
    plt.grid()

    plt.subplot(324)
    plt.plot(t, x[:, 5], "b")
    plt.plot(t, x[:, 5] + 3 * np.sqrt(P[:, 4, 4]), 'r--')
    plt.plot(t, x[:, 5] - 3 * np.sqrt(P[:, 4, 4]), 'r--')
    plt.xlabel('t(s)')
    plt.title('Second rate component (rad/s)')
    plt.grid()

    plt.subplot(325)
    plt.plot(t, x[:, 3], "b")
    plt.plot(t, x[:, 3] + 3 * np.sqrt(P[:, 2, 2]), 'r--')
    plt.plot(t, x[:, 3] - 3 * np.sqrt(P[:, 2, 2]), 'r--')
    plt.xlabel('t(s)')
    plt.title('Third MRP component')
    plt.grid()

    plt.subplot(326)
    plt.plot(t, x[:, 6], "b")
    plt.plot(t, x[:, 6] + 3 * np.sqrt(P[:, 5, 5]), 'r--')
    plt.plot(t, x[:, 6] - 3 * np.sqrt(P[:, 5, 5]), 'r--')
    plt.xlabel('t(s)')
    plt.title('Third rate component (rad/s)')
    plt.grid()

    return plt


def post_fit_residuals(Res, noise, testName):
    MeasNoise = np.zeros(len(Res[:, 0]))
    t = np.zeros(len(Res[:, 0]))
    for i in range(len(Res[:, 0])):
        t[i] = Res[i, 0] * 1E-9
        MeasNoise[i] = 3 * noise
        # Don't plot zero values, since they mean that no measurement is taken
        for j in range(len(Res[0, :]) - 1):
            if -1E-10 < Res[i, j + 1] < 1E-10:
                Res[i, j + 1] = np.nan

    plt.figure(num=None, figsize=(10, 10), dpi=80, facecolor='w', edgecolor='k')
    plt.subplot(311)
    plt.plot(t, Res[:, 1], "b.", label='Residual')
    plt.plot(t, MeasNoise, 'r--', label='Covar')
    plt.plot(t, -MeasNoise, 'r--')
    plt.legend(loc='lower right')
    plt.ylim([-10 * noise, 10 * noise])
    plt.title('First Meas Comp (m)')
    plt.grid()

    plt.subplot(312)
    plt.plot(t, Res[:, 2], "b.")
    plt.plot(t, MeasNoise, 'r--')
    plt.plot(t, -MeasNoise, 'r--')
    plt.ylim([-10 * noise, 10 * noise])
    plt.title('Second Meas Comp (m)')
    plt.grid()

    plt.subplot(313)
    plt.plot(t, Res[:, 3], "b.")
    plt.plot(t, MeasNoise, 'r--')
    plt.plot(t, -MeasNoise, 'r--')
    plt.ylim([-10 * noise, 10 * noise])
    plt.title('Third Meas Comp (m)')
    plt.grid()

    return plt


def two_orbits(r_BN, r_BN2):
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.set_xlabel('$R_x$, km')
    ax.set_ylabel('$R_y$, km')
    ax.set_zlabel('$R_z$, km')
    ax.plot(r_BN[:, 1] * m2km, r_BN[:, 2] * m2km, r_BN[:, 3] * m2km, color_x, label="True orbit")
    for i in range(len(r_BN2[:, 0])):
        if np.abs(r_BN2[i, 1]) > 0 or np.abs(r_BN2[i, 2]) > 0:
            ax.scatter(r_BN2[i, 1] * m2km, r_BN2[i, 2] * m2km, r_BN2[i, 3] * m2km, color=color_y, label="Meas orbit")
    ax.scatter(0, 0, color='r')
    ax.set_title('Spacecraft Orbits')

    return plt


def outlier_rejection(meas, truth, valid, testName):
    """Plot the measurement stream fed to the filter, coloured by whether the filter kept it.

    meas  : [t_ns, m1, ... mN] the measurements actually written to the module
    truth : [t_ns, x1, ... xN] the truth the clean measurements follow
    valid : bool array aligned with the rows of meas; True where the filter applied the measurement

    The y-limits are deliberately left to auto-scale: the rejected samples sitting far off the
    truth curve is the point of the figure.
    """
    numComponents = len(meas[0, :]) - 1
    numColumns = 1 if numComponents <= 3 else 2
    numRows = -(-numComponents // numColumns)

    t = meas[:, 0] * 1E-9
    tTruth = truth[:, 0] * 1E-9
    accepted = np.asarray(valid, dtype=bool)
    rejected = ~accepted

    plt.figure(num=None, figsize=(10, 10), dpi=80, facecolor='w', edgecolor='k')
    for j in range(numComponents):
        plt.subplot(numRows, numColumns, j + 1)
        plt.plot(tTruth, truth[:, j + 1], 'k', label='Truth')
        plt.plot(t[accepted], meas[accepted, j + 1], '.', color=color_x, label='Accepted')
        plt.plot(t[rejected], meas[rejected, j + 1], 'x', color='crimson', label='Rejected')
        if j == 0:
            plt.legend(loc='lower right')
            plt.title(testName + ': component 1 (' + str(int(np.count_nonzero(rejected))) + ' of '
                      + str(len(accepted)) + ' rejected)')
        else:
            plt.title('Component ' + str(j + 1))
        if j >= numComponents - numColumns:
            plt.xlabel('t(s)')
        plt.grid()
    # Multi-column grids otherwise overlap their titles with the axis above.
    plt.tight_layout()

    return plt


def error_recovery(err, outlierSteps, ambient, testName):
    """Plot the estimate error against its ambient band, marking each injected outlier, so the
    spike and the decay back are readable directly.

    err          : [t_ns, |error|] the error norm per step
    outlierSteps : indices into err at which an outlier was injected
    ambient      : the error level away from the outliers, drawn as a reference band

    The tuning read is the height of each spike relative to the band (how much of the outlier the
    Kalman gain let through) and its width (how long the estimate took to settle).
    """
    t = err[:, 0] * 1E-9
    steps = [k for k in outlierSteps if 0 <= k < len(err)]

    plt.figure(num=None, figsize=(10, 6), dpi=80, facecolor='w', edgecolor='k')
    plt.semilogy(t, err[:, 1], color=color_x, label='|error|')
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

    return plt
