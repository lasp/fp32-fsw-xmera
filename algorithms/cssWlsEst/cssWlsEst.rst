Executive Summary
-----------------
A weighted least-squares minimum-norm algorithm estimates the body-relative sun heading from a cluster of coarse sun
sensors (CSS). Using two successive sun heading evaluations the module also computes the inertial angular velocity
vector. Rotations about the sun heading are not observable, so that angular velocity contains only the body rates
orthogonal to the sun heading.

The estimator is intended for safe mode, where the spacecraft points its solar arrays at the Sun using a minimum set of
sensors in order to reach a power-positive state. With cosine-response CSS, at least three sensors must report a signal
for the heading to be uniquely determined; with one or two the algorithm falls back to a minimum-norm solution, and with
none it returns the zero vector.

All computation is single-precision (float32). Further background on the estimator is available in Steve O'Keefe's
PhD dissertation.

Module Architecture
-------------------

The module is split into two layers:

- The **adapter** (``cssWlsEst.h``/``.cpp``) is the Xmera ``SysModel``. It owns the message I/O, converts between the
  message payloads' C arrays and Eigen types, and holds the public configuration properties.
- The **algorithm** (``cssWlsEstAlgorithm.h``/``.cpp``) is the estimator proper. It has no framework or messaging
  dependency, takes Eigen types, returns its own output struct, and never throws.

A pure-C shim (``cssWlsEstAlgorithm_c.h``/``.cpp``) wraps the algorithm class for use by Ada/Adamant components via
FFI, with the shared C data model in ``cssWlsEstTypes.h``.

Adapter Layer
-------------

The adapter consumes the following messages and public configuration properties.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - cssDataInMsg
      - :ref:`CSSArraySensorMsgF32Payload`
      - CSS array measurement input, one cosine reading per sensor
    * - navStateOutMsg
      - :ref:`NavAttMsgF32Payload`
      - navigation output carrying the estimated sun heading and body rate
    * - cssWLSFiltResOutMsg
      - :ref:`SunlineFilterMsgF32Payload`
      - post-fit residuals and observation count; written only when the message is connected

The CSS constellation geometry is supplied as adapter properties rather than through a configuration message, so the
configuration is a pure function of the module's public state, readable and editable at any time rather than latched
from a message inside ``reset()``.

.. list-table:: Module Configuration Properties
    :widths: 20 15 10 15 40
    :header-rows: 1

    * - Name
      - Type
      - Units
      - Bounds
      - Description
    * - numCss
      - uint32_t
      - \-
      - [1, 32]
      - Number of configured coarse sun sensors
    * - cssNHat
      - Eigen::MatrixXf
      - \-
      - unit rows within 1e-3
      - Per-sensor boresight unit vectors in body frame, ``numCss`` rows by three columns; normalized when the
        configuration is built
    * - cssBias
      - Eigen::VectorXf
      - \-
      - >= 0, finite
      - Per-sensor calibration scale factor applied to the boresight
    * - useWeights
      - bool
      - \-
      - \-
      - Whether to weight the measurements in the least squares fit
    * - sensorUseThresh
      - float
      - \-
      - [-1, 1]
      - Cosine threshold at or below which a sensor reading is discarded

The module also publishes ``numActiveCss``, the number of sensors above the use threshold on the most recent cycle. It
is written by ``updateState()`` for telemetry and logging and is not a configuration input.

Two-phase initialization
~~~~~~~~~~~~~~~~~~~~~~~~

The module is constructed once. Set the public properties, then call ``reset()`` at startup; it validates the message
links, builds the validated configuration, and constructs the algorithm. On a state transition the flight software
calls ``reInitialize()`` rather than ``reset()``. ``reconfigure()`` rebuilds the configuration from edited properties
without disturbing the estimator's runtime state.

.. code-block:: python

    module = cssWlsEstF32.CssWlsEst()
    module.numCss = 8
    module.cssNHat = cssOrientationList
    module.cssBias = [1.0] * 8
    module.useWeights = True
    module.sensorUseThresh = 0.15
    module.cssDataInMsg.subscribeTo(cssDataInMsg)
    module.reset(0)

Calling ``updateState()`` before ``reset()`` raises ``XmeraLifecycleException``.

Algorithm Layer
---------------

Mathematical Formulation
~~~~~~~~~~~~~~~~~~~~~~~~

Each cycle the algorithm selects the active sensors, those whose reading exceeds ``sensorUseThresh``. For each active
sensor :math:`i` it forms a row of the observation matrix from the calibrated boresight, and the corresponding entry of
the observation vector from the measurement:

.. math::

    \mathbf{H}_i = c_i \hat{\mathbf{n}}_i, \qquad y_i = \cos\theta_i

where :math:`c_i` is the sensor bias and :math:`\hat{\mathbf{n}}_i` its body-frame boresight. The active measurements
are compacted, so the row index counts active sensors rather than sensor slots.

Sun Heading Evaluation
~~~~~~~~~~~~~~~~~~~~~~

The fit depends on how many sensors are active, because the problem is over-determined only from three measurements up:

- **Three or more active sensors.** A true weighted least squares fit, where the weights are the measurements
  themselves so that the best-illuminated sensors are trusted most:

  .. math::

      \mathbf{d} = \left( \mathbf{H}^T \mathbf{W} \mathbf{H} \right)^{-1} \mathbf{H}^T \mathbf{W} \mathbf{y}

  With ``useWeights`` false, :math:`\mathbf{W}` is the identity.

- **Two active sensors.** The system is underdetermined, so the minimum-norm solution is taken. The weights carry no
  information in this case and are not applied:

  .. math::

      \mathbf{d} = \mathbf{H}^T \left( \mathbf{H} \mathbf{H}^T \right)^{-1} \mathbf{y}

- **One active sensor.** No fit is possible; the heading is only known to lie on a cone about the boresight. The
  algorithm returns the scaled boresight :math:`\mathbf{d} = y_0 \mathbf{H}_0` as a best guess.

- **No active sensors.** The sun cannot be estimated and the zero vector is returned.

The fit is then normalized to give the reported heading. The post-fit residuals are computed against the
**unnormalized** fit, before normalization.

Partial Angular Velocity Evaluation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Two successive heading evaluations give a partial solution for the inertial angular velocity. Rates about the sun
heading are unobservable; rates about the other two axes are recovered. With :math:`\mathbf{d}_n` the current heading,
:math:`\mathbf{d}_{n-1}` the prior heading and :math:`\Delta t` the elapsed time:

.. math::

    \boldsymbol{\omega}_{B/N} = \frac{\mathbf{d}_n \times \mathbf{d}_{n-1}}{|\mathbf{d}_n \times \mathbf{d}_{n-1}|}
    \arccos\left( \frac{\mathbf{d}_n \cdot \mathbf{d}_{n-1}}{|\mathbf{d}_n| |\mathbf{d}_{n-1}|} \right)
    \frac{1}{\Delta t}

All components are body frame. The arc-cosine argument is clamped to :math:`[-1, 1]` so that round-off cannot push it
outside the domain.

Post-Fit Residuals
~~~~~~~~~~~~~~~~~~

Residuals measure how well the estimate explains the measurements. For each configured sensor the estimate is projected
onto the raw boresight, without the bias, and differenced against the observation:

.. math::

    r_i = \cos\theta_i - \max\left(0, \hat{\mathbf{d}} \cdot \hat{\mathbf{n}}_i\right)

The predicted value is floored at zero because a coarse sun sensor cannot report a negative cosine. Residuals are
computed for every configured sensor, not only the active ones; entries beyond ``numCss`` remain zero.

Edge Case Guards
~~~~~~~~~~~~~~~~

**Indeterminate normal matrix.** The two- and three-or-more-sensor branches both invert a matrix built from the sensor
geometry. When that matrix is singular, as it is for a collinear or coplanar constellation, the fit has no unique
solution. The determinant is tested against a *relative* tolerance, the factor times the matrix norm raised to the
matrix dimension, which keeps the test scale invariant. On failure the heading and rate are zeroed.

The factor is sized at a few multiples of the working precision's machine epsilon, which is where round-off in the
computed determinant itself lands. A fixed absolute threshold cannot serve here: set below that floor it catches only
an exactly-zero determinant and lets near-singular geometry produce an unguarded inverse, and set above it it rejects
a well-scaled constellation.

**Indeterminate rotation axis.** The rate axis is :math:`(\mathbf{d}_n \times \mathbf{d}_{n-1})` normalized, and
that cross product vanishes when successive headings are collinear or antipodal. Because normalization is
discontinuous about its zero guard, arbitrarily small rounding noise flips the reported axis between zero and a full
unit vector, and the reported rate between zero and :math:`\pi / \Delta t`.

A zero rate is returned only when that cross product is essentially exactly zero, not when the headings are merely
*nearly* collinear, so a 180 degree heading reversal reports the correct rate magnitude about an arbitrary axis.
**The algorithm should be changed in future** to detect an indeterminate rotation axis explicitly instead of relying
on exact cancellation. Note that a 180 degree reversal within one control cycle corresponds to a body rate far
outside the nominal envelope, so this is a fault-condition input rather than a nominal one.

**First call and zero time step.** The prior time starts at zero and is only set at the end of an update, so no rate is
produced on the first call after construction or re-initialization. A repeated timestamp likewise gives
:math:`\Delta t = 0` and yields no rate rather than dividing by zero.

Algorithm Assumptions and Limitations
-------------------------------------

- At least three active sensors are required for a unique heading. With two the result is the minimum-norm solution,
  and with one it is a point on a cone, which can be far from the true heading. Callers should treat ``numActiveCss``
  as a quality indicator.
- Rates about the sun heading are structurally unobservable. The reported angular velocity is only the component
  orthogonal to the heading.
- Sensor biases are applied to the observation matrix but not to the residual projection, so a biased sensor's
  residual is measured against the raw boresight.
