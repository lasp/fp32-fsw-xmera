Executive Summary
-----------------
This module estimates the body-frame Sun-heading direction, the body angular rate, and a coarse sun
sensor (CSS) intensity bias using a square-root unscented Kalman filter (SRuKF), fusing CSS array
measurements and gyro rates on a single measurement timeline. All computation is double precision.

The module is split into an xmera ``SysModel`` adapter (``SunlineSRuKF``) and a framework-agnostic
algorithm (``SunlineSRuKFAlgorithm``). The algorithm owns a ``filtering::SRuKF`` from
:ref:`filteringCore`. Configuration is validated through an immutable ``SunlineSRuKFConfig`` and applied
with a two-phase initialization lifecycle: the host sets the adapter's public properties, then
``reset()`` builds the validated config and seeds the filter.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages. The msg type contains a link to the
message structure definition, while the description provides information on what this message is used
for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - navAttInMsg
      - :ref:`NavAttMsgPayload`
      - Input gyro message
    * - cssDataInMsg
      - :ref:`CSSArraySensorMsgPayload`
      - Input message containing the direction cosines of the Sun as seen by the CSSs
    * - cssConfigInMsg
      - :ref:`CSSConfigMsgPayload`
      - Input message containing the geometry of the CSS constellation; latched at ``reset()``
    * - navAttOutMsg
      - :ref:`NavAttMsgPayload`
      - Output message containing the estimated Sun vector in body-frame coordinates
    * - filterOutMsg
      - :ref:`FilterMsgPayload`
      - Output message with the filter estimated state and covariance
    * - filterGyroResOutMsg
      - :ref:`FilterResidualsMsgPayload`
      - Output message containing pre- and post-fit residuals for the gyro measurements
    * - filterCssResOutMsg
      - :ref:`FilterResidualsMsgPayload`
      - Output message containing pre- and post-fit residuals for the CSS measurements


Detailed Module Description
---------------------------
The estimated state is a 7-element vector combining the Sun-heading unit vector and the body angular
rate, both in body-frame coordinates, plus a scalar bias :math:`b` that models the varying solar
intensity which scales the CSS measurements:

.. math::
    \boldsymbol{x} = \left\{ \begin{matrix} {}^\mathcal{B}\boldsymbol{\hat{s}} \\
    {}^\mathcal{B}\boldsymbol{\omega} \\ b \end{matrix} \right\}.

Dynamics model
++++++++++++++
The sun heading is fixed in inertial coordinates; it only changes in body-frame coordinates due to the
motion of the spacecraft. Therefore, the dynamics of the sun heading is given by the body-frame
derivative of the sun-heading unit-direction vector. The derivative of the angular rate vector is set to
zero in this module, and the bias state has no dynamics. This gives:

.. math::
    \boldsymbol{\dot{x}} = \left\{ \begin{matrix} {}^\mathcal{B}\boldsymbol{\hat{s}} \times
    {}^\mathcal{B}\boldsymbol{\omega} \\ \boldsymbol{0} \\ 0 \end{matrix} \right\}.

Measurement model
+++++++++++++++++
Two types of measurements are processed by this filter: gyro measurements and CSS measurements. For gyro
measurements, the gyro rates are mapped directly to the rate component of the state via a
:math:`3 \times 3` identity matrix, which constitutes the measurement model.

For the CSS measurement, the measurement model is constituted by the :math:`n \times 3` matrix
:math:`[\boldsymbol{H}]`, where :math:`n` is the number of CSSs active at that moment. With
:math:`{}^\mathcal{B}\boldsymbol{\hat{n}}_i` being the boresight of the :math:`i`-th CSS, the
measurement model is given by:

.. math::
    [\boldsymbol{H}] = \left[ \begin{matrix} {}^\mathcal{B}\boldsymbol{\hat{n}}_1 \\ \vdots \\
    {}^\mathcal{B}\boldsymbol{\hat{n}}_n \end{matrix} \right].

With the bias state :math:`b` and the current heading estimate :math:`\hat{\boldsymbol{s}}`, the
predicted CSS measurements are:

.. math::
    yMeas = b\,[\boldsymbol{H}]\,\hat{\boldsymbol{s}}


Module Architecture
-------------------
``SunlineSRuKFAlgorithm`` is framework-agnostic: it holds a ``filtering::SRuKF`` and a validated
``SunlineSRuKFConfig``, owns the ``measurement_queue``, and exposes ``update()``, ``reInitialize()``,
and ``reInitializeAll()``. ``SunlineSRuKF`` is the xmera adapter: it owns the message ports, converts
payloads to/from the algorithm's Eigen types, and drives the lifecycle.

Configuration is immutable once built. ``SunlineSRuKFConfig::create(...)`` validates every constrained
parameter and throws on invalid input; the algorithm trusts the config thereafter. The constant filter
parameters are pushed into the SRuKF by ``setConfig()`` (which calls the SRuKF's ``reConfigure()`` to
re-derive the sigma-point spread, weights, and process-noise Cholesky), so a configuration change takes
effect immediately while preserving the current estimate.

Lifecycle
+++++++++
The adapter follows a two-phase initialization:

- **Phase 1** — the host sets the public configuration properties (see the User Guide) and connects the
  input/output messages.
- **Phase 2** — ``reset(callTime)`` validates that the input messages are connected, latches the CSS
  geometry (boresights, per-sensor scale factors, count) from ``cssConfigInMsg``, builds and validates a
  ``SunlineSRuKFConfig``, and constructs the algorithm (which seeds the filter state and covariance).

Two runtime reset entry points are exposed on both the algorithm and the adapter:

- ``reInitialize()`` clears the internal runtime (the pending-measurement queue and the residual
  snapshots) while **preserving** the filter state and covariance.
- ``reInitializeAll()`` performs ``reInitialize()`` and additionally re-seeds the filter state and
  covariance from the configured initial values.

Configuration parameters
+++++++++++++++++++++++++
.. list-table:: Configuration parameters and valid ranges
    :widths: 30 50 20
    :header-rows: 1

    * - Property
      - Description
      - Valid range
    * - alpha
      - sigma-point spread tunable
      - any
    * - beta
      - prior-knowledge tunable
      - any
    * - processNoise
      - N x N process noise covariance Q
      - positive semi-definite
    * - initialState
      - N-element initial state seed
      - any
    * - initialCovariance
      - N x N initial covariance P0
      - positive semi-definite
    * - biasLowerBound
      - lower clamp on the bias state
      - > 0 and < biasUpperBound
    * - biasUpperBound
      - upper clamp on the bias state
      - > 0 and > biasLowerBound
    * - sensorThreshold
      - minimum cosValue for a CSS to be counted active
      - >= 0
    * - cssMeasurementNoiseStd
      - CSS measurement noise standard deviation
      - >= 0
    * - gyroMeasurementNoiseStd
      - gyro measurement noise standard deviation
      - >= 0

The CSS geometry is not set directly: ``numberOfCss`` (:math:`0 \le n \le` ``MaxCss``), the per-sensor
boresights (each of the first ``numberOfCss`` rows must be unit length within 1e-3), and the per-sensor
scale factors (each :math:`\ge 0`) are latched from ``cssConfigInMsg`` at ``reset()``.


User Guide
----------
Set the public properties, connect the messages, and let ``reset()`` build and validate the
configuration::

    filter = sunlineSRuKFF32.SunlineSRuKF()

    filter.alpha = 0.02
    filter.beta = 2.0
    filter.initialState = [0.0, 0.0, 1.0, 0.02, -0.005, 0.01, 0.6]
    filter.initialCovariance = [[0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                [0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0],
                                [0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0],
                                [0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0],
                                [0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0],
                                [0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0],
                                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5]]
    filter.biasLowerBound = 0.5
    filter.biasUpperBound = 1.5
    filter.sensorThreshold = 0.0
    filter.cssMeasurementNoiseStd = 0.01
    filter.gyroMeasurementNoiseStd = 0.001
    sigmaSun = (1e-6) ** 2
    sigmaRate = (1e-8) ** 2
    sigmaBias = (1e-5) ** 2
    filter.processNoise = [[sigmaSun, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                           [0.0, sigmaSun, 0.0, 0.0, 0.0, 0.0, 0.0],
                           [0.0, 0.0, sigmaSun, 0.0, 0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0, sigmaRate, 0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0, 0.0, sigmaRate, 0.0, 0.0],
                           [0.0, 0.0, 0.0, 0.0, 0.0, sigmaRate, 0.0],
                           [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, sigmaBias]]

    # Connect the input/output messages (navAttInMsg, cssDataInMsg, cssConfigInMsg, ...), then the
    # simulation calls reset() once before stepping. To restart the filter at runtime, call
    # reInitializeAll() (state + covariance reset to the configured seed) or reInitialize() (keep the
    # current estimate, clear only the pending measurements and residual snapshots).
