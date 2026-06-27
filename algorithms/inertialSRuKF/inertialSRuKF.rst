Executive Summary
-----------------
This module estimates the inertial-to-body attitude (as a Modified Rodrigues Parameter set) and the body
angular rate using a square-root unscented Kalman filter (SRuKF), fusing star-tracker attitude
measurements and gyro rates on a single measurement timeline. All computation is double precision.

The module is split into an xmera ``SysModel`` adapter (``InertialSRuKF``) and a framework-agnostic
algorithm (``InertialSRuKFAlgorithm``). The algorithm owns a ``filtering::SRuKF`` from
:ref:`filteringCore`. Configuration is validated through an immutable ``InertialSRuKFConfig`` and applied
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
    * - stAttInMsg
      - :ref:`STAttMsgPayload`
      - Input star-tracker attitude measurement (inertial-to-body MRP); required
    * - gyrBuffInMsg
      - :ref:`AccDataMsgPayload`
      - Input gyro buffer; the first packet's body-frame rate is used as the gyro measurement (optional)
    * - navAttOutMsg
      - :ref:`NavAttMsgPayload`
      - Output message containing the estimated attitude and body rate
    * - filterOutMsg
      - :ref:`FilterMsgPayload`
      - Output message with the filter estimated state and covariance
    * - filterStResOutMsg
      - :ref:`FilterResidualsMsgPayload`
      - Output message containing pre- and post-fit residuals for the star-tracker measurements
    * - filterGyroResOutMsg
      - :ref:`FilterResidualsMsgPayload`
      - Output message containing pre- and post-fit residuals for the gyro measurements


Detailed Module Description
---------------------------
The estimated state is a 6-element vector combining the inertial-to-body MRP attitude and the body
angular rate, both expressed in body-frame coordinates:

.. math::
    \boldsymbol{x} = \left\{ \begin{matrix} \boldsymbol{\sigma}_{B/N} \\
    {}^\mathcal{B}\boldsymbol{\omega}_{B/N} \end{matrix} \right\}.

Dynamics model
++++++++++++++
The attitude evolves with the MRP kinematic differential equation. The derivative of the angular rate
vector is set to zero in this module, so the rate behaves as a random walk driven by the process noise:

.. math::
    \boldsymbol{\dot{x}} = \left\{ \begin{matrix} \frac{1}{4} [B(\boldsymbol{\sigma}_{B/N})]\,
    {}^\mathcal{B}\boldsymbol{\omega}_{B/N} \\ \boldsymbol{0} \end{matrix} \right\}.

Measurement model
+++++++++++++++++
Two types of measurements are processed by this filter: gyro measurements and star-tracker attitude
measurements. For gyro measurements, the gyro rates are mapped directly to the rate component of the
state via a :math:`3 \times 3` identity matrix.

For the star-tracker measurement, the measured MRP attitude is compared against the attitude component of
the state. The innovation is formed with the relative MRP difference (``subMrp``) and applied with a
linear state correction; together with the linear sigma-point means of the unscented transform this is
valid for moderate attitude errors. Unlike the Basilisk inertialUKF, the unscented transform here does
not switch MRP sigma points across the :math:`|\boldsymbol{\sigma}| = 1` boundary, so the filter is
intended to operate away from the MRP shadow-set singularity.


Module Architecture
-------------------
``InertialSRuKFAlgorithm`` is framework-agnostic: it holds a ``filtering::SRuKF`` and a validated
``InertialSRuKFConfig``, owns the ``measurement_queue``, and exposes ``update()``, ``reInitialize()``,
and ``reInitializeAll()``. ``InertialSRuKF`` is the xmera adapter: it owns the message ports, converts
payloads to/from the algorithm's Eigen types, and drives the lifecycle.

Configuration is immutable once built. ``InertialSRuKFConfig::create(...)`` validates every constrained
parameter and throws on invalid input; the algorithm trusts the config thereafter. The constant filter
parameters are pushed into the SRuKF by ``setConfig()`` (which calls the SRuKF's ``reConfigure()`` to
re-derive the sigma-point spread, weights, and process-noise Cholesky), so a configuration change takes
effect immediately while preserving the current estimate.

Lifecycle
+++++++++
The adapter follows a two-phase initialization:

- **Phase 1** — the host sets the public configuration properties (see the User Guide) and connects the
  input/output messages.
- **Phase 2** — ``reset(callTime)`` validates that ``stAttInMsg`` is connected, builds and validates an
  ``InertialSRuKFConfig``, and constructs the algorithm (which seeds the filter state and covariance).

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
    * - stMeasurementNoiseStd
      - star-tracker attitude measurement noise standard deviation
      - >= 0
    * - gyroMeasurementNoiseStd
      - gyro measurement noise standard deviation
      - >= 0


User Guide
----------
Set the public properties, connect the messages, and let ``reset()`` build and validate the
configuration::

    filter = inertialSRuKFF32.InertialSRuKF()

    filter.alpha = 0.02
    filter.beta = 2.0
    filter.initialState = [0.0, 0.0, 0.0, 0.02, -0.005, 0.01]
    filter.initialCovariance = (1e-4 * np.identity(6)).tolist()
    filter.stMeasurementNoiseStd = 1e-4
    filter.gyroMeasurementNoiseStd = 0.001
    sigmaAtt = (1e-7) ** 2
    sigmaRate = (1e-8) ** 2
    processNoise = np.zeros([6, 6])
    np.fill_diagonal(processNoise, [sigmaAtt] * 3 + [sigmaRate] * 3)
    filter.processNoise = processNoise.tolist()

    # Connect the input/output messages (stAttInMsg required, gyrBuffInMsg optional, ...), then the
    # simulation calls reset() once before stepping. To restart the filter at runtime, call
    # reInitializeAll() (state + covariance reset to the configured seed) or reInitialize() (keep the
    # current estimate, clear only the pending measurements and residual snapshots).
