Executive Summary
-----------------
This module estimates the inertial-to-body attitude (as a Modified Rodrigues Parameter set) and the body
angular rate using a square-root unscented Kalman filter (SRuKF), fusing star-tracker attitude
measurements and gyro rates on a single measurement timeline. All computation is double precision.

The module is split into an xmera ``SysModel`` adapter (``InertialFilter``) and a framework-agnostic
algorithm (``InertialFilterAlgorithm``). The algorithm is a thin, problem-specific layer over the
reusable :ref:`filtering-architecture` framework: it wires the inertial state, dynamics, and measurement
models into a ``filtering::SRuKF`` and drains its measurements through the framework's robust scheduler
(see `Relation to filteringCore`_). Configuration is validated through an immutable
``InertialFilterConfig`` and applied with a two-phase initialization lifecycle: the host sets the
adapter's public properties, then ``reset()`` builds the validated config and seeds the filter.

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
angular rate, both expressed in body-frame coordinates. It is composed from filteringCore state tags as
``filtering::StateVector<filtering::MrpAttitude<3>, filtering::AngularRate<3>>``:

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
measurements. Each is expressed as a small model type satisfying the filteringCore ``Measurement``
concept (``observation()``, ``model()``, ``noise()``, ``subtract()``), which the ``SRuKF`` consumes
generically. For gyro measurements, the gyro rates are mapped directly to the rate component of the state
via a :math:`3 \times 3` identity matrix, and the innovation is the plain vector difference.

For the star-tracker measurement, the measured MRP attitude is compared against the attitude component of
the state. The innovation is formed with the relative MRP difference (``subMrp`` in ``subtract()``) and
applied with a linear state correction; together with the linear sigma-point means of the unscented
transform this is valid for moderate attitude errors. Unlike the Basilisk inertialUKF, the unscented
transform here does not switch MRP sigma points across the :math:`|\boldsymbol{\sigma}| = 1` boundary, so
the filter is intended to operate away from the MRP shadow-set singularity. After each ``update()`` the
resulting state is regularized (``mrpSwitch``) to the inner MRP set so :math:`|\boldsymbol{\sigma}| \le 1`
is maintained, without altering the represented attitude.


Module Architecture
-------------------
``InertialFilterAlgorithm`` is framework-agnostic: it holds a ``filtering::SRuKF`` and a validated
``InertialFilterConfig``, owns the ``measurement_queue``, and exposes ``update()``, ``reInitializeExceptPersistentStates()``,
and ``reInitialize()``. On each ``update()`` it enqueues whichever measurements are fresh, drains the
queue through the filteringCore ``applySequentialRobust`` scheduler, and regularizes the resulting MRP
state. ``InertialFilter`` is the xmera adapter: it owns the message ports, converts payloads to/from the
algorithm's Eigen types, and drives the lifecycle.

Configuration is immutable once built. ``InertialFilterConfig::create(...)`` validates every constrained
parameter and throws on invalid input; the algorithm trusts the config thereafter. The constant filter
parameters are pushed into the SRuKF by ``setConfig()`` (which calls the SRuKF's ``configure()`` to
re-derive the sigma-point spread, weights, and process-noise Cholesky), so a configuration change takes
effect immediately while preserving the current estimate.

Relation to filteringCore
+++++++++++++++++++++++++
This module is a concrete instantiation of the :ref:`filtering-architecture` building blocks; it supplies
the problem-specific types and lets the core provide the estimator machinery. The mapping is:

.. list-table:: filteringCore primitives used by this module
    :widths: 35 65
    :header-rows: 1

    * - filteringCore primitive
      - Inertial-filter specialization
    * - ``StateVector<Components...>``
      - ``StateVector<MrpAttitude<3>, AngularRate<3>>`` — the 6-element attitude/rate state
    * - ``Dynamics`` concept
      - ``InertialDynamics`` — the MRP kinematics functor (rate held constant)
    * - ``Measurement`` concept
      - ``StAttMeasurementModel`` and ``RateMeasurementModel`` (``observation``/``model``/``noise``/``subtract``)
    * - ``SRuKF<State, Dyn>``
      - ``SRuKF<InertialState, InertialDynamics>`` — the square-root UKF that owns the estimate
    * - ``measurement_queue<Measurement, N>``
      - time-ordered buffer of the per-cycle star-tracker and gyro measurements
    * - ``applySequentialRobust``
      - drains the queue, one ``timeUpdate`` + ``measurementUpdate`` per measurement, holding the anchor

Because the module selects the *robust* scheduler, ``InertialFilterAlgorithm`` fulfils that scheduler's
contract: its ``timeUpdate`` and ``measurementUpdate`` return a ``bool`` validity, and it implements
``clear()`` to roll back to the last-good state after a bad update. The config validators reuse the
``SRuKF`` static checks (``alphaIsValid``/``betaIsValid``) so the valid ranges stay defined in one place.

Lifecycle
+++++++++
The adapter follows a two-phase initialization:

- **Phase 1** — the host sets the public configuration properties (see the User Guide) and connects the
  input/output messages.
- **Phase 2** — ``reset(callTime)`` validates that ``stAttInMsg`` is connected, builds and validates an
  ``InertialFilterConfig``, and constructs the algorithm (which seeds the filter state and covariance).

Two runtime reset entry points are exposed on both the algorithm and the adapter:

- ``reInitializeExceptPersistentStates()`` clears the internal runtime (the pending-measurement queue and the residual
  snapshots) while **preserving** the filter state and covariance.
- ``reInitialize()`` performs ``reInitializeExceptPersistentStates()`` and additionally re-seeds the filter state and
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
      - 0 < alpha < 1
    * - beta
      - prior-knowledge tunable
      - 0 <= beta <= 2
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

    filter = inertialFilterF32.InertialFilter()

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
    # reInitialize() (state + covariance reset to the configured seed) or reInitializeExceptPersistentStates() (keep the
    # current estimate, clear only the pending measurements and residual snapshots).
