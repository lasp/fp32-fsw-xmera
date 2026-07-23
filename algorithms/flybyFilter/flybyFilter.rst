Executive Summary
-----------------
This module estimates the inertial spacecraft position and velocity relative to a central body from
optical-navigation heading (unit-vector) measurements, using a square-root unscented Kalman filter
(SRuKF) under two-body point-mass gravity. This is angles-only orbit determination: a single heading
observes the pointing direction, and range/velocity are resolved through the dynamics over an arc. All
computation is double precision.

The module is split into an xmera ``SysModel`` adapter (``FlybyFilter``) and a framework-agnostic
algorithm (``FlybyFilterAlgorithm``). The algorithm is a thin, problem-specific layer over the reusable
:ref:`filtering-architecture` framework: it wires the position/velocity state, two-body dynamics, and the
heading measurement model into a ``filtering::SRuKF`` and drains its measurements through the framework's
robust scheduler (see `Relation to filteringCore`_). Configuration is validated through an immutable
``FlybyFilterConfig`` and applied with a two-phase initialization lifecycle: the host sets the adapter's
public properties, then ``reset()`` builds the validated config and seeds the filter. The filter operates
internally in km and km/s for numerical conditioning; the adapter converts to/from SI at the message
boundary.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - opNavHeadingMsg
      - :ref:`OpNavUnitVecMsgPayload`
      - Input optical-navigation heading: unit vector ``rhat_BN_N`` from the spacecraft to the central
        body, inertial frame; required
    * - navTransOutMsg
      - :ref:`NavTransMsgPayload`
      - Output message containing the estimated inertial position and velocity
    * - filterOutMsg
      - :ref:`FilterMsgPayload`
      - Output message with the filter estimated state and covariance
    * - filterResOutMsg
      - :ref:`FilterResidualsMsgPayload`
      - Output message containing pre- and post-fit residuals for the heading measurements


Detailed Module Description
---------------------------
The estimated state is a 6-element vector combining the inertial position and velocity of the spacecraft
relative to the central body. It is composed from filteringCore state tags as
``filtering::StateVector<filtering::Position<3>, filtering::Velocity<3>>``:

.. math::
    \boldsymbol{x} = \left\{ \begin{matrix} \boldsymbol{r}_{B/N} \\
    \boldsymbol{v}_{B/N} \end{matrix} \right\}.

Dynamics model
++++++++++++++
The state evolves under two-body point-mass gravity, with :math:`\mu` the central-body gravitational
parameter:

.. math::
    \boldsymbol{\dot{x}} = \left\{ \begin{matrix} \boldsymbol{v}_{B/N} \\
    -\dfrac{\mu}{\lVert \boldsymbol{r}_{B/N} \rVert^3}\, \boldsymbol{r}_{B/N} \end{matrix} \right\}.

The dynamics functor carries :math:`\mu` (in internal units) and is set from the configuration; the SRuKF
propagates the sigma points with the shared RK4 integrator.

Measurement model
+++++++++++++++++
A single measurement type is processed: an optical-navigation heading unit vector. It is expressed as a
model type satisfying the filteringCore ``Measurement`` concept (``observation()``, ``model()``,
``noise()``, ``subtract()``), which the ``SRuKF`` consumes generically. The measured quantity is the unit
vector ``rhat_BN_N``, and the measurement model is the normalized position:

.. math::
    \boldsymbol{h}(\boldsymbol{x}) = \frac{\boldsymbol{r}_{B/N}}{\lVert \boldsymbol{r}_{B/N} \rVert}.

The innovation is the plain vector difference of the measured and predicted unit vectors. The measurement
noise covariance is diagonal, built from the configured ``headingMeasurementNoiseStd``. Because a heading
only observes two of the three position degrees of freedom (the transverse directions), range is only
weakly observable instantaneously and is resolved through the two-body dynamics over a measurement arc.


Module Architecture
-------------------
``FlybyFilterAlgorithm`` is framework-agnostic: it holds a ``filtering::SRuKF`` and a validated
``FlybyFilterConfig``, owns the ``measurement_queue``, and exposes ``update()``, ``reInitializeExceptPersistentStates()``, and
``reInitialize()``. On each ``update()`` it enqueues a fresh heading reading if present and drains the
queue through the filteringCore ``applySequentialRobust`` scheduler. ``FlybyFilter`` is the xmera adapter:
it owns the message ports, converts payloads to/from the algorithm's Eigen types (SI on the wire,
km/(km/s) internally), and drives the lifecycle.

Configuration is immutable once built. ``FlybyFilterConfig::create(...)`` validates every constrained
parameter and throws on invalid input; the algorithm trusts the config thereafter. The constant filter
parameters (including the ``mu``-carrying dynamics functor) are pushed into the SRuKF by ``setConfig()``
(which calls the SRuKF's ``configure()`` to re-derive the sigma-point spread, weights, and process-noise
Cholesky), so a configuration change takes effect immediately while preserving the current estimate.

Relation to filteringCore
+++++++++++++++++++++++++
This module is a concrete instantiation of the :ref:`filtering-architecture` building blocks; it supplies
the problem-specific types and lets the core provide the estimator machinery. The mapping is:

.. list-table:: filteringCore primitives used by this module
    :widths: 35 65
    :header-rows: 1

    * - filteringCore primitive
      - Flyby-filter specialization
    * - ``StateVector<Components...>``
      - ``StateVector<Position<3>, Velocity<3>>`` -- the 6-element position/velocity state
    * - ``Dynamics`` concept
      - ``FlybyDynamics`` -- the two-body point-mass gravity functor (carries ``mu``)
    * - ``Measurement`` concept
      - ``HeadingMeasurementModel`` (``observation``/``model``/``noise``/``subtract``); model = ``r/|r|``
    * - ``SRuKF<State, Dyn>``
      - ``SRuKF<FlybyState, FlybyDynamics>`` -- the square-root UKF that owns the estimate
    * - ``measurement_queue<Measurement, N>``
      - time-ordered buffer of the per-cycle heading measurements
    * - ``applySequentialRobust``
      - drains the queue, one ``timeUpdate`` + ``measurementUpdate`` per measurement, holding the anchor

Because the module selects the *robust* scheduler, ``FlybyFilterAlgorithm`` fulfils that scheduler's
contract: its ``timeUpdate`` and ``measurementUpdate`` return a ``bool`` validity, and it implements
``clear()`` to roll back to the last-good state after a bad update. The config validators reuse the
``SRuKF`` static checks (``alphaIsValid``/``betaIsValid``) so the valid ranges stay defined in one place.

Lifecycle
+++++++++
The adapter follows a two-phase initialization:

- **Phase 1** -- the host sets the public configuration properties (see the User Guide) and connects the
  input/output messages.
- **Phase 2** -- ``reset(callTime)`` validates that ``opNavHeadingMsg`` is connected, converts the SI
  properties to internal units, builds and validates a ``FlybyFilterConfig``, and constructs the algorithm
  (which seeds the filter state and covariance).

Two runtime reset entry points are exposed on both the algorithm and the adapter:

- ``reInitializeExceptPersistentStates()`` clears the internal runtime (the pending-measurement queue and the residual snapshot)
  while **preserving** the filter state and covariance.
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
    * - mu
      - central-body gravitational parameter [m^3/s^2]
      - > 0
    * - unitConversion
      - SI -> internal length scale (1e-3 = m -> km)
      - > 0
    * - processNoise
      - N x N process noise covariance Q
      - positive semi-definite
    * - initialState
      - N-element initial state seed [m, m/s]
      - any
    * - initialCovariance
      - N x N initial covariance P0
      - positive semi-definite
    * - headingMeasurementNoiseStd
      - heading (unit-vector) measurement noise standard deviation
      - >= 0


User Guide
----------
Set the public properties (in SI units), connect the messages, and let ``reset()`` build and validate the
configuration::

    filter = flybyFilterF32.FlybyFilter()

    filter.alpha = 0.02
    filter.beta = 2.0
    filter.unitConversion = 1e-3           # filter internally in km, km/s
    filter.mu = 42828.314 * 1e9            # [m^3/s^2] central-body gravitational parameter
    filter.headingMeasurementNoiseStd = 1e-3
    filter.initialState = [r_x, r_y, r_z, v_x, v_y, v_z]   # [m, m/s]
    filter.initialCovariance = np.diag([1000.*1e6]*3 + [0.1*1e6]*3).tolist()
    filter.processNoise = np.diag([(1e-6)**2]*3 + [(1e-8)**2]*3).tolist()

    # Connect the input/output messages (opNavHeadingMsg required), then the simulation calls reset()
    # once before stepping. To restart the filter at runtime, call reInitialize() (state + covariance
    # reset to the configured seed) or reInitializeExceptPersistentStates() (keep the current estimate, clear only the pending
    # measurements and residual snapshot).
