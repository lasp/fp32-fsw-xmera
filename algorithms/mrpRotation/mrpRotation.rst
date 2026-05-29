.. raw:: latex

    {\LARGE \textbf{mrpRotation}}

Executive Summary
-----------------
This module produces a dynamic reference frame attitude state by superimposing a constant-rate rotation on top of an
input reference frame. The initial orientation relative to the input reference :math:`\mathcal R_0` is specified
through an MRP set :math:`\mathbf\sigma_{R/R_0}`, and the :math:`\mathcal R`-frame angular velocity vector
:math:`{}^{\mathcal R}{\mathbf\omega}_{R/R_0}` is held constant in :math:`\mathcal R`-frame components. The module
integrates :math:`\mathbf\sigma_{R/R_0}` forward each cycle using the MRP kinematic differential equation, then
composes the result with the input reference to produce the output attitude :math:`\mathbf\sigma_{R/N}`, the inertial
angular velocity :math:`{}^{\mathcal N}{\mathbf\omega}_{R/N}`, and the inertial angular acceleration
:math:`{}^{\mathcal N}{\dot{\mathbf\omega}}_{R/N}`. This is a single-precision (float32) port of the original
double-precision Xmera implementation.

Module Architecture
-------------------

The module is split into two layers:

- The **adapter** (``mrpRotation.h``/``.cpp``) is the SysModel-derived class that handles message I/O, validates
  configuration, builds an immutable ``MrpRotationConfig`` from public properties, and constructs the algorithm via
  two-phase initialization. The adapter is also the **payload ↔ Eigen conversion boundary**: it reads the input
  ``AttRefMsgF32Payload``, converts each ``float[3]`` field to ``Eigen::Vector3f`` via
  ``architecture/utilities/eigenSupport.h`` (``cArrayToEigenVector``), passes the resulting algorithm-native input
  struct into the algorithm, and packs the algorithm's output struct back into the output ``AttRefMsgF32Payload``
  via ``eigenVectorToCArray``.
- The **algorithm** (``mrpRotationAlgorithm.h``/``.cpp``) is a pure C++23 class with no framework dependencies and no
  messaging-payload includes. Its ``update()`` consumes one Eigen-typed input bundle -- ``MrpRotationAttRefInputs``
  (mirrors ``AttRefMsgF32Payload``) -- and returns its own POD ``MrpRotationOutput`` (shape of the output
  ``AttRefMsgF32Payload``). Both structs are declared at the top of ``mrpRotationAlgorithm.h``. ``update()`` must not
  throw. The rotating reference frame is fully determined by the configured ``sigma_RR0``, ``omega_RR0_R``, and
  ``controlPeriod``; the algorithm has no runtime command-latching path.

A pure-C shim (``mrpRotationAlgorithm_c.h``/``.cpp``) wraps the algorithm class for use by Ada/Adamant components via
``extern "C"`` bindings. The shim's ``_update`` takes a ``MrpRotationAttRefInputs_c`` POD mirror (declared in
``mrpRotationTypes.h`` and built from ``Vector3f_c``) and returns ``MrpRotationOutput_c``; Ada callers are responsible
for the analogous payload-to-POD conversion on their side.

Adapter Layer
^^^^^^^^^^^^^

The adapter consumes the following messages and exposes the configuration as public properties:

.. list-table:: Module I/O Messages
    :widths: 20 30 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - attRefInMsg
      - :ref:`AttRefMsgF32Payload`
      - Required input reference frame attitude :math:`\mathbf\sigma_{R_0/N}`, rate
        :math:`{}^{\mathcal N}{\mathbf\omega}_{R_0/N}`, and acceleration
        :math:`{}^{\mathcal N}{\dot{\mathbf\omega}}_{R_0/N}`
    * - attRefOutMsg
      - :ref:`AttRefMsgF32Payload`
      - Output reference frame :math:`\mathbf\sigma_{R/N}`, :math:`{}^{\mathcal N}{\mathbf\omega}_{R/N}`,
        :math:`{}^{\mathcal N}{\dot{\mathbf\omega}}_{R/N}`

.. list-table:: Module Configuration Properties
    :widths: 25 20 10 15 30
    :header-rows: 1

    * - Property
      - Type
      - Units
      - Default
      - Bounds
    * - sigma_RR0
      - Eigen::Vector3f
      - [-]
      - zero
      - Components must be finite (``allFinite``); MRPs greater than 1 in norm are mapped to the shadow set during
        integration via ``mrpSwitch``
    * - omega_RR0_R
      - Eigen::Vector3f
      - [rad/s]
      - zero
      - Components must be finite (``allFinite``)
    * - controlPeriod
      - float
      - [s]
      - 0.0 (invalid)
      - Must be ``> 0`` (the validator also rejects NaN). Used directly as the forward-Euler
        integration step on every ``update()``; the caller must set this before ``reset()`` runs --
        typically to the simulation task's update rate.

Two-Phase Initialization
~~~~~~~~~~~~~~~~~~~~~~~~

The adapter follows the standard two-phase init pattern: the user sets public properties, then ``reset()`` validates
inputs and constructs the algorithm. ``updateState()`` throws ``XmeraLifecycleException`` if invoked before
``reset()``::

    module = mrpRotationF32.MrpRotation()
    module.modelTag = "mrpRotation"
    module.sigma_RR0 = [0.3, 0.5, 0.0]
    module.omega_RR0_R = [0.001745, 0.0, 0.0]   # ~0.1 deg/s about x in R-frame
    module.controlPeriod = 0.5                  # [s] forward-Euler integration step; required (> 0)

    module.attRefInMsg.subscribeTo(att_ref_msg)

    sim.AddModelToTask(task_name, module)
    sim.InitializeSimulation()  # invokes reset(); validators run, algorithm is constructed

Algorithm Layer
---------------

Mathematical Formulation
^^^^^^^^^^^^^^^^^^^^^^^^

Assume the input reference frame :math:`\mathcal R_0` is given through an attitude state input message containing
:math:`\mathbf\sigma_{R_0/N}`, :math:`{}^{\mathcal N}{\mathbf\omega}_{R_0/N}`, and
:math:`{}^{\mathcal N}{\dot{\mathbf\omega}}_{R_0/N}`. The MRP set is mapped into the corresponding direction cosine
matrix (DCM)

.. math:: [R_0 N] = [R_0 N(\mathbf\sigma_{R_0/N})]

The output reference frame :math:`\mathcal R` is constructed so that

.. math::
   \begin{align}
       \dot{\mathbf\sigma}_{R/R_0} &= \frac{1}{4} [B(\mathbf\sigma_{R/R_0})]\,{}^{\mathcal R}{\mathbf\omega}_{R/R_0}
       & \label{eq:mRot1} \\
       \frac{{}^{\mathcal R}{\textrm{d}}{\mathbf\omega}_{R/R_0}}{\textrm{d}t} &= \mathbf 0 & \label{eq:mRot2}
   \end{align}

The MRP set :math:`\mathbf\sigma_{R/R_0}` is propagated each cycle using forward Euler integration with the integration
step :math:`\Delta t` taken directly from the configured ``controlPeriod``:

.. math::
   \mathbf\sigma_{R/R_0}(t_{k+1}) = \texttt{mrpSwitch}\!\left(\mathbf\sigma_{R/R_0}(t_k) + \Delta t \cdot
   \dot{\mathbf\sigma}_{R/R_0},\ 1\right)

``mrpSwitch`` maps the MRP to the shadow set when the result has norm greater than one, ensuring the representation
stays bounded. Every ``update()`` advances the MRP by ``controlPeriod``; the caller is responsible for setting
``controlPeriod`` to match the rate at which ``update()`` is invoked (typically the simulation task rate).

The current DCM of the :math:`\mathcal R`-frame is

.. math:: [RN] = [RR_0(\mathbf\sigma_{R/R_0}(t))]\,[R_0 N]

The output MRP set is read from this DCM:

.. math:: \mathbf\sigma_{R/N} = \texttt{dcmToMrp}([RN])

The angular velocity is mapped to inertial-frame components and combined with the input rate:

.. math::
   \begin{align}
       {}^{\mathcal N}{\mathbf\omega}_{R/R_0} &= [RN]^{T}\,{}^{\mathcal R}{\mathbf\omega}_{R/R_0} \\
       {}^{\mathcal N}{\mathbf\omega}_{R/N} &= {}^{\mathcal N}{\mathbf\omega}_{R/R_0} +
       {}^{\mathcal N}{\mathbf\omega}_{R_0/N}
   \end{align}

The inertial angular acceleration of the output frame is found via the transport theorem, noting that
:math:`{}^{\mathcal R}{\dot{\mathbf\omega}}_{R/R_0} = \mathbf 0` and using
:math:`\mathbf\omega_{R/N} \times {\mathbf\omega}_{R/R_0} = \mathbf\omega_{R_0/N} \times {\mathbf\omega}_{R/R_0}`:

.. math::
   {}^{\mathcal N}{\dot{\mathbf\omega}}_{R/N} = {}^{\mathcal N}{\mathbf\omega}_{R_0/N} \times
   {}^{\mathcal N}{\mathbf\omega}_{R/R_0} + {}^{\mathcal N}{\dot{\mathbf\omega}}_{R_0/N}

Module Assumptions and Limitations
----------------------------------

- Every ``update()`` advances the MRP by the configured ``controlPeriod``; there is no first-update "warm-up" tick.
  The caller must set ``controlPeriod`` to match the rate at which the adapter is being driven (e.g. the simulation
  task rate), otherwise the integrated reference frame will drift relative to true wall-clock time.
- The configured ``sigma_RR0`` and ``omega_RR0_R`` are the sole source of the rotating-reference seed values; the
  module does not consume a runtime command stream. To re-target the rotation at runtime, update the public
  properties and call ``reset()`` again.
- Forward Euler integration is used; for large :math:`\Delta t` and large :math:`\mathbf\omega_{R/R_0}`, integration
  drift will accumulate. The expected use case is small steps (<= 1 s) and modest rotation rates.
- All math uses single-precision (float32). Compared to the double-precision Xmera implementation, regression
  tolerances are relaxed to roughly :math:`10^{-5}`.

Test Description
----------------
The module is verified through regression tests that drive the algorithm through several integration steps and compare
against an independently coded reference implementation, setup tests for the ``MrpRotationConfig`` validators, property
tests for finiteness of the integrated output, and edge-case tests covering zero angular velocity and the
reset-reseeds-runtime-state contract. Fuzz tests randomize the configuration and inputs over reasonable ranges.
