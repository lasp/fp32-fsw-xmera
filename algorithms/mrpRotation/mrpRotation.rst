.. raw:: latex

    {\LARGE \textbf{mrpRotation}}

Executive Summary
-----------------
This module makes a reference frame :math:`\mathcal R` from an input reference frame :math:`\mathcal R_0`. It applies
three rotations to the input reference, in this sequence:

1. A fixed body correction :math:`\mathbf\sigma_{\mathcal{R}_c/\mathcal{R}_0}`. It sets which body axes align with the
   input reference axes. For example, it can put the body :math:`-\hat{\mathbf b}_2` axis on the central body of a Hill
   frame.
2. A pointing offset :math:`\mathbf\sigma_{\mathcal{R}_p/\mathcal{R}_c}`. It tilts the corrected frame about body axes,
   for example 10 deg off the target. The operator can change it at runtime.
3. A constant-rate rotation :math:`{}^{\mathcal R}{\mathbf\omega}_{\mathcal{R}/\mathcal{R}_p}` about output (body) axes.
   The module integrates this rotation from identity with the MRP kinematic differential equation.

The output message contains the attitude :math:`\mathbf\sigma_{\mathcal{R}/\mathcal{N}}`, the inertial angular velocity
:math:`{}^{\mathcal N}{\mathbf\omega}_{\mathcal{R}/\mathcal{N}}`, and the inertial angular acceleration
:math:`{}^{\mathcal N}{\dot{\mathbf\omega}}_{\mathcal{R}/\mathcal{N}}` of :math:`\mathcal R`. All math uses single
precision (float32).

Module Architecture
-------------------

The module is split into two layers:

- The **adapter** (``mrpRotation.h``/``.cpp``) is the SysModel-derived class that handles message I/O, validates
  configuration, builds an immutable ``MrpRotationConfig`` from public properties, and constructs the algorithm via
  two-phase initialization. The adapter is also the **payload ↔ Eigen conversion boundary**: it reads the input
  ``AttRefMsgF32Payload``, converts each ``float[3]`` field to ``Eigen::Vector3f`` via
  ``utilities/fsw/eigenSupport.h`` (``cArrayToEigenVector``), passes the resulting algorithm-native input
  struct into the algorithm, and packs the algorithm's output struct back into the output ``AttRefMsgF32Payload``
  via ``eigenVectorToCArray``.
- The **algorithm** (``mrpRotationAlgorithm.h``/``.cpp``) is a pure C++23 class with no framework dependencies and no
  messaging-payload includes. Its ``update()`` consumes one Eigen-typed input bundle -- ``MrpRotationAttRefInputs``
  (mirrors ``AttRefMsgF32Payload``) -- and returns its own POD ``MrpRotationOutput`` (shape of the output
  ``AttRefMsgF32Payload``). Both structs are declared at the top of ``mrpRotationAlgorithm.h``. ``update()`` must not
  throw. The configured ``sigma_RcR0``, ``sigma_RpRc``, ``omega_RRp_R``, and ``controlPeriod`` fully set the output
  reference frame.

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
      - Required input reference frame attitude :math:`\mathbf\sigma_{\mathcal{R}_0/\mathcal{N}}`, rate
        :math:`{}^{\mathcal N}{\mathbf\omega}_{\mathcal{R}_0/\mathcal{N}}`, and acceleration
        :math:`{}^{\mathcal N}{\dot{\mathbf\omega}}_{\mathcal{R}_0/\mathcal{N}}`
    * - attRefOutMsg
      - :ref:`AttRefMsgF32Payload`
      - Output reference frame :math:`\mathbf\sigma_{\mathcal{R}/\mathcal{N}}`, :math:`{}^{\mathcal N}{\mathbf\omega}_{\mathcal{R}/\mathcal{N}}`,
        :math:`{}^{\mathcal N}{\dot{\mathbf\omega}}_{\mathcal{R}/\mathcal{N}}`

.. list-table:: Module Configuration Properties
    :widths: 25 20 10 15 30
    :header-rows: 1

    * - Property
      - Type
      - Units
      - Default
      - Bounds
    * - sigma_RcR0
      - Eigen::Vector3f
      - [-]
      - zero
      - Components must be finite (``allFinite``). Fixed body correction: the corrected frame :math:`\mathcal R_c`
        relative to the input reference :math:`\mathcal R_0`. Row :math:`i` of :math:`[R_cR_0]` is the input reference
        axis :math:`i` in body components. An MRP with norm greater than 1 is stored as its shadow set
    * - sigma_RpRc
      - Eigen::Vector3f
      - [-]
      - zero
      - Components must be finite (``allFinite``). Pointing offset: the frame :math:`\mathcal R_p` relative to
        :math:`\mathcal R_c`, in body axes. An MRP with norm greater than 1 is stored as its shadow set
    * - omega_RRp_R
      - Eigen::Vector3f
      - [rad/s]
      - zero
      - Components must be finite (``allFinite``). Constant rotation rate of :math:`\mathcal R` relative to
        :math:`\mathcal R_p`, in output (body) components
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
    module.sigma_RcR0 = rbk.C2MRP(RcR0)         # fixed body correction; row i of RcR0 = R0 axis i in body axes
    module.sigma_RpRc = [0.0437, 0.0, 0.0]      # 10 deg pointing offset about body x
    module.omega_RRp_R = [0.001745, 0.0, 0.0]   # ~0.1 deg/s roll about body x
    module.controlPeriod = 0.5                  # [s] forward-Euler integration step; required (> 0)

    module.attRefInMsg.subscribeTo(att_ref_msg)

    sim.AddModelToTask(task_name, module)
    sim.InitializeSimulation()  # invokes reset(); validators run, algorithm is constructed

To change the pointing offset at runtime, set the property and call ``reconfigure()``. The new offset applies at the
next update. The integrated rotation continues from its current value::

    module.sigma_RpRc = [0.0, 0.0, 0.0437]
    module.reconfigure()

Algorithm Layer
---------------

Mathematical Formulation
^^^^^^^^^^^^^^^^^^^^^^^^

Assume the input reference frame :math:`\mathcal R_0` is given through an attitude state input message containing
:math:`\mathbf\sigma_{\mathcal{R}_0/\mathcal{N}}`, :math:`{}^{\mathcal N}{\mathbf\omega}_{\mathcal{R}_0/\mathcal{N}}`, and
:math:`{}^{\mathcal N}{\dot{\mathbf\omega}}_{\mathcal{R}_0/\mathcal{N}}`. The MRP set is mapped into the corresponding direction cosine
matrix (DCM)

.. math:: [R_0 N] = [R_0 N(\mathbf\sigma_{\mathcal{R}_0/\mathcal{N}})]

The output reference frame :math:`\mathcal R` comes from three rotations of :math:`\mathcal R_0`:

.. math:: [RN] = [RR_p(\mathbf\sigma_{\mathcal{R}/\mathcal{R}_p}(t))]\,[R_pR_c]\,[R_cR_0]\,[R_0 N]

The body correction :math:`[R_cR_0]` and the pointing offset :math:`[R_pR_c]` are constant. The algorithm calculates
their product :math:`[R_pR_0] = [R_pR_c][R_cR_0]` when it receives a configuration.

The rotation :math:`[RR_p]` starts at identity. Its rate is constant in :math:`\mathcal R` components:

.. math::
   \begin{align}
       \dot{\mathbf\sigma}_{\mathcal{R}/\mathcal{R}_p} &= \frac{1}{4} [B(\mathbf\sigma_{\mathcal{R}/\mathcal{R}_p})]\,
       {}^{\mathcal R}{\mathbf\omega}_{\mathcal{R}/\mathcal{R}_p} \\
       \frac{{}^{\mathcal R}{\textrm{d}}{\mathbf\omega}_{\mathcal{R}/\mathcal{R}_p}}{\textrm{d}t} &= \mathbf 0
   \end{align}

Each update propagates the MRP set with one forward Euler step. The step :math:`\Delta t` is the configured
``controlPeriod``:

.. math::
   \mathbf\sigma_{\mathcal{R}/\mathcal{R}_p}(t_{k+1}) = \texttt{mrpSwitch}\!\left(\mathbf\sigma_{\mathcal{R}/\mathcal{R}_p}(t_k)
   + \Delta t \cdot \dot{\mathbf\sigma}_{\mathcal{R}/\mathcal{R}_p},\ 1\right)

``mrpSwitch`` changes the MRP to its shadow set when its norm is greater than one.

The rotation is the last of the three, so the rate acts about output (body) axes. A rate along body :math:`\hat{\mathbf
b}_1` rolls the body about :math:`\hat{\mathbf b}_1`, also when a pointing offset is present. The rotation starts at
identity and its axis is constant, so the integrated MRP stays on that axis. The integration error changes only the
rotation angle, not the axis.

The output MRP set comes from the DCM:

.. math:: \mathbf\sigma_{\mathcal{R}/\mathcal{N}} = \texttt{dcmToMrp}([RN])

The frame :math:`\mathcal R_p` is fixed in :math:`\mathcal R_0`, thus
:math:`\mathbf\omega_{\mathcal{R}/\mathcal{R}_0} = \mathbf\omega_{\mathcal{R}/\mathcal{R}_p}`. The algorithm changes the
rate to inertial components and adds the input rate:

.. math::
   \begin{align}
       {}^{\mathcal N}{\mathbf\omega}_{\mathcal{R}/\mathcal{R}_p} &= [RN]^{T}\,{}^{\mathcal R}{\mathbf\omega}_{\mathcal{R}/\mathcal{R}_p} \\
       {}^{\mathcal N}{\mathbf\omega}_{\mathcal{R}/\mathcal{N}} &= {}^{\mathcal N}{\mathbf\omega}_{\mathcal{R}/\mathcal{R}_p} +
       {}^{\mathcal N}{\mathbf\omega}_{\mathcal{R}_0/\mathcal{N}}
   \end{align}

The transport theorem gives the inertial angular acceleration. It uses
:math:`\frac{{}^{\mathcal R}{\textrm{d}}{\mathbf\omega}_{\mathcal{R}/\mathcal{R}_p}}{\textrm{d}t} = \mathbf 0` and
:math:`\mathbf\omega_{\mathcal{R}/\mathcal{N}} \times {\mathbf\omega}_{\mathcal{R}/\mathcal{R}_p} =
\mathbf\omega_{\mathcal{R}_0/\mathcal{N}} \times {\mathbf\omega}_{\mathcal{R}/\mathcal{R}_p}`:

.. math::
   {}^{\mathcal N}{\dot{\mathbf\omega}}_{\mathcal{R}/\mathcal{N}} = {}^{\mathcal N}{\mathbf\omega}_{\mathcal{R}_0/\mathcal{N}} \times
   {}^{\mathcal N}{\mathbf\omega}_{\mathcal{R}/\mathcal{R}_p} + {}^{\mathcal N}{\dot{\mathbf\omega}}_{\mathcal{R}_0/\mathcal{N}}

Cone Around a Target
^^^^^^^^^^^^^^^^^^^^

A boresight can turn on a cone around the target. Set the body correction so that the boresight
:math:`\hat{\mathbf b}` points at the target. Set the pointing offset to the cone half-angle :math:`\alpha`. Set the
rate along the target line in body components:

.. math:: {}^{\mathcal R}{\mathbf\omega}_{\mathcal{R}/\mathcal{R}_p} = \omega\,[R_pR_c]\,\hat{\mathbf b}

The boresight then stays at :math:`\alpha` from the target and turns around it once every :math:`2\pi/\omega`. The
spacecraft turns as a rigid body, thus it also turns 360 deg about the target line in each cycle.

Module Assumptions and Limitations
----------------------------------

- Every ``update()`` advances the MRP by the configured ``controlPeriod``; there is no first-update "warm-up" tick.
  The caller must set ``controlPeriod`` to match the rate at which the adapter is being driven (e.g. the simulation
  task rate), otherwise the integrated reference frame will drift relative to true wall-clock time.
- A change of the pointing offset through ``reconfigure()`` applies as a step at the next update. The output rate and
  acceleration do not include a rate for this step.
- The module uses forward Euler integration. The integration error of the rotation angle increases with
  :math:`\Delta t` and :math:`\|\mathbf\omega_{\mathcal{R}/\mathcal{R}_p}\|`. Above approximately 0.2 rad per step,
  float32 rounding errors off the rotation axis increase quickly. Keep
  :math:`\|\mathbf\omega_{\mathcal{R}/\mathcal{R}_p}\|\,\Delta t` well below 0.2 rad.
- All math uses single-precision (float32). Compared to the double-precision Xmera implementation, regression
  tolerances are relaxed to roughly :math:`10^{-5}`.

Test Description
----------------
Regression tests run the algorithm for several steps and compare the output with an independent reference
implementation. Setup tests examine the ``MrpRotationConfig`` validators and the shadow-set storage of the offsets.
Property tests examine:

- The output is finite.
- With zero rate, the output is the input reference with the two offsets applied.
- The rotation axis stays fixed in the input reference for 2000 steps.
- The output rate obeys the transport theorem.

Edge-case tests examine ``reInitialize()``, a new pointing offset through ``setConfig()``, the shadow-set switch, and a
10 deg cone around a target. Fuzz tests use random configurations and inputs in realistic ranges.
