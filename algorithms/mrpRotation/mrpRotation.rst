.. raw:: latex

    {\LARGE \textbf{mrpRotation}}

Executive Summary
-----------------
This module points a selected component, such as a camera or an antenna, rather than a nominal body axis. It builds
the output reference frame :math:`\mathcal R` from the input reference frame :math:`\mathcal R_0` in pointing order:

1. A base frame :math:`\mathbf\sigma_{\mathcal{R}_b/\mathcal{R}_0}`. It puts the nominal body axis of the state on
   the target, for example body :math:`-\hat{\mathbf b}_2` on the central body of a Hill frame.
2. The component. The body turns so the measured component frame sits where the nominal component frame sits in the
   base attitude, so every measured component axis points where its nominal axis would. This step uses the nominal
   mounting :math:`\mathbf\sigma_{\mathcal{C}_n/\mathcal{B}}` and the measured mounting
   :math:`\mathbf\sigma_{\mathcal{C}/\mathcal{B}}`, and it is where the misalignment is corrected.
3. A pointing offset about component axes. It turns the component frame by the offset angle. The operator can change
   it at runtime.
4. A constant-rate spin :math:`\Omega\,\hat{\mathbf e}` about a spin axis :math:`\hat{\mathbf e}` that is constant in
   component axes. The module integrates this spin from identity with the MRP kinematic differential equation.

Each step turns the body. The offset and the spin axis are given in component axes, and the offset comes after the
component, so the measured component frame is off its nominal place in the base attitude by exactly the commanded
offset, whatever the misalignment. Because the spin comes after the offset, the spin axis holds its direction while
the spacecraft turns about it.

Every component, a camera or an antenna alike, has a full frame. Its nominal frame
:math:`\mathbf\sigma_{\mathcal{C}_n/\mathcal{B}}` comes from the ICD and its measured frame
:math:`\mathbf\sigma_{\mathcal{C}/\mathcal{B}}` from the alignment survey. Offsets and spin are in its own axes, for
example image axes for a camera.

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
  throw. The configured ``sigma_RbR0``, ``sigma_CnB``, ``sigma_CB``, ``sigma_RpRc_C``, ``spinAxis_C``, ``spinRate``, and
  ``controlPeriod`` fully set the output reference frame.

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
    * - sigma_RbR0
      - Eigen::Vector3f
      - [-]
      - zero
      - Components must be finite (``allFinite``). Base frame: the frame :math:`\mathcal R_b` relative to the input
        reference :math:`\mathcal R_0`, with the nominal body axis on the target. Row :math:`i` of :math:`[R_bR_0]` is
        body axis :math:`i` in input reference components. An MRP with norm greater than 1 is stored as its shadow set
    * - sigma_CnB
      - Eigen::Vector3f
      - [-]
      - zero
      - Components must be finite. Nominal mounting: the nominal component frame :math:`\mathcal C_n` relative to the
        body frame. Row :math:`i` of :math:`[C_nB]` is nominal component axis :math:`i` in body components. Stored as
        its shadow set when the norm is greater than 1
    * - sigma_CB
      - Eigen::Vector3f
      - [-]
      - zero
      - Components must be finite. Measured mounting: the component frame :math:`\mathcal C` relative to the body
        frame, as measured. Row :math:`i` of :math:`[CB]` is component axis :math:`i` in body components. Stored as its
        shadow set when the norm is greater than 1
    * - sigma_RpRc_C
      - Eigen::Vector3f
      - [-]
      - zero
      - Components must be finite (``allFinite``). Pointing offset: the frame :math:`\mathcal R_p` relative to
        :math:`\mathcal R_c`, as an MRP in component axes. An MRP with norm greater than 1 is stored as its shadow set
    * - spinAxis_C
      - Eigen::Vector3f
      - [-]
      - zero (invalid)
      - Must be finite with norm within 1e-3 of 1. Spin axis :math:`\hat{\mathbf e}` of :math:`\mathcal R` relative
        to :math:`\mathcal R_p`, in component axes. The configuration stores the normalized axis. Set it also when
        ``spinRate`` is zero
    * - spinRate
      - float
      - [rad/s]
      - 0.0
      - Must be finite. Spin rate :math:`\Omega` of :math:`\mathcal R` relative to :math:`\mathcal R_p` about
        ``spinAxis_C``. A negative value turns in the opposite direction
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
    module.sigma_RbR0 = rbk.C2MRP(RbR0)         # base frame; row i of RbR0 = body axis i in R0 components
    module.sigma_CnB = rbk.C2MRP(CnB)           # nominal mounting; row i of CnB = camera axis i in body components
    module.sigma_CB = rbk.C2MRP(CB)             # measured mounting, from the alignment survey
    module.sigma_RpRc_C = [0.0437, 0.0, 0.0]    # 10 deg pointing offset about the camera x axis
    module.spinAxis_C = [0.0, 0.0, 1.0]         # spin about the camera z axis
    module.spinRate = 0.001745                  # [rad/s] ~0.1 deg/s
    module.controlPeriod = 0.5                  # [s] forward-Euler integration step; required (> 0)

    module.attRefInMsg.subscribeTo(att_ref_msg)

    sim.AddModelToTask(task_name, module)
    sim.InitializeSimulation()  # invokes reset(); validators run, algorithm is constructed

To change the pointing offset at runtime, set the property and call ``reconfigure()``. The new offset applies at the
next update. The integrated spin continues from its current value::

    module.sigma_RpRc_C = [0.0, 0.0437, 0.0]
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

The output reference frame :math:`\mathcal R` comes from four rotations of :math:`\mathcal R_0`, in pointing order:

.. math:: [RN] = [RR_p(\mathbf\sigma_{\mathcal{R}/\mathcal{R}_p}(t))]\,[R_pR_c]\,[R_cR_b]\,[R_bR_0]\,[R_0 N]

Each rotation turns the body:

- The base frame :math:`[R_bR_0]` puts the nominal body axis on the target.
- The component rotation :math:`[R_cR_b]` turns the body so the measured component frame :math:`\mathcal C` sits where
  the nominal component frame :math:`\mathcal C_n` sits in the base attitude, :math:`[CB][R_cN] = [C_nB][R_bN]`:

  .. math:: [R_cR_b] = [BC]\,[C_nB]

  with :math:`[BC] = [CB]^T`. It is the small rotation between the nominal and the measured component frames, and the
  identity for a perfect mounting. Every component axis then points where its nominal counterpart points in the
  base attitude.
- The pointing offset :math:`[R_pR_c]` turns the body about component axes, so the component frame turns by the
  offset angle. It is given as the MRP :math:`{}^{\mathcal C}\mathbf\sigma_{\mathcal{R}_p/\mathcal{R}_c}` in
  component axes. An MRP lies along the axis of its rotation, so in body axes it is

  .. math:: {}^{\mathcal B}\mathbf\sigma_{\mathcal{R}_p/\mathcal{R}_c} = [BC]\,{}^{\mathcal C}\mathbf\sigma_{\mathcal{R}_p/\mathcal{R}_c}

- The spin :math:`[RR_p]` turns the body about the spin axis :math:`{}^{\mathcal C}\hat{\mathbf e}`, given in component
  axes. In body axes it is :math:`{}^{\mathcal R_p}\hat{\mathbf e} = [BC]\,{}^{\mathcal C}\hat{\mathbf e}`.

The base frame, the mountings and the pointing offset are constant. The algorithm calculates their product
:math:`[R_pR_0] = [R_pR_c][R_cR_b][R_bR_0]` and the spin axis in body axes when it receives a configuration.

The rotation :math:`[RR_p]` starts at identity. Its rate is constant in :math:`\mathcal R_p` components. The MRP
kinematic differential equation uses the rate in :math:`\mathcal R` components:

.. math::
   \begin{align}
       {}^{\mathcal R_p}{\mathbf\omega}_{\mathcal{R}/\mathcal{R}_p} &= \Omega\,{}^{\mathcal R_p}\hat{\mathbf e} \\
       {}^{\mathcal R}{\mathbf\omega}_{\mathcal{R}/\mathcal{R}_p} &= [RR_p(\mathbf\sigma_{\mathcal{R}/\mathcal{R}_p})]\,{}^{\mathcal R_p}{\mathbf\omega}_{\mathcal{R}/\mathcal{R}_p} \\
       \dot{\mathbf\sigma}_{\mathcal{R}/\mathcal{R}_p} &= \frac{1}{4} [B(\mathbf\sigma_{\mathcal{R}/\mathcal{R}_p})]\,
       {}^{\mathcal R}{\mathbf\omega}_{\mathcal{R}/\mathcal{R}_p} \\
       \frac{{}^{\mathcal R_p}{\textrm{d}}{\mathbf\omega}_{\mathcal{R}/\mathcal{R}_p}}{\textrm{d}t} &= \mathbf 0
   \end{align}

Each update propagates the MRP set with one forward Euler step. The step :math:`\Delta t` is the configured
``controlPeriod``:

.. math::
   \mathbf\sigma_{\mathcal{R}/\mathcal{R}_p}(t_{k+1}) = \texttt{mrpSwitch}\!\left(\mathbf\sigma_{\mathcal{R}/\mathcal{R}_p}(t_k)
   + \Delta t \cdot \dot{\mathbf\sigma}_{\mathcal{R}/\mathcal{R}_p},\ 1\right)

``mrpSwitch`` changes the MRP to its shadow set when its norm is greater than one.

The rotation starts at identity and its axis is constant in :math:`\mathcal R_p`, so the integrated MRP stays on that
axis. The integration error changes only the spin angle, not the axis. The axis of a rotation has the same components
in :math:`\mathcal R` and :math:`\mathcal R_p`. Thus the component axis along :math:`\hat{\mathbf e}` stays fixed in
:math:`\mathcal R_0`, and the spacecraft turns about it.

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
:math:`\frac{{}^{\mathcal R}{\textrm{d}}{\mathbf\omega}_{\mathcal{R}/\mathcal{R}_p}}{\textrm{d}t} = \mathbf 0`, which is
true because the rate is constant in :math:`\mathcal R_p` and :math:`\mathbf\omega_{\mathcal{R}/\mathcal{R}_p} \times
\mathbf\omega_{\mathcal{R}/\mathcal{R}_p} = \mathbf 0`, and
:math:`\mathbf\omega_{\mathcal{R}/\mathcal{N}} \times {\mathbf\omega}_{\mathcal{R}/\mathcal{R}_p} =
\mathbf\omega_{\mathcal{R}_0/\mathcal{N}} \times {\mathbf\omega}_{\mathcal{R}/\mathcal{R}_p}`:

.. math::
   {}^{\mathcal N}{\dot{\mathbf\omega}}_{\mathcal{R}/\mathcal{N}} = {}^{\mathcal N}{\mathbf\omega}_{\mathcal{R}_0/\mathcal{N}} \times
   {}^{\mathcal N}{\mathbf\omega}_{\mathcal{R}/\mathcal{R}_p} + {}^{\mathcal N}{\dot{\mathbf\omega}}_{\mathcal{R}_0/\mathcal{N}}

Cone Around a Target
^^^^^^^^^^^^^^^^^^^^

A component axis can also turn on a cone around the target. Let :math:`\hat{\mathbf a}` be the component axis that
the base puts on the target, in component axes: :math:`[C_nB]` times the body axis of the base frame entry. Set the
pointing offset to the cone half-angle :math:`\alpha` about an axis normal to :math:`\hat{\mathbf a}`, and set the
spin axis along the target line. The target is on :math:`\hat{\mathbf a}` before the offset. So in component axes
after the offset:

.. math:: {}^{\mathcal C}\hat{\mathbf e} = [CB]\,[R_pR_c]\,[BC]\,\hat{\mathbf a}

The axis :math:`\hat{\mathbf a}` then stays at :math:`\alpha` from the target and turns around it once every
:math:`2\pi/|\Omega|`. The
spacecraft turns as a rigid body, thus it also turns 360 deg about the target line in each cycle.

Component Pointing Examples
^^^^^^^^^^^^^^^^^^^^^^^^^^^

``_tests/test_mrpRotation_componentPointing.py`` chains the guidance module of each point state into this module,
with no spacecraft dynamics, and checks where the selected component points. Run it directly to save its figures::

    python test_mrpRotation_componentPointing.py <output directory>

Each figure shows the sky around the target, seen from the spacecraft: the target, the component axis that the base
puts on the target, the nominal body axis that the base frame puts on the target, and arrows toward
:math:`+\hat{\mathbf b}_3` that show the roll. The angles are exaggerated so they are visible: a 2 deg misalignment and a 6 deg offset about the component
x axis.

Earth Point uses the HGA, whose frame is aligned with the body frame when it is mounted perfectly, so the base puts
:math:`+\hat{\mathbf c}_2` on Earth and the example spins about it: ``spinAxis_C = [0, 1, 0]``. Hill Point and
Inertial Point use the MAC, with :math:`+\hat{\mathbf c}_3` on the target and the spin about it. Each state runs four
scenarios:

- **Component only.** The measured component frame sits where the nominal one sits in the base attitude, so the
  component axis is on the target and the nominal body axis is off it by the misalignment.
- **Offset only.** The component axis is off the target by the offset angle.
- **Component and offset.** The component puts its axis on the target, and the offset then turns it off by exactly
  the offset angle, about component axes.
- **Then spin.** The spin axis holds its direction and the spacecraft turns about it. The nominal body axis circles
  it.

The spin and offset figure keeps the same offset in Hill Point and changes only the spin axis. A second row shows
where the target appears in the MAC image:

- **Spin about** :math:`+\hat{\mathbf c}_3` (this design). :math:`+\hat{\mathbf c}_3` holds the offset point, and
  the target circles the image center.
- **Spin about the target line.** :math:`+\hat{\mathbf c}_3` cones around the target, and the target stays put in the
  image. An offset applied after the spin gives the same motion.
- **Spin about** :math:`+\hat{\mathbf c}_1`, which is the offset axis. :math:`+\hat{\mathbf c}_3` sweeps back along
  the offset line, across the target.

Module Assumptions and Limitations
----------------------------------

- Every ``update()`` advances the MRP by the configured ``controlPeriod``; there is no first-update "warm-up" tick.
  The caller must set ``controlPeriod`` to match the rate at which the adapter is being driven (e.g. the simulation
  task rate), otherwise the integrated reference frame will drift relative to true wall-clock time.
- A change of the pointing offset or the mountings through ``reconfigure()`` applies as a step at the next update. The
  output rate and acceleration do not include a rate for this step.
- The module accepts any finite mountings. It does not check that the measured mounting is close to the nominal one;
  the caller is responsible for that check before it configures the module.
- The module uses forward Euler integration. The integration error of the spin angle increases with
  :math:`\Delta t` and :math:`|\Omega|`. Above approximately 0.2 rad per step,
  float32 rounding errors off the spin axis increase quickly. Keep
  :math:`|\Omega|\,\Delta t` well below 0.2 rad.
- All math uses single-precision (float32). Compared to the double-precision Xmera implementation, regression
  tolerances are relaxed to roughly :math:`10^{-5}`.

Test Description
----------------
Regression tests run the algorithm for several steps and compare the output with an independent reference
implementation. Setup tests examine the ``MrpRotationConfig`` validators, the shadow-set storage of the MRPs, and the
normalized storage of the spin axis.
Property tests examine:

- The output is finite.
- With zero rate, the output is the input reference with the constant rotations applied.
- With zero offset and zero rate, the measured component frame sits where the nominal frame sits in the base attitude.
- The component axis along the spin axis stays fixed in the input reference for 2000 steps.
- The output rate obeys the transport theorem.

Component pointing tests examine a camera with a 0.5 deg misalignment: the measured camera z axis lands on the
target, an offset about the camera x axis turns it off the target by the offset angle, the attitude with the offset is
the component-only attitude turned about the measured camera axis, and a spin about the camera z axis holds that axis
still while the camera turns about it. Edge-case tests examine ``reInitialize()``, a new pointing offset through
``setConfig()``, the shadow-set switch, and a 10 deg cone around a target. Fuzz tests use random configurations and
inputs in realistic ranges.

The integrated component pointing test runs the scenarios of the examples above through the guidance modules and
checks that the measured component frame sits where the nominal one sits in the base attitude, turned by the
offset, that the spin axis holds its direction, and that the output rate matches the central difference of the output
attitude.
