Executive Summary
-----------------

The ``convertStPlatformToBody`` module converts a star tracker attitude measurement from the sensor (case) frame into
the spacecraft body frame. It takes the inertial-to-case attitude quaternion and case-frame angular velocity quaternion
from the star tracker platform solution (``AscPlatformAttitudeTmMsgF32Payload``) and converts them using a configured
mounting MRP.

The output is the inertial-to-body MRP :math:`\boldsymbol{\sigma}_{B/N}` and the body-frame angular velocity
:math:`{}^{\mathcal{B}}\boldsymbol{\omega}_{B/N}`, for downstream navigation and control modules. The sensor is
rigidly mounted, so the angular velocity is only rotated into body components.

All numeric calculation is single-precision (FP32). The module has one algorithm and two interface
adapters. A ``SysModel`` adapter connects it to the Xmera system with messages. A C shim connects it
to the Adamant system through the C/Ada FFI.

Module Architecture
-------------------

The module is split into a thin adapter that handles framework integration and an algorithm class that contains the
pure math.

The algorithm class is ``ConvertStPlatformToBodyAlgorithm``. It keeps no cycle-to-cycle measurement state.
``setConfig()`` computes the inverse mounting MRP (``sigma_BC``) and the case-to-body DCM (``dcm_BC``) once per
configuration change.

Adapter Layer
~~~~~~~~~~~~~

The adapter class is ``ConvertStPlatformToBody``. It inherits from ``SysModel`` and owns the input and output
messages. Each ``updateState()`` copies ``platformAttitude`` and ``platformAngularVelocity`` into ``q_CN`` and
``dq_CN`` without conversion or reordering, passes them to the algorithm, and publishes ``sigma_BN`` and
``omega_BN_B`` with ``solutionTimestamp`` as the time tag in seconds (``double``). The output message is written every
cycle. If the input message has not been written, zero quaternions and a zero time tag are used, so the output is
``sigma_BC`` with zero angular velocity and a zero time tag.

.. list-table:: Module I/O Messages
    :widths: 24 32 44
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - ``stSensorInMsg``
      - :ref:`AscPlatformAttitudeTmMsgF32Payload`
      - Star tracker platform attitude solution: attitude quaternion, rate quaternion and time tag.
    * - ``stAttOutMsg``
      - :ref:`STAttMsgF32Payload`
      - Body-frame attitude output: inertial-to-body MRP, body-frame angular velocity and the time tag passed
        through.

Configuration
~~~~~~~~~~~~~

``sigma_CB`` is validated in ``ConvertStPlatformToBodyConfig::create()`` when the adapter's ``reset()`` or
``reconfigure()`` runs; the algorithm never sees an invalid configuration.

.. list-table:: Configuration parameters
    :widths: 16 28 8 12 36
    :header-rows: 1

    * - Parameter Name
      - Type
      - Units
      - Default
      - Description
    * - ``sigma_CB``
      - ``Eigen::Vector3f``
      - [-]
      - Zero
      - MRP of the case frame relative to the body frame. Must be finite. A shadow set is stored as its principal
        set. The default (zero) assumes the star tracker case frame is aligned with the spacecraft body frame; set
        the actual mounting MRP otherwise.

Two-Phase Initialization
~~~~~~~~~~~~~~~~~~~~~~~~

The adapter is set up in two phases. First, create the module, set ``sigma_CB``, connect ``stSensorInMsg``, and add
the module to the simulation task; nothing is checked in this phase and the algorithm is not yet built. Second,
``reset()``, called by ``InitializeSimulation()``, checks that ``stSensorInMsg`` is connected, validates ``sigma_CB``,
and builds the algorithm. It throws if either check fails::

    module = convertStPlatformToBodyF32.ConvertStPlatformToBody()
    module.modelTag = "stPlatformToBody"
    module.sigma_CB = sigma_CB
    module.stSensorInMsg.subscribeTo(stSensorMsg)

    scSim.AddModelToTask(simTaskName, module)
    scSim.InitializeSimulation()  # runs reset()

After ``reset()``, ``updateState()`` runs each cycle. To change the mounting, set ``sigma_CB`` and then call
``reconfigure()``; the new value is used from the next ``updateState()``. If ``updateState()`` or
``reconfigure()`` is called before ``reset()``, an ``XmeraLifecycleException`` is raised.

Mathematical Formulation
------------------------

The module works with three reference frames: the inertial frame :math:`\mathcal{N}`, the spacecraft body frame
:math:`\mathcal{B}`, and the star tracker case (platform) frame :math:`\mathcal{C}`.

Inertial-to-Case Attitude
~~~~~~~~~~~~~~~~~~~~~~~~~

The star tracker gives the inertial-to-case attitude as a scalar-last quaternion
:math:`\mathbf{q}_{C/N} = [q_0, q_1, q_2, q_3]^T`, with :math:`q_3` the scalar part. It is reordered to the
scalar-first Euler parameter set

.. math::

    \boldsymbol{\beta}_{C/N} = [q_3, q_0, q_1, q_2]^T

and converted to the equivalent MRP set :math:`\boldsymbol{\sigma}_{C/N}` (``epToMrp``).

Inertial-to-Body Attitude
~~~~~~~~~~~~~~~~~~~~~~~~~

The case-to-body MRP is the inverse of the configured mounting MRP,

.. math::

    \boldsymbol{\sigma}_{B/C} = -\boldsymbol{\sigma}_{C/B}.

The inertial-to-body MRP is the composition of the inertial-to-case rotation followed by the case-to-body rotation,
:math:`[BN] = [BC][CN]`, computed by MRP addition (``addMrp``):

.. math::

    \boldsymbol{\sigma}_{B/N} = \boldsymbol{\sigma}_{B/C} \oplus \boldsymbol{\sigma}_{C/N}.

Angular Velocity from the Rate Quaternion
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The star tracker reports the case-frame angular velocity packed in quaternion form (micro-ASC ICD,
ASC-DTU-ICD-3004B, Angular Velocity Quaternion Struct),

.. math::

    \delta\mathbf{q}_{C/N} = \left[\hat{\mathbf{e}}\sin\!\left(\tfrac{\lVert\boldsymbol{\omega}\rVert}{2}\right),\
                       \cos\!\left(\tfrac{\lVert\boldsymbol{\omega}\rVert}{2}\right)\right],

with :math:`\lVert\boldsymbol{\omega}\rVert` in rad/s. It is a rate, not a rotation over a sample interval, so no
time step is needed: inverting the packing gives :math:`{}^{\mathcal{C}}\boldsymbol{\omega}_{C/N}` in rad/s
directly. With :math:`\delta\mathbf{q}_v = [\delta q_0, \delta q_1, \delta q_2]^T`,

.. math::

    {}^{\mathcal{C}}\boldsymbol{\omega}_{C/N} =
    \frac{2\,\mathrm{atan2}\!\left(\lVert\delta\mathbf{q}_v\rVert,\ \delta q_3\right)}{\lVert\delta\mathbf{q}_v\rVert}\,
    \delta\mathbf{q}_v.

The four-quadrant arctangent is used in place of :math:`\arccos` so that small rates (where
:math:`\delta q_3 \approx 1`) retain float32 precision. When :math:`\lVert\delta\mathbf{q}_v\rVert = 0` the angular
velocity is set to zero, avoiding a divide-by-zero.

Body-Frame Angular Velocity
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Because the sensor is rigidly mounted, :math:`\boldsymbol{\omega}_{B/N} = \boldsymbol{\omega}_{C/N}`, so only the
components change. The case-frame components are transformed into body-frame components with the transpose of the
mounting DCM, :math:`[BC] = [CB]^T`, where :math:`[CB]` is the DCM of the configured mounting MRP
:math:`\boldsymbol{\sigma}_{C/B}`:

.. math::

    {}^{\mathcal{B}}\boldsymbol{\omega}_{B/N} = [BC]\, {}^{\mathcal{C}}\boldsymbol{\omega}_{C/N}.

Assumptions and Limitations
---------------------------

* The star tracker case is rigidly mounted to the hub, so :math:`\boldsymbol{\omega}_{B/N} = \boldsymbol{\omega}_{C/N}`
  and ``sigma_CB`` is constant between reconfigurations.
* ``q_CN`` must have length 1, as every attitude quaternion does. The micro-ASC ICD defines it as unit length, so
  the module does not check or renormalize it.
* If the input message has not been written, the output is not a valid measurement and carries a zero time tag
  (see Adapter Layer).
