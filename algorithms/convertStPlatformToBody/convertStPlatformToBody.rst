==================================
Convert ST Platform to Body
==================================

-----------------
Executive Summary
-----------------

The ``convertStPlatformToBody`` module converts a star tracker attitude measurement from the sensor (case) frame into
the spacecraft body frame. It takes the inertial-to-case attitude quaternion and case-frame delta quaternion produced
by the star tracker and, using a configured body-to-case mounting DCM, outputs the inertial-to-body MRP and body-frame
angular velocity suitable for downstream navigation and control modules. The adapter layer accepts the legacy
angular-velocity field on ``STSensorMsgPayload`` and converts it to a delta quaternion before invoking the algorithm,
so upstream producers may continue to publish angular velocity until the message layout is updated.


-------------------------------
Module Input/Output Messages
-------------------------------

The following table lists all the module input and output messages:

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - stSensorInMsg
      - :ref:`STSensorMsgPayload`
      - Star tracker sensor measurement input message (inertial-to-case attitude and case-frame rate)
    * - stAttOutMsg
      - :ref:`STAttMsgPayload`
      - Star tracker body-frame attitude output message (inertial-to-body MRP and body-frame rate)


-------------------------------
Module Parameters
-------------------------------

The following table lists all the module parameters that can be set.

.. list-table:: Module Parameters
    :widths: 25 20 10 20 40
    :header-rows: 1

    * - Parameter Name
      - Type
      - Units
      - Default
      - Description
    * - dcm_CB
      - Eigen::Matrix3f
      - [-]
      - Identity
      - Direction cosine matrix mapping body-frame components to case-frame components. Must be a valid
        DCM (orthonormal, det +1); validated when the configuration is built in ``reset()``.


-------------------------------
Module Architecture
-------------------------------

The ``convertStPlatformToBody`` module is structured into two layers:

1. Adapter layer (FSW interface)
2. Algorithm layer (pure mathematical operations)

The algorithm represents the pure mathematical logic of the module, while the adapter handles the simulation framework
interface.

**1. Adapter Layer**

The adapter (``ConvertStPlatformToBody``) provides the Xmera-facing interface and is responsible for:

- Initializing the module by verifying that input messages are properly linked in ``reset()``, throwing
  ``std::invalid_argument`` if ``stSensorInMsg`` is not connected
- Building a validated ``ConvertStPlatformToBodyConfig`` from the ``dcm_CB`` property and constructing the algorithm
  in ``reset()`` (two-phase initialization); ``updateState()`` raises ``XmeraLifecycleException`` if called first
- Reading the input message ``stSensorInMsg``
- Converting the inertial-to-case quaternion from ``double`` to a ``float`` ``Eigen::Vector4f``
- Converting the message time tag from seconds (``double``) to nanoseconds (``uint64_t``) for the output message
- Converting the case-frame angular velocity on the incoming message to a unit delta quaternion ``dq_CN``
  (``Eigen::Vector4f``) before handing it to the algorithm
- Calling the algorithm layer ``ConvertStPlatformToBodyAlgorithm::update``
- Converting the ``StAttitudeOutput`` Eigen fields back to ``double`` for the output payload
- Writing the ``stAttOutMsg`` output message

**2. Algorithm Layer**

The algorithm (``ConvertStPlatformToBodyAlgorithm``) performs the core mathematical computations and is fully decoupled
from the Xmera framework. Its ``update`` takes the inertial-to-case quaternion and case-frame delta quaternion as
``Eigen::Vector4f`` values and returns the ``StAttitudeOutput`` struct (inertial-to-body MRP and body-frame angular
velocity, both ``Eigen::Vector3f``) declared in ``convertStPlatformToBodyAlgorithm.h``. It has no dependency on
``SysModel``, message readers, or writers. The pure-C POD mirrors used by the C shim live in
:ref:`convertStPlatformToBodyTypes.h`.


-------------------------------------
Input Constraints and Assumptions
-------------------------------------

- The star tracker case is rigidly mounted to the hub so that
  :math:`^{C}\boldsymbol{\omega}_{CN} = [CB]\, {}^{B}\boldsymbol{\omega}_{BN}`
- ``q_CN`` is a unit quaternion representing the rotation from the inertial frame
  :math:`\mathcal{N}` to the case frame :math:`\mathcal{C}`
- ``dcm_CB`` is a proper orthogonal direction cosine matrix (det = +1)
- ``dq_CN`` is a unit delta quaternion in scalar-last convention,
  :math:`\delta \boldsymbol{q}_{CN} = [\sin(\theta/2)\,\hat{\boldsymbol{e}},\ \cos(\theta/2)]`,
  representing a one-sample case-frame rotation about unit axis :math:`\hat{\boldsymbol{e}}` by angle :math:`\theta`
- A zero vector part (:math:`\lVert[\delta q_0, \delta q_1, \delta q_2]\rVert = 0`) is treated as a zero rotation
  and yields :math:`^{C}\boldsymbol{\omega}_{CN} = \boldsymbol{0}`


-------------------------------
Mathematical Formulation
-------------------------------

The module works with three reference frames:

- :math:`\mathcal{N}`: inertial frame
- :math:`\mathcal{B}`: spacecraft body (hub) frame
- :math:`\mathcal{C}`: star tracker case (platform) frame

**1. Quaternion to MRP conversion**

The inertial-to-case attitude is converted from an Euler parameter (quaternion) set to the equivalent Modified
Rodrigues Parameter set:

:math:`\boldsymbol{\sigma}_{CN} = \mathrm{epToMrp}(\boldsymbol{\beta}_{CN})`

**2. Inertial-to-body MRP**

The case-to-body MRP is obtained from the inverse of the configured body-to-case DCM:

:math:`\boldsymbol{\sigma}_{BC} = \mathrm{dcmToMrp}\!\left([CB]^{T}\right) = \mathrm{dcmToMrp}\!\left([BC]\right)`

The inertial-to-body MRP is then computed by MRP addition:

:math:`\boldsymbol{\sigma}_{BN} = \boldsymbol{\sigma}_{BC} \oplus \boldsymbol{\sigma}_{CN}`

**3. Delta quaternion to angular velocity**

The case-frame angular velocity is recovered from the incoming unit delta quaternion by inverting the standard
axis-angle-to-quaternion mapping. The four-quadrant arctangent is used in place of :math:`\arccos` so that
near-identity rotations (where :math:`\delta q_3 \approx 1`) retain float32 precision:

.. math::

    {}^{C}\boldsymbol{\omega}_{CN} =
    \dfrac{2\,\mathrm{atan2}\!\left(\lVert[\delta q_0, \delta q_1, \delta q_2]\rVert,\ \delta q_3\right)}
          {\lVert[\delta q_0, \delta q_1, \delta q_2]\rVert}\,
    [\delta q_0, \delta q_1, \delta q_2]^{T}

When the vector part has zero norm the recovered angular velocity is taken to be
:math:`\boldsymbol{0}`, avoiding a divide-by-zero at the identity rotation.

**4. Angular velocity transformation**

Because the sensor is rigidly mounted, the angular velocity in the body frame is obtained by rotating the
case-frame measurement through the mounting DCM:

:math:`{}^{B}\boldsymbol{\omega}_{BN} = [CB]^{T}\, {}^{C}\boldsymbol{\omega}_{CN}`


----------
User Guide
----------

The module is configured from Python as::

    module = convertStPlatformToBodyF32.ConvertStPlatformToBody()
    module.modelTag = "stPlatformToBody"
    module.dcm_CB = dcm_CB
    module.stSensorInMsg.subscribeTo(stSensorMsg)

The ``dcm_CB`` mounting matrix must be set before ``reset()`` is called so that the algorithm uses the
intended mounting configuration on the first update cycle. If ``dcm_CB`` is never set, an identity DCM is
used (body and case frames are assumed to coincide).
