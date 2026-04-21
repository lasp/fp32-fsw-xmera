==================================
Convert ST Platform to Body
==================================

-----------------
Executive Summary
-----------------

The ``convertStPlatformToBody`` module converts a star tracker attitude measurement from the sensor (case) frame into
the spacecraft body frame. It takes the inertial-to-case attitude quaternion and case-frame angular velocity produced
by the star tracker and, using a configured body-to-case mounting DCM, outputs the inertial-to-body MRP and body-frame
angular velocity suitable for downstream navigation and control modules.


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
      - Eigen::Matrix3d (adapter) / Eigen::Matrix3f (algorithm)
      - [-]
      - Identity
      - Direction cosine matrix mapping body-frame components to case-frame components


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

- Initializing the module by verifying that input messages are properly linked in ``reset()``
- Logging an error condition if ``stSensorInMsg`` is not connected
- Reading the input message ``stSensorInMsg``
- Converting the sensor message data from ``double`` to ``float``
- Packing the float data into the ``StSensorInput`` struct
- Calling the algorithm layer ``ConvertStPlatformToBodyAlgorithm::update``
- Converting the ``StAttitudeOutput`` struct fields back to ``double``
- Writing the ``stAttOutMsg`` output message
- Accepting the ``dcm_CB`` mounting matrix from Python as a ``double`` and forwarding it to the algorithm as ``float``

**2. Algorithm Layer**

The algorithm (``ConvertStPlatformToBodyAlgorithm``) performs the core mathematical computations and is fully decoupled
from the Xmera framework. It operates exclusively on the float input and output structs defined in
:ref:`convertStPlatformToBodyTypes.h` and has no dependency on ``SysModel``, message readers, or writers.


-------------------------------------
Input Constraints and Assumptions
-------------------------------------

- The star tracker case is rigidly mounted to the hub so that
:math:`^{C}\boldsymbol{\omega}_{CN} = [CB]\, {}^{B}\boldsymbol{\omega}_{BN}`
- ``qInrtl2Case`` is a unit quaternion representing the rotation from the inertial frame
:math:`\mathcal{N}` to the case frame :math:`\mathcal{C}`
- ``dcm_CB`` is a proper orthogonal direction cosine matrix (det = +1)
- Input angular velocity ``omega_CN_C`` is expressed in the case frame


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

**3. Angular velocity transformation**

Because the sensor is rigidly mounted, the angular velocity in the body frame is obtained by rotating the
case-frame measurement through the mounting DCM:

:math:`{}^{B}\boldsymbol{\omega}_{BN} = [CB]^{T}\, {}^{C}\boldsymbol{\omega}_{CN}`


----------
User Guide
----------

The module is configured from Python as::

    module = convertStPlatformToBodyF32.ConvertStPlatformToBody()
    module.modelTag = "stPlatformToBody"
    module.setDcmCB(dcm_CB)
    module.stSensorInMsg.subscribeTo(stSensorMsg)

The ``dcm_CB`` mounting matrix must be set before ``reset()`` is called so that the algorithm uses the
intended mounting configuration on the first update cycle. If ``dcm_CB`` is never set, an identity DCM is
used (body and case frames are assumed to coincide).
