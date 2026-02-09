Executive Summary
-----------------
This module compares body rate estimates from an IMU and a star tracker. If the two rates differ by more than a
configurable threshold, the module reports a fault and outputs the IMU rate. Otherwise it outputs the star tracker rate.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages. The module msg variable name is set by the user
from python. The msg type contains a link to the message structure definition, while the description provides
information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - imuSensorBodyInMsg
      - :ref:`IMUSensorBodyMsgF32Payload`
      - IMU body rate input message (angular rate in body frame)
    * - stBodyInMsg
      - :ref:`STAttMsgPayload`
      - star tracker attitude input message (uses omega_BN_B)
    * - navAttMsg
      - :ref:`NavAttMsgPayload`
      - navigation output message containing the selected body rate
    * - rateFaultMsg
      - :ref:`BodyRateFaultMsgPayload`
      - fault output message indicating IMU and star tracker miscompare

Algorithm Description
---------------------
The body rate miscompare algorithm operates on two input vectors, the IMU body rate and the star tracker body rate,
both expressed in body-frame components and in consistent units (rad/s). Let the two inputs be
:math:`\omega_{\text{imu}}` and :math:`\omega_{\text{st}}`.

The algorithm computes the difference

.. math::

    \Delta \omega = \omega_{\text{st}} - \omega_{\text{imu}}

and compares its Euclidean norm to a positive threshold :math:`\tau`:

.. math::

    \|\Delta \omega\| > \tau

If the threshold is exceeded, the algorithm declares a fault and outputs the IMU rate. Otherwise, it outputs the star
tracker rate. The output also includes a boolean flag indicating whether the fault was detected.

Algorithm Assumptions and Limitations
-------------------------------------
- The threshold :math:`\tau` must be strictly positive. A zero or negative threshold is invalid.
- The two input rates must be expressed in the same frame and units.
- The comparison is instantaneous and uses a strict greater-than check. There is no hysteresis, filtering,
  or persistence logic.
- The algorithm does not validate the physical plausibility of the input rates beyond finite arithmetic.

Module Description (Xmera Usage)
--------------------------------
The `BodyRateMiscompare` simulation module provides the Xmera integration layer for the algorithm. It:

- Reads `imuSensorBodyInMsg` (IMU rate) and `stBodyInMsg` (star tracker attitude message).
- Converts the star tracker payload to float and extracts `omega_BN_B`.
- Calls the body rate miscompare algorithm to select the output rate.
- Writes `navAttMsg` with the selected body rate and `rateFaultMsg` with the fault flag.

The module sets the output time tag using the simulation `callTime`.

Module Assumptions and Limitations
----------------------------------
- Both input messages must be linked before `reset`. The module throws an error if either input is not connected.
- The module only uses the body rate from the star tracker message and ignores other attitude fields.
- The module does not perform unit conversions beyond the internal float conversion for the star tracker message.

User Guide
----------
Typical usage in Python is:

.. code-block:: python

    module = bodyRateMiscompareF32.BodyRateMiscompare()
    module.modelTag = "bodyRateMiscompare"
    module.setBodyRateThreshold(body_rate_threshold_rad_per_sec)

    module.imuSensorBodyInMsg.subscribeTo(imu_msg)
    module.stBodyInMsg.subscribeTo(st_msg)

The output body rate is available on `navAttMsg`, and the fault status is available on `rateFaultMsg`.
