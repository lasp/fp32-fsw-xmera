Executive Summary
-----------------
This module compares body rate estimates from an IMU and a star tracker. If the two rates differ by more than a
configurable threshold for a configurable number of consecutive updates, the module reports a fault and outputs the IMU
rate. Otherwise it outputs the star tracker rate. Once a fault is declared it persists across resets; re-applying the
configuration clears it.

Message Connection Descriptions
-------------------------------
The following table lists all module input and output messages. Message connections are set by the
user from Python.

.. list-table:: Module I/O Messages
    :widths: 30 30 50
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
    * - navAttOutMsg
      - :ref:`NavAttMsgF32Payload`
      - navigation output message containing the selected body rate
    * - rateFaultOutMsg
      - :ref:`BodyRateFaultMsgPayload`
      - fault output message indicating IMU and star tracker miscompare

Module Parameters
-------------------------------
The following table lists all the module parameters that can be set.

.. list-table:: Module Parameters
    :widths: 30 15 10 10 40 30
    :header-rows: 1

    * - Parameter Name
      - Type
      - Units
      - Default
      - Description
      - Bounds
    * - bodyRateThreshold
      - float
      - [rad/s]
      - 0.0
      - Euclidean norm threshold for rate miscompare detection
      - Must be finite and strictly positive (validated when the config is built in ``reset()``)
    * - faultPersistenceLimit
      - uint32_t
      - [-]
      - 1
      - Number of consecutive threshold violations required to declare a fault
      - Must be >= 1 (validated when the config is built in ``reset()``)
    * - useImuRates
      - bool
      - [-]
      - false
      - When true, always output IMU rates regardless of miscompare detection
      - N/A

Algorithm Input/Output
-------------------------------
The following tables list the inputs and outputs of the pure algorithm ``update()`` method, independent of the Xmera
messaging layer.

.. list-table:: Algorithm Inputs
    :widths: 25 25 10 40
    :header-rows: 1

    * - Variable
      - Type
      - Units
      - Description
    * - imuOmega_BN_B
      - Eigen::Vector3f
      - [rad/s]
      - IMU body rate in body-frame components
    * - stOmega_BN_B
      - Eigen::Vector3f
      - [rad/s]
      - Star tracker body rate in body-frame components

.. list-table:: Algorithm Outputs (BodyRateMiscompareOutput)
    :widths: 25 25 10 40
    :header-rows: 1

    * - Variable
      - Type
      - Units
      - Description
    * - omega_BN_B
      - Eigen::Vector3f
      - [rad/s]
      - Selected body rate (IMU if fault, star tracker otherwise)
    * - bodyRateFaultDetected
      - bool
      - [-]
      - True if fault persistence limit has been reached

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

If the threshold is exceeded, a persistence counter is incremented. If the threshold is not exceeded, the counter resets
to zero. When the persistence counter reaches the configurable fault persistence limit :math:`N`, the algorithm declares
a fault and outputs the IMU rate. Otherwise, it outputs the star tracker rate. The output also includes a boolean flag
indicating whether the fault was detected.

Once a fault has been declared, the algorithm continues to output the IMU rate on all subsequent calls. The persistence
counter is no longer evaluated. The configured ``useImuRates`` value is held in the immutable configuration; an internal
flag tracks the latched fault state separately. To fully recover from a detected fault, the caller can call
``reInitializeAll()`` (which clears both the persistence counter and the latched fault), or trigger a module ``reset()``
(which reconstructs the algorithm and likewise clears all state).

The ``reInitialize()`` method sets the persistence counter back to zero but preserves the latched fault state,
allowing the counter to be cleared independently of the rate source selection. ``reInitializeAll()`` clears both, and
the module-level ``reset()`` reconstructs the algorithm to produce a fully cleared state.

The ``useImuRates`` configuration value forces the algorithm to output IMU rates without waiting for a fault to be
detected. When set to true, the algorithm bypasses the miscompare logic and always outputs the IMU rate.

Algorithm Assumptions and Limitations
-------------------------------------
- The two input rates must be expressed in the same frame and units.
- A declared fault persists across ``reInitialize()`` (which clears only the persistence counter); ``reInitializeAll()``
  (or a module ``reset()``, which reconstructs the algorithm) clears the latched fault.
- The algorithm does not validate the physical plausibility of the input rates beyond finite arithmetic.

Module Description (Xmera Usage)
--------------------------------
The `BodyRateMiscompare` simulation module provides the Xmera integration layer for the algorithm. It:

- Reads `imuSensorBodyInMsg` (IMU rate) and `stBodyInMsg` (star tracker attitude message).
- Converts the star tracker payload to float and extracts `omega_BN_B`.
- Calls the body rate miscompare algorithm to select the output rate.
- Writes `navAttMsg` with the selected body rate and `rateFaultMsg` with the fault flag.

The module sets the output time tag using the simulation `callTime`.

User Guide
----------
Typical usage in Python is::

    module = bodyRateMiscompareF32.BodyRateMiscompare()
    module.modelTag = "bodyRateMiscompare"
    module.bodyRateThreshold = body_rate_threshold_rad_per_sec
    module.faultPersistenceLimit = 3  # require 3 consecutive violations before declaring fault
    module.useImuRates = True  # optionally force IMU rate output without waiting for a fault

    module.imuSensorBodyInMsg.subscribeTo(imu_msg)
    module.stBodyInMsg.subscribeTo(st_msg)

The configuration properties (``bodyRateThreshold``, ``faultPersistenceLimit``, ``useImuRates``) must be set before
``reset()`` is called: the adapter builds and validates the immutable configuration at reset, raising on invalid values.

The output body rate is available on `navAttOutMsg`, and the fault status is available on `rateFaultOutMsg`.

The ``faultPersistenceLimit`` parameter (default 1) controls how many consecutive threshold violations are required
before the fault is declared. Setting it to 1 means the fault triggers on the first violation.
