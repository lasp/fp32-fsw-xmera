==============================
Ephemeris to Navigation Converter
==============================

====================
Module Description
====================

Introduction
============
The ``ephemNavConverter`` flight software algorithm is a simple module that takes an ephemeris input containing position, velocity, and time and republishes these into a translational navigation message payload.

Module Input/Output Messages
============================
The following table lists all the module input and output messages. The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - **Msg Variable Name**
      - **Msg Type**
      - **Description**

    * - ``ephInMsg``
      - :ref:`EphemerisMsgF32Payload`
      - Input message containing the ephemeris data, including time tag, position, and velocity. The attitude fields of MRPs and angular velocity will be ignored for this module.

    * - ``stateOutMsg``
      - :ref:`NavTransMsgF32Payload`
      - Output message containing the translation navigation data, including time tag, position, and velocity, all expressed in inertial frame.

Algorithm Flow
==============
At every update cycle, the ``ephemNavConverter`` module performs the following steps:

- It reads the latest ephemeris navigation data from ``ephInMsg``.
- Republishes the ephemeris data of time tag, position and velocity into the output navigation translation message ``NavTransMsgF32Payload``, while the unused field of ``vehAccumDv`` remains zero.
- Writes and outputs the ``stateOutMsg`` for downstream FSW modules.

Algorithm Functions
====================
Below is a list of functions that this flight software module performs:

- The algorithm will read the latest data from ``ephInMsg`` at each update.
- It will copy the ephemeris input data of ``timeTag`` into the output navigation time tag [s].
- Then copy the position ephemeris input data of ``r_BdyZero_N`` to the translation output data ``r_BN_N``.
- And copy the velocity ephemeris input data of ``v_BdyZero_N`` to the translation output data ``v_BN_N``.
- The unused output message field of ``vehAccumDV`` will remain zero as it's not part of the translational data.
- All the output fields will be published through ``stateOutMsg``.

where:

- ``timeTag`` is the current time-tag associated with measurements [s].
- ``r_BdyZero_N`` and ``r_BN_N`` are the current inertial position vectors expressed in the inertial frame components [m].
- ``v_BdyZero_N`` and ``v_BN_N`` are the current inertial velocity vectors expressed in the inertial frame components [m/s].
- ``vehAccumDv`` is the total accumulated delta-velocity that must always remain zero [m/s].

Module Assumptions and Limitations
======================================

- It is the user's responsibility to ensure that all the ephemeris input data are provided in SI units:

  - position in meters [m].
  - Velocity in meters per second [m/s].
  - Time tag in seconds [s].

- All the ephemeris data must be expressed in the inertial frame, as the module does not perform frame checks.
- In the input message ``ephInMsg``, only translation fields are used and attitude navigation fields (e.g. ``sigma_BN``, ``omega_BN_B``) are ignored.
- In the output message ``stateOutMsg``, the ``vehAccumDv`` is always zero as the input ephemeris message does not provide ∆V information.


Test Description and Success Criteria
=====================================
The ephemeris navigation converter unit test is located in ``algorithms/ephemNavConverter/_tests/test_ephemNavConverter.py``. This test verifies that the module works
as expected by simply converting ephemeris input data into a translation navigation output.

In this unit test, a built-in module ``astroFunctions`` is used to validate the module by comparing the values of Earth's position and velocity in the inertial frame with the output of the translation navigation message. The function converts Gregorian [2018, 10, 16] date to a Julian date and computes the Keplerian parameters. These parameters are converted to Earth's position and velocity in PQW frame and then rotate them to the inertial frame.

For the success criteria, the output values of Earth's position and velocity must be identical within a tolerance of :math:`10` m for position and :math:`10^{-4}` m/s for velocity.
