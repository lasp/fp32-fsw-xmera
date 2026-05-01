Executive Summary
-----------------

This module schedules two control torques such that they can be applied simultaneously, one at a time, or neither
applied, and combines them into a single output message. It is useful for systems with two coupled degrees of
freedom, where changes in one controlled variable can affect the other and prevent convergence.

This is the FP32 port: all torques and the switch time are single-precision (``float``) and message payloads are the
F32 variants. Numerical accuracy expectations are correspondingly loosened (typical tolerance ``1e-6``).


Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages. The module msg connection is set by the user from
Python. The msg type contains a link to the message structure definition, while the description provides information
on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - motorTorqueOutMsg
      - :ref:`ArrayMotorTorqueMsgF32Payload`
      - Output array motor torque message (paired torques from input 1 and input 2).
    * - effectorLockOutMsg
      - :ref:`ArrayEffectorLockMsgF32Payload`
      - Output per-motor lock-flag message (1 = locked / torque suppressed, 0 = free / torque applied).
    * - motorTorque1InMsg
      - :ref:`ArrayMotorTorqueMsgF32Payload`
      - First input motor torque.
    * - motorTorque2InMsg
      - :ref:`ArrayMotorTorqueMsgF32Payload`
      - Second input motor torque.


Module Assumptions and Limitations
----------------------------------
The two input torques are always read and written through to ``motorTorqueOutMsg`` unchanged; only the
``effectorLockOutMsg`` reflects the schedule. Downstream consumers are expected to honour the lock flag — the module
itself does not zero the torque of a "locked" effector.

The schedule is a function of elapsed seconds since the most recent ``reset()``. The adapter converts ``callTime``
(nanoseconds) into seconds via ``kNano2Sec`` and casts the result to ``float`` for comparison against ``tSwitch``.

The ``Config`` validator rejects an unrecognized ``LockFlag`` value or a negative ``tSwitch`` at construction time;
``update()`` itself does not throw.


Detailed Module Description
---------------------------
The module reads a ``LockFlag`` enumerator and a ``tSwitch`` scalar from the user. ``LockFlag`` decides how the input
torques are scheduled; for each motor, the lock flag is ``0`` if its torque is currently being applied and ``1`` if
its effector is locked:

  - ``LockFlag::BothFree`` (0): both motor torques are applied for all ``t``.
  - ``LockFlag::LockSecondThenFirst`` (1): second is locked while ``t <= tSwitch``; first is locked once ``t > tSwitch``.
  - ``LockFlag::LockFirstThenSecond`` (2): first is locked while ``t <= tSwitch``; second is locked once ``t > tSwitch``.
  - ``LockFlag::BothLocked`` (3): both effectors are locked for all ``t``.

The ``motorTorqueOut.motorTorque`` array contains the first input's torque at index ``0`` and the second input's
torque at index ``1`` regardless of the lock flag.


User Guide
----------
The module follows the two-phase init lifecycle. Set the public configuration properties before calling
``reset()`` (which the simulation framework invokes from ``InitializeSimulation()``). The properties are validated at
``reset()`` via ``TorqueSchedulerConfig::create``::

    scheduler = torqueSchedulerF32.TorqueScheduler()
    scheduler.modelTag = "torqueScheduler"
    scheduler.lockFlag = torqueSchedulerF32.LockFlag_LockSecondThenFirst
    scheduler.tSwitch = 5.0
    unitTestSim.AddModelToTask(unitTaskName, scheduler)

The module is configurable with the following parameters:

.. list-table:: Module Parameters
   :widths: 20 20 60
   :header-rows: 1

   * - Parameter
     - Valid range
     - Description
   * - ``lockFlag``
     - One of ``LockFlag_BothFree``, ``LockFlag_LockSecondThenFirst``, ``LockFlag_LockFirstThenSecond``,
       ``LockFlag_BothLocked``
     - Selects the schedule. Defaults to ``LockFlag_BothFree``.
   * - ``tSwitch``
     - ``>= 0.0`` (seconds)
     - Time at which the schedule switches between motor inputs. Defaults to ``0.0``.
