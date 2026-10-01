Executive Summary
-----------------

This module applies a fixed duty cycle to a commanded body torque. In the on window, the output torque is equal to
the commanded torque. In the off window, the output torque is zero. The module counts the cycle in control periods.
The cycle is continuous and independent of the torque command.

The module does no arithmetic on the torque. When the gate is on, the output is equal to the input. The gate applies
the same state to all three components. Thus, the output is always the input or zero.

The neighbor modules of this module do all numeric calculations in single precision (``float`` / fp32). The message
payloads of this module are ``float`` arrays.

Use Case: Momentum Desaturation with On/Off Thrusters
-----------------------------------------------------

The module can apply a duty cycle to thrusters that remove momentum from the reaction wheels. The module is between
the momentum management and the thruster force mapping::

    momentumManagement              requested dumping torque   [Nm]
      -> torqueDutyCycle              gated dumping torque       [Nm]
        -> forceTorqueThrForceMapping   per-thruster force         [N]
          -> thrFiringRemainder / thrFiringSchmitt   thruster on-time  [s]
            -> thrusters

An on/off thruster has a fixed thrust, so it cannot give a very small torque. Its smallest action is one pulse at
full thrust for the minimum fire time. Thus, the thrusters supply a dumping torque as a sequence of large pulses.
These pulses cause disturbances in the attitude control loop. The duty cycle changes the *time* in which the
thrusters fire, not the thrust. In the off window, the thrusters do not fire. The reaction wheels then have control
periods in which they correct the pointing. Find the number of control periods that the wheels use to correct the
attitude error of one on window. Set the off window to this number.

The gate is before the mapping. The thruster commands are the same as when the gate is after the mapping, on the
force of each thruster. Two conditions make this result true. The mapping keeps no state between calls. The mapping
also changes a zero torque to a zero force on each thruster. The second condition is true only when the mapping
receives no body force request. This module does not see the body force request and does not apply the gate to it.

Use this module only with on/off thrusters:

- A magnetorquer or a throttleable electric thruster can supply a small continuous torque. Thus, a duty cycle is not
  necessary for these actuators. Set the upstream gains sufficiently low, so that the attitude controller can
  reject the dumping torque.
- For small requests, ``thrFiringRemainder`` already makes its own cycle. It keeps each on-time that is less than
  ``thrMinFireTime`` as a pulse remainder. It then sends one minimum pulse after some cycles. The remainder cycle
  occurs only when

  .. math::

      \frac{F}{F_\text{max}} < \frac{t_\text{min fire}}{T_\text{control}}.

  The condition is true when a proportional on-time is less than the minimum pulse. Above this ratio, the thruster
  fires in each cycle, and the reaction wheels get no control periods without thruster pulses. If the gain in
  ``momentumManagement`` is sufficiently small, this module is not necessary.
- Do **not** put the module on an attitude control path. On such a path, the module removes a commanded torque for
  full control periods. The module then decreases the performance of the attitude control loop.

In this sequence of modules, the duty cycle has three effects:

- The duty ratio decreases the effective gain of the desaturation loop.
- An integrating ``momentumManagement`` has integrator windup in the off windows.
- The first pulse of a new request can wait for a maximum of ``offPeriods`` control periods.

For more data, refer to `Module Behavior Notes`_.

Module Architecture
-------------------

The **algorithm** (``TorqueDutyCycleAlgorithm``) has no framework dependencies. It keeps a validated
``TorqueDutyCycleConfig`` and uses the cadence in `Cadence`_. Its ``update()`` does not cause an exception, and it
gives the gated torque command. The position in the cycle is the only runtime state of the module. This state is
not persistent. Thus, ``reInitialize()`` starts the cycle again, and the module has no
``reInitializeExceptPersistentStates()``.

The **Xmera adapter** (``TorqueDutyCycle``) is a subclass of ``SysModel`` and does all of the messaging. It converts
the C array of the message payload to the ``Eigen::Vector3f`` of the algorithm and back. It writes the output
message at each update. The configuration uses a two-phase initialization. First, the caller sets the public
properties. Then ``reset()`` makes sure that the input message is connected, makes the configuration, and makes the
algorithm. All of the configuration is in the module properties. Thus, the module does not read an input message to
make the configuration.

The **Adamant adapter** is a C shim (``torqueDutyCycleAlgorithm_c.h`` / ``.cpp``). It gives Ada FFI access to the
algorithm through an opaque handle. The torque command goes across the boundary as a ``Vector3f_c``. The
configuration goes across as flattened scalars. ``validateConfig()`` does not cause an exception. Ada uses it to
make sure that a configuration is valid before it calls ``create()`` or ``setConfig()``, which cause an exception.

Message Connection Descriptions
-------------------------------

The table that follows shows all of the module input and output messages. The user connects the module messages
from Python. The message type has a link to the definition of the message structure. The description tells the
function of each message.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - cmdTorqueInMsg
      - :ref:`CmdTorqueBodyMsgF32Payload`
      - Commanded body-frame torque [Nm]. The module reads this message at each update.
    * - cmdTorqueOutMsg
      - :ref:`CmdTorqueBodyMsgF32Payload`
      - Gated body-frame torque [Nm]. This torque is the input in an on period and zero in an off period.
        The module writes this message at each update.

Cadence
-------

One duty cycle has :math:`N_\text{on} + N_\text{off}` control periods, where :math:`N_\text{on}` is ``onPeriods`` and
:math:`N_\text{off}` is ``offPeriods``. The on window is the first positions of the cycle. Let :math:`n` be the number
of the update after the last restart. The output is then equal to the command when

.. math::

    n \bmod (N_\text{on} + N_\text{off}) < N_\text{on}

In all other updates, the output torque is zero. Let :math:`\boldsymbol{L}` be the commanded body torque. The output
is then

.. math::

    \boldsymbol{L}_\text{out} = \begin{cases}
    \boldsymbol{L}, & n \bmod (N_\text{on} + N_\text{off}) < N_\text{on}\\
    \boldsymbol{0}, & \text{otherwise}
    \end{cases}

The cadence has three important properties.

**The cadence is continuous.** The position in the cycle moves forward at each update, independent of the torque
command. Thus, the on windows stay at a fixed phase, and a new command does not start a new on window. A new command
can wait for a maximum of :math:`N_\text{off}` control periods before the output is equal to it.

**The gate applies the same state to all axes.** In one update, the gate applies the same state to each torque
component. Thus, the direction of the gated torque is always the direction of the command.

**The module does not increase the torque.** The output is equal to the command, not a larger value. Thus, the
average torque in one cycle is

.. math::

    \bar{\boldsymbol{L}} = \frac{N_\text{on}}{N_\text{on} + N_\text{off}} \, \boldsymbol{L}.

The duty ratio thus decreases the gain of each loop that goes through the module. Set the upstream gain for this
decrease (refer to `Module Behavior Notes`_).

Module Parameters
-----------------

``reset()`` validates the configuration parameters when it makes the algorithm configuration. A value that is not in
the valid range causes an ``fsw::invalid_argument`` exception, and the module does not make the algorithm.

.. list-table:: Module Configuration Parameters
    :widths: 20 15 30 35
    :header-rows: 1

    * - Parameter
      - Type
      - Valid range
      - Description
    * - onPeriods
      - uint32
      - :math:`\ge 1`
      - [-] Number of sequential control periods at the start of each cycle in which the output is equal to the
        commanded torque. The module rejects zero. With zero, the output torque is always zero, and the module
        stops the command without an indication.
    * - offPeriods
      - uint32
      - all values with ``onPeriods + offPeriods`` :math:`\le` ``UINT32_MAX``
      - [-] Number of sequential control periods in which the output torque is zero. Zero is permitted. With zero,
        the gate is always on, and the duty cycle is disabled. The module rejects only the values that cause an
        overflow of the sum with ``onPeriods``. An overflow gives a cycle that is shorter than its on window.

The two parameters are numbers of **control periods**, not seconds. Thus, the module has no ``controlPeriod``
parameter and no measured time step. The module counts its own calls. Thus, the cadence is exact, because the module
does not change a duration to a number of control periods. But the duration of a cycle in seconds changes with the
schedule rate of the module.

User Guide
----------

The module uses a two-phase initialization. First, set the public configuration properties and connect the input
message. Then ``reset()`` makes and validates the configuration.

.. code-block:: python

    from xmera.fp32 import torqueDutyCycleF32

    module = torqueDutyCycleF32.TorqueDutyCycle()
    module.modelTag = "torqueDutyCycle"

    # Phase 1: set the configuration properties before reset()
    module.onPeriods = 1    # [-] output equal to the command for one control period ...
    module.offPeriods = 4   # [-] ... then zero for four, for a 1-in-5 duty cycle

    # Connect the necessary input message
    module.cmdTorqueInMsg.subscribeTo(cmd_torque_in_msg)

    # Phase 2: reset() makes sure that the message is connected and makes the configuration
    sim.AddModelToTask(task_name, module)

The input message is necessary. If the input message is not connected, ``reset()`` causes an exception.

To give changed configuration properties to a running algorithm, call ``reconfigure()``. The position in the cycle
does not change. To start the cadence again at its on window, call ``reInitialize()``. If you call one of these
functions before ``reset()``, it causes an ``XmeraLifecycleException``.

Module Assumptions and Limitations
----------------------------------

- **The module counts the cadence in calls.** The module must operate at the correct control rate. The duty cycle in
  seconds changes with the task period.
- **The cadence is continuous.** Thus, the output can be zero for a maximum of ``offPeriods`` control periods before
  it is equal to a new command.

Module Behavior Notes
---------------------

- **Set the upstream gain for the duty ratio.** The module does not increase the torque. Thus, the average torque
  is :math:`N_\text{on} / (N_\text{on} + N_\text{off})` of the command. A change of the cadence thus changes the
  effective loop gain of the upstream controller.
- **An upstream integral term has integrator windup in the off windows.** In the off windows, the gate removes the torque, but
  the upstream error stays. Thus, an integrating controller continues to increase its integral term, but the torque
  has no effect. Tune the integral limit and ``offPeriods`` of this module together. A long off window and a large
  integral limit cause an overshoot of the command when the gate is on again.
- **The module discards the commands in the off windows.** The module does not keep these commands for a later
  update, because the upstream integral term already keeps the error. If the module also keeps the command, the
  same error has two integrations. Thus, the module is correct only after a closed-loop command, not after a fixed
  impulse budget.
