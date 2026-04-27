.. raw:: latex

    {\LARGE \textbf{thrFiringSchmitt}}

Executive Summary
-----------------

A Schmitt trigger logic is implemented to map a desired thruster force value into a thruster on command time.

The module reads in the attitude control thruster force values for both on- and off-pulsing scenarios, and then maps
this into a time which specifies how long a thruster should be on. The thruster configuration data is read in through a
separate input message in the reset method. The Schmitt trigger allows for an upper and lower bound where the thruster
is either turned on or off. The paper `Steady-State Attitude and Control Effort Sensitivity Analysis of Discretized
Thruster Implementations <https://doi.org/10.2514/1.A33709>`__ includes a detailed discussion on the Schmitt Trigger
and compares it to other thruster firing methods.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg connection is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 35 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - thrForceInMsg
      - :ref:`THRArrayCmdForceMsgPayload`
      - thruster force input message
    * - onTimeOutMsg
      - :ref:`THRArrayOnTimeCmdMsgPayload`
      - thruster on-time output message
    * - thrConfInMsg
      - :ref:`THRArrayConfigMsgPayload`
      - Thruster array configuration input message

Module Parameters
-------------------------------
The following table lists all the module parameters than can be set. The parameters are optional unless indicated
(if not specified default is used).

.. list-table:: Module Parameters
    :widths: 30 30 10 10 30 30
    :header-rows: 1

    * - Parameter Name
      - Type
      - Units
      - Default
      - Description
      - Bounds
    * - levelOn (required)
      - float
      - [-]
      - 0
      - ON duty cycle fraction
      - 0.0 < levelOn :math:`\le` 1.0 (checked in setter)
    * - levelOff
      - float
      - [-]
      - 0
      - OFF duty cycle fraction
      - 0.0 :math:`\le` levelOff < 1.0 (checked in setter)
    * - thrMinFireTime (required)
      - float
      - [s]
      - 0
      - [s] Minimum ON time for thrusters
      - Must be greater than zero (checked in setter)
    * - baseThrustState
      - enum PulsingRegime
      - [-]
      - 0
      - Indicates on-pulsing (0) or off-pulsing (1)
      - N/A
    * - controlPeriod (required)
      - float
      - [s]
      - 0
      - Control period (time between two update calls, i.e. 1/fsw_rate)
      - Must be greater than zero (checked in setter)

Additionally, it is checked that ``levelOn`` is greater than ``levelOff``.

Module Assumptions and Limitations
----------------------------------

The module assumes that the incoming forces :math:`F_{i}` can be both
positive or negative, depending if an on- or off-pulsing mode is being
implemented. The particular mode is set through ``baseThrustState``.
It is also assumed that ``thrMinFireTime`` is less than the control period.

Initialization
--------------
The module is configured by::

    module = thrFiringSchmitt.ThrFiringSchmitt()
    module.modelTag = "thrFiringSchmitt"
    module.levelOn = 0.75
    module.levelOff = 0.25
    module.thrMinFireTime = 0.02
    module.baseThrustState = 0  # on-pulsing
    module.controlPeriod = 0.5

Detailed Module Description
---------------------------

This module implements a Schmitt trigger thruster firing logic. Here if the minimum desired on-time
:math:`t_{\text{min}}` is specified. If the commanded on-time
:math:`t_{i}>t_{\text{min}}` then the thruster is turned off for the
duration of :math:`t_{i}`. If :math:`t_{i}< t_{\text{min}}` then the
Schmitt trigger logic is invoked. Let :math:`l` be the current thruster
duty cycle relative to this minimum thruster firing time.

.. math:: l = \frac{t_{i}}{t_{\text{min}}}

If :math:`l` is larger than a threshold :math:`l_{\text{on}}` then the
thruster control time is set to :math:`t_{i} = t_{\text{min}}`. Once on,
the thruster level must drop below a lower threshold
:math:`l_{\text{off}}` to turn off again. The benefit of this logic is
that it provides a good balance between fuel efficiency and pointing
accuracy.

Module Input and Output Messages
--------------------------------

The module reads in two messages. One message contains the thruster
configuration message from which the maximum thrust force value for each
thruster is extracted and stored in the module. This message is only
read in on ``reset()``.

The second message reads in an array of requested thruster force values
with every ``Update()`` function call. These force values :math:`F_{i}`
can be positive if on-pulsing is requested, or negative if off-pulsing
is required. On-pulsing is used to achieve an attitude control torque
onto the spacecraft by turning on a thruster. Off-pulsing assumes a
thruster is nominally on, such as with doing an extended orbit
correction maneuver, and the attitude control is achieved by doing
periodic off-pulsing.

The output of the module is a message containing an array of thruster
on-time requests. If these on-times are larger than the control period,
then the thruster remains on only for the control period upon which the
on-criteria is reevaluated.

reset() Functionality
---------------------

- The thruster configuration message is read in and the number of
  thrusters is stored in the module variable ``numThrusters``. The
  maximum force per thruster is stored in ``maxThrust``.

- The previous thruster state variable ``prevThrustState`` is set to off
  (i.e. false)

Update() Functionality
----------------------

The goal of the ``update()`` method is to read in the current attitude
control thruster force message and map these into a thruster on-time
output message using the Schmitt trigger
logic.:raw-latex:`\cite{Alcorn:2016rz}` The module sets a desired
minimum thruster on time :math:`t_{\text{min}}`. This is typically not
set to the lower limit of the thruster resolution, but rather to a value
that provides a good duty cycle and avoids excessive short on-off
switching. The cost naturally is a reduced pointing capability. Let the
duty cycle :math:`l` be defined as the ratio between the commanded on
time :math:`t_{i}` and :math:`t_{\text{min}}`. The thruster is turned on
for a period :math:`t_{i} = t_{\text{min}}` if :math:`l` is larger than
:math:`l_{\text{on}}`. The thruster then remains on until :math:`l`
drops below a lower threshold :math:`l_{\text{off}}`. The benefit of
this method is that it provides a good balance between fuel usage and
pointing accuracy.:raw-latex:`\cite{Alcorn:2016rz}`

The configured control period :math:`\Delta t` is used directly to map
the requested force :math:`F_{i}` into an on-time :math:`t_{i}`.
A loop goes over each installed thruster using the following logic.

- If off-pulsing is used then :math:`F_{i}\le 0` and we set

  .. math:: F_{i} += F_{\text{max}}

  \ to a reduced thrust to achieve the negative :math:`F_{i}` force.

- Next, if :math:`F_{i} < 0` then it set to be equal to zero. This can
  occur if an off-pulsing request is larger than the maximum thruster
  force magnitude :math:`F_{\text{max}}`.

- The nominal thruster on-time is computed using

  .. math:: t_{i}  = \dfrac{F_{i}}{F_{\text{max}}} \Delta t

- If :math:`\Delta t > t_{i} \ge t_{\text{min}}` the thruster on time is
  set to :math:`t_{i}`

- If :math:`t_{i} > \Delta t` then the thruster is saturated. In this
  case the on-time is set to :math:`t_{i} = 1.1\Delta t` such that the
  thruster remains on through the control period.

- The Schmitt trigger logic occurs if :math:`t_{i} < t_{\text{min}}`. If
  :math:`l>l_{\text{on}}` then :math:`t_{i} = t_{\text{min}}`. This
  command of :math:`t_{i} = t_{\text{min}}` remains on as long as
  :math:`l > l_{\text{off}}`. If :math:`l<l_{\text{off}}` then
  :math:`t_{i} = 0` and the thruster is turned off again for the control
  period.

- The final step is to store the thruster on-time into and write out
  this output message

Module Functions
================

- **Read in thruster configuration message**: This is used to determine
  the number of installed thrusters and what the maximum force is for
  each.

- **Convert thruster force requested into an on-time request**: Knowing
  how strong the thruster is, the on-time is scaled such that the
  effectively applied force is equal to the requested force.
