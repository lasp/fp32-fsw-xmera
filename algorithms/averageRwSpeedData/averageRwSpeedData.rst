Executive Summary
-----------------
This module calculates the mean of the reaction wheel (RW) speed samples in a time window. It keeps the samples in
a ring of constant size and adds one sample each time that the input message changes. The output speed of each wheel is the
mean speed of that wheel over the samples in the window. The module sends the wheel angles from the input to
the output without a change.

The end of the time window is the newest sample in the ring, and not the simulation time. Thus, the output stays
the same while the module receives no new sample.

All calculations use single-precision floating point (``float`` / fp32). The module has one algorithm class
(``AverageRwSpeedDataAlgorithm``) and two adapters. The ``SysModel`` adapter connects the algorithm to the Xmera
system with messages. The C shim connects the algorithm to the Adamant system with the C/Ada FFI.

Module Architecture
-------------------
The algorithm (``AverageRwSpeedDataAlgorithm``) has no framework dependencies. It holds the ring and the
``AverageRwSpeedDataConfig`` object, which contains the averaging window. ``update()`` receives one
``RwSpeedSample``, which contains a measurement time and one speed for each wheel. ``update()`` adds the sample to
the ring and gives the mean wheel speeds. ``setConfig()`` replaces the configuration and keeps the ring.
``reInitialize()`` erases the ring.

The Xmera adapter (``AverageRwSpeedData``) uses ``SysModel`` as its base class and does all message operations.
The configuration parameter is a public member variable (two-phase initialization). ``reset()`` makes sure that the
input message is connected, makes the configuration, and makes a new algorithm with an empty ring.
``reconfigure()`` sends the parameter value to the algorithm again. ``reInitialize()`` erases the ring.

``updateState()`` reads the input message only if the time at which it was written changed after the previous
call. That time becomes the measurement time of the sample, because the message has no measurement time of its
own. If the message did not change, ``updateState()`` does not call the algorithm and does not write the output
message.

The Adamant adapter is a C shim (``averageRwSpeedDataAlgorithm_c.h`` / ``.cpp``). It makes the algorithm available
through an ``extern "C"`` interface for the C/Ada FFI bindings. ``validateConfig()`` tells the caller if a
configuration is correct, and it does not throw. Thus, Ada can examine a configuration before it calls the
``create()`` or ``setConfig()`` functions, which throw an exception for an incorrect configuration.

Message Connection Descriptions
-------------------------------
The table that follows gives the module input and output messages. The user sets the message variable name from
Python. The message type contains a link to the message structure definition.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - rwSpeedInMsg
      - :ref:`RWSpeedMsgF32Payload`
      - Input message with the speed [rad] and the angle [rad] of each wheel. This message must be connected.
    * - rwSpeedOutMsg
      - :ref:`RWSpeedMsgF32Payload`
      - Output message. ``wheelSpeeds`` contains the mean speed of each wheel [rad/s]. ``wheelThetas`` contains the
        wheel angles from the newest input message [rad], without a change.

Module Parameters
-----------------
``reset()`` makes sure that the configuration parameter is correct when it makes the algorithm configuration.
An incorrect value causes an ``fsw::invalid_argument`` exception.

.. list-table:: Module Parameters
    :widths: 25 15 20 40
    :header-rows: 1

    * - Parameter
      - Default
      - Valid Range
      - Description
    * - ``rwSpeedAveragingWindow``
      - 0.0
      - :math:`(0, W_\text{max}]`
      - Length [s] of the time window for the mean. :math:`W_\text{max}` is ``kMaxAveragingWindowSec``.

The default value of ``rwSpeedAveragingWindow`` is zero, and the module rejects a window of zero. Thus, a caller
must always set this parameter.

The constants that follow are set at compile time:

.. list-table:: Compile-Time Constants
    :widths: 30 15 55
    :header-rows: 1

    * - Constant
      - Value
      - Description
    * - ``kRwSpeedSampleRateHz``
      - 5.0 [Hz]
      - Usual sample rate :math:`f` of the RW speeds.
    * - ``kMaxAveragingWindowSec``
      - 2.0 [s]
      - Largest permitted averaging window :math:`W_\text{max}`.
    * - ``kRingCapacity``
      - 11
      - Number of slots :math:`N` in the ring. The module calculates it from :math:`f` and :math:`W_\text{max}`.

Mathematical Formulation
------------------------

Ring size
^^^^^^^^^
The ring must hold all the samples that a window of length :math:`W_\text{max}` can contain at the sample rate
:math:`f`. A window contains the samples at both of its ends. Thus, the number of slots is

.. math::

    N = \lceil f \, W_\text{max} \rceil + 1

For :math:`f = 5` Hz and :math:`W_\text{max} = 2` s, the window contains the samples at 0, 0.2, ..., 2.0 s. Thus,
:math:`N = 11`. If the product is not a full number, the module increases it to the next full number. Thus, the ring also
holds a window with a length that is not a full number of sample periods.

Addition of a sample
^^^^^^^^^^^^^^^^^^^^
:math:`t_\text{max}` is the highest measurement time in the ring before ``update()`` adds the new sample.
:math:`t` is the measurement time of the new sample. The module ignores the new sample if one of these conditions
is true:

- :math:`t = 0`. A measurement time of zero shows that the sample has no data.
- :math:`t < t_\text{max}` and :math:`t_\text{max} - t > W_\text{max}`. The new sample must have been written within the
  defined window size.

If not, the module writes the sample into the next slot of the ring. When the ring is full, the new sample
replaces the sample that the module wrote first. The module can receive a sample out of order. The module adds
such a sample if :math:`t_\text{max} - t` is not more than :math:`W_\text{max}`.

Calculation of the mean
^^^^^^^^^^^^^^^^^^^^^^^
After the addition, :math:`t_\text{max}` is the highest measurement time in the ring, and :math:`W` is
``rwSpeedAveragingWindow``. The set :math:`S` contains each written slot :math:`i` with

.. math::

    t_\text{max} - t_i \le W

The module ignores a slot that it did not write. Such a slot has a measurement time of zero. For each wheel
:math:`k`, the output speed is the mean of the speeds in :math:`S`, with the same weight for each sample:

.. math::

    \bar{\Omega}_k = \frac{1}{|S|} \sum_{i \in S} \Omega_{i,k}

If :math:`S` is empty, the output speed of each wheel is zero. This occurs only when the ring is empty.

``setConfig()`` changes :math:`W` but keeps the ring. Thus, a larger window can include samples that a smaller
window did not include before. A window that is smaller than :math:`W_\text{max}` keeps the samples in the ring that
are outside the window. The next mean ignores them.

User Guide
----------
The module uses two-phase initialization. Do the steps that follow:

1. Set the public configuration parameter.
2. Connect the input message.
3. Add the module to the simulation task. ``reset()`` then makes the configuration.

::

    rwSpeedAverage = averageRwSpeedDataF32.AverageRwSpeedData()
    rwSpeedAverage.modelTag = "rwSpeedAverage"
    rwSpeedAverage.rwSpeedAveragingWindow = 1.0

    rwSpeedAverage.rwSpeedInMsg.subscribeTo(rwStateEffector.rwSpeedOutMsg)

    scSim.AddModelToTask(simTaskName, rwSpeedAverage)

To change the window during the simulation, set ``rwSpeedAveragingWindow`` and then call ``reconfigure()``. To
erase the ring, call ``reInitialize()``.

A larger window gives a smoother output. But, when the wheel speeds change, the output then changes more slowly.

Module Assumptions and Limitations
----------------------------------
**Assumption.** The measurement time of a sample is correct for all the wheels in that sample. The module cannot
examine the time at which each wheel was measured.

**Assumption.** The caller does not send the same sample two times. The module does not identify a sample that it
received before, thus that sample gets two times the weight. The Xmera adapter prevents this, because it
reads a message only when the message changes.

**Limitation.** The Xmera adapter uses the time at which the input message was written as the measurement time.
If the sender writes the message after the measurement, the module uses an incorrect time.

**Limitation.** The ring holds a full window only at the sample rate :math:`f` or less. At a higher sample rate,
new samples replace samples that are in the window. The mean then uses a shorter window than :math:`W`.

**Limitation.** A sample with an incorrect measurement time in the future sets :math:`t_\text{max}`. The module
then ignores each correct sample with a measurement time :math:`t` for which :math:`t_\text{max} - t` is more than
:math:`W_\text{max}`. ``reInitialize()``
erases the ring and stops this condition.
