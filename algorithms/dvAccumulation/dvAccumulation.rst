.. raw:: latex

    {\LARGE \textbf{dvAccumulation}}

Executive Summary
-----------------
``dvAccumulation`` integrates one sample of the body-frame acceleration into a Delta-V accumulator.
Each ``updateState()`` reads one ``IMUSensorBodyMsgF32Payload``. It uses the ``AccelBody`` field as
the body-frame acceleration :math:`\ddot{\mathbf{r}}_{B}` that gravity does not cause. The module
subtracts the accelerometer bias :math:`\mathbf{b}` that the caller supplies. Then it integrates the remainder
during the configured control period :math:`\Delta t`. Thus each sample gives
:math:`\Delta\mathbf{v} = \Delta\mathbf{v} + \Delta t \, (\ddot{\mathbf{r}}_{B} - \mathbf{b})`. The
module writes the accumulator to the output message. It tags the message with the module call time.

Message Connection Descriptions
-------------------------------
.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Variable Name
      - Type
      - Description
    * - imuInMsg
      - ``IMUSensorBodyMsgF32Payload``
      - Input IMU body message. The module reads only ``AccelBody`` (m/s^2, float[3], the body-frame
        acceleration that gravity does not cause).
    * - dvAccumulationOutMsg
      - ``NavTransMsgF32Payload``
      - Output navigation message. The adapter writes ``timeTag`` (seconds, double, the module call
        time) and ``vehAccumDV`` (m/s, float[3]). The position and velocity fields stay zero.

Module Parameters
-----------------
.. list-table:: Module Parameters
    :widths: 30 15 10 10 40 30
    :header-rows: 1

    * - Parameter Name
      - Type
      - Units
      - Default
      - Description
      - Bounds
    * - controlPeriod (necessary)
      - float
      - [s]
      - 0
      - Control period. The algorithm uses it as the integration step. It is the time between two
        ``updateState()`` calls, or 1/fsw_rate.
      - Must be finite and more than zero
    * - accelBias_B
      - Eigen::Vector3f
      - [m/s^2]
      - zeros
      - Accelerometer bias that the measured body-frame acceleration contains. The algorithm
        subtracts it from each sample. A value of zero makes no correction. The adapter gives it to
        the algorithm on each ``updateState()``, thus an edit takes effect on the next call.
      - None. The algorithm does not validate the bias

``DvAccumulationConfig::create`` applies these limits at ``reset()``. If a parameter is not in its
limits, ``create`` throws ``fsw::invalid_argument``.

Module Assumptions and Limitations
----------------------------------
- ``accelBias_B`` is the offset that **the measurement contains**. The algorithm subtracts it from
  each sample. If you have a *correction* and not a bias, change the sign of the value before you set
  it. If you do not change the sign, the error doubles and does not cancel.
- ``accelBias_B`` is a constant offset only. An accelerometer that is not at the center of mass also
  measures terms that change with the rate:
  :math:`\boldsymbol{\omega} \times (\boldsymbol{\omega} \times \mathbf{r}) +
  \dot{\boldsymbol{\omega}} \times \mathbf{r}`. No constant vector can remove these terms. A bias
  that includes them is correct only at the rate of the fit. Lever-arm compensation is out of scope.
- The bias is one body-frame vector. The module applies it after ``averageMimuData`` makes the
  average of the devices. Thus it is the total of the biases of the devices. If the set of devices
  changes (for example, a MIMU dropout), the total changes. Then the configured value is not correct.
- ``accelBias_B`` is an argument to ``update()`` and not a configuration parameter. Thus the caller
  owns the bias, and the algorithm keeps no calibration state. The algorithm does not validate the
  bias. A bias integrates, thus a unit error or a g-level value causes an error in a full burn, and a
  bias that is not finite gives a Delta-V that is not finite with no exception.
- The algorithm has no time input. It integrates during the configured ``controlPeriod`` on each
  call. Thus the caller must use it one time in each control period. ``dvAccumulation`` operates at
  the same rate as its upstream producer, and immediately after it. Thus the acceleration sample is
  new on each call.
- The first ``update()`` after construction or after ``reInitialize()`` **starts the accumulation
  window**. That first ``update()`` does not integrate. N samples have N-1 intervals, thus this
  interval count is correct and no sample is lost. The accumulated Delta-V is equal to the integral
  of the acceleration during the elapsed time after that first call. This causes no bias.
- ``reInitialize()`` sets the accumulator to zero and starts a new accumulation window. It must do
  the two actions together. If it sets the accumulator to zero but keeps the window open, it
  integrates a full step into a new window. Then the accumulated Delta-V is one interval more than
  the elapsed time. These two values are all of the runtime state of the algorithm. Thus there is no
  entry point for a partial reset.
- Lifecycle: the adapter constructs the algorithm in ``reset()`` at startup only. State-transition
  hooks use ``reInitialize()``. A state transition does not use ``reset()`` again. ``reconfigure()``
  installs the edited parameters and does not start a new accumulation window.
- The accumulator is float precision (``Eigen::Vector3f``). ``controlPeriod`` is also float
  precision. Thus all of the integration is single precision. ``timeTag`` stays double in the output
  message.

Accumulation precision
~~~~~~~~~~~~~~~~~~~~~~

Quadrature, and not float32, is the real limit on accuracy. The rectangle rule holds
:math:`\ddot{\mathbf{r}}_{B}` constant during each control period. Thus an acceleration that changes
gives an :math:`O(\Delta t \, \Delta\ddot{r})` residual in each step. This residual is larger than
the limit on rounding that follows.

The accumulator is single precision. Look at an example of a small acceleration of 0.005 m/s^2 during
a maximum burn of 1.5 hours. The ``controlPeriod`` is 0.2 s, thus there are 27,000 updates and the
accumulator gets to 27 m/s. Each ``+=`` rounds to the nearest value that ``float`` can hold. Thus the
relative limit on the accumulated error is :math:`N \cdot 2^{-25}` for ``float`` and
:math:`N \cdot 2^{-54}` for ``double``:

.. list-table:: Accumulation drift, example 1.5 hour burn at 0.005 m/s^2
    :widths: 35 25 25
    :header-rows: 1

    * - Accumulator type
      - Worst-case drift
      - Fraction of 27 m/s
    * - ``float`` (as built)
      - 2.2e-2 m/s
      - 0.080%
    * - ``double`` (reference only)
      - 4.0e-11 m/s
      - 1.5e-10%

Single precision is satisfactory and has margin. A ``double`` accumulator or a compensated summation
is not necessary.

Range and saturation
~~~~~~~~~~~~~~~~~~~~

Look at an example acceleration of 0.5 m/s^2. The ``controlPeriod`` is 0.2 s, thus the increment in
each update is :math:`\delta = \Delta t \, a = 0.1` m/s. The accumulator has two limits:

.. list-table:: Accumulator range limits, example acceleration 0.5 m/s^2
    :widths: 40 20 35
    :header-rows: 1

    * - Limit
      - Value
      - Time at 0.5 m/s^2
    * - Stagnation (the accumulator stops the increments)
      - 2.1e6 m/s
      - ~49 days of continuous thrust
    * - IEEE ``float`` overflow to infinity
      - 3.4e38 m/s
      - ~2e31 years

Stagnation is the first limit. The accumulator does not add an increment when
:math:`\delta < \mathrm{ULP}(M)/2`, which is at more than :math:`M \approx \delta \cdot 2^{24}`
(here :math:`2^{21}` m/s). After that point the accumulator stops. There is no trap and no infinity.
The module cannot get to either limit in operation.

Module Architecture
-------------------
The module has three layers:

- **Adapter (``dvAccumulation.h/.cpp``, ``class DvAccumulation : SysModel``).** It reads the input
  message. It converts ``AccelBody`` to an ``Eigen::Vector3f`` with
  ``utilities/fsw/eigenSupport.h``. Then it uses the algorithm with the current ``callTime`` and
  writes the output message. The adapter holds the algorithm in a ``std::unique_ptr``. It constructs
  the algorithm in ``reset()``, after it makes sure that ``imuInMsg`` is linked.
- **Algorithm (``dvAccumulationAlgorithm.h/.cpp``, ``class DvAccumulationAlgorithm``).** Pure
  algorithm — no SysModel, no messages, no time. ``update(rDDotNoGravity_BN_B, accelBias_B)`` gets
  two ``Eigen::Vector3f``. It subtracts the ``accelBias_B``, then it integrates during the
  ``controlPeriod``. Its validated ``DvAccumulationConfig`` holds the ``controlPeriod``. ``update``
  gives the accumulated ``vehAccumDV_B`` (``Eigen::Vector3f``, m/s). The adapter puts the time tag on
  the output message.
- **C shim (``dvAccumulationAlgorithm_c.h/.cpp``).** Pure-C interface for Ada FFI: an opaque handle
  and ``DvAccumulationAlgorithm_create``/``_destroy``/``_validateConfig``/``_setConfig``/
  ``_reInitialize``/``_update``. ``_update`` gets a ``Vector3f_c`` acceleration and a ``Vector3f_c``
  bias, and gives a ``Vector3f_c`` Delta-V. It uses the shared ``Vector3f_c`` from
  ``utilities/fsw/plainCAlgorithmDataTypes.h``.

Algorithm Layer
---------------
The configured control period is :math:`\Delta t` (``controlPeriod``). Each ``update()`` call gets
two arguments: the bias :math:`\mathbf{b}` (``accelBias_B``) and the body-frame acceleration
``rDDotNoGravity_BN_B``.

1. On the first ``update()`` after construction or after ``reInitialize()``, start the accumulation
   window. Do not integrate: the elapsed time is zero.
2. On each later call, subtract the bias and integrate one control period:

   .. math::

      \Delta\mathbf{v} = \Delta\mathbf{v} + \Delta t \, (\ddot{\mathbf{r}}_{B} - \mathbf{b})

3. Give ``vehAccumDV_B`` = :math:`\Delta\mathbf{v}`. The adapter tags the output message with
   ``timeTag = callTime * kNano2Sec``.

Thus after :math:`N` calls the accumulator holds the integral of the bias-corrected acceleration
during :math:`(N-1)\,\Delta t`. This is the elapsed time after the window started.

User Guide
----------
The necessary module configuration is::

    module = dvAccumulationF32.DvAccumulation()
    module.modelTag = "dvAccumulation"
    module.controlPeriod = 0.2      # [s] integration step, necessary (> 0), must be the task rate
    module.accelBias_B = [0., 0., 0.]  # [m/s^2] measured bias, subtracted per sample, optional
    module.imuInMsg.subscribeTo(mimuMajorityVote.imuSensorBodyOutMsg)
    # Subscribe a downstream consumer (for example, dvExecuteGuidance) to module.dvAccumulationOutMsg.

Set ``controlPeriod`` to the rate of the adapter before ``reset()``. Use ``reset(callTime)`` one time
before the first ``updateState(callTime)``. ``reset`` throws ``std::invalid_argument`` if
``imuInMsg`` is not linked. It throws ``fsw::invalid_argument`` if ``controlPeriod`` is not more than
zero. If you edit ``controlPeriod`` after ``reset()``, the next ``reconfigure()`` installs it.
``reconfigure()`` does not start a new accumulation window. ``accelBias_B`` needs no
``reconfigure()``: the adapter reads the property on each ``updateState()``. On a state transition the
flight software uses ``reInitialize()`` and not ``reset()``.
