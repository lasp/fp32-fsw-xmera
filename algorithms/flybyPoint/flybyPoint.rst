Executive Summary
-----------------
This module computes a reference attitude frame for a spacecraft in relative motion about a small body. The implicit assumption is that the small body's mass does not perturb the motion of the spacecraft significantly. Conceptually, this module is equivalent to :ref:`hillPoint`, but for the relative motion of a spacecraft about a body that is not the main center of gravity.

The module starts by reading the first usable, non-collinear input under the assumption it is valid in order to compute
a solution.
At a settable cadence, the module will update the pointing profile with the help of a new filter solution. That
solution is the average of all the filter states received since the previous update, each propagated to the update
time, which low-pass filters the noisy filter output. Before using it, the module checks the validity of the solution: 1. It does not predict a collision trajectory 2. It does not predict
excessive rates and accelerations 3. Its position agrees with the rectilinear prediction made from the first read. If
the solution is valid a new pointing profile is constructed.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages. The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - filterInMsg
      - :ref:`NavTransMsgF32Payload`
      - Input message containing the relative position and velocity of the spacecraft with respect to the small body, estimated from a filter. Read at every update.
    * - attRefOutMsg
      - :ref:`AttRefMsgF32Payload`
      - Output attitude reference message containing reference attitude, reference angular rates and accelerations. As in the other guidance modules there is no validity flag: the reference is all zero until the first usable,
        non-collinear filter state seeds the profile, and zero for any update whose guidance solution is not finite
        (numerical overflow). Otherwise it is the reference propagated from the last accepted update.
    * - flybyDiagnosticOutMsg
      - :ref:`FlybyDiagnosticMsgF32Payload`
      - Output diagnostic message, written at every update. ``collinearityTrigger``, ``maxRateTrigger``, ``maxAccelerationTrigger`` and ``positionKnowledgeExceedTrigger`` report which validity checks rejected the averaged filter state, on the update that ends an averaging window (false otherwise). ``inputSampleRejected`` is true when the filter state of this update was unusable and left out of the average. ``rejectedSamplesInWindow`` is the number of unusable filter states in the averaging window that ends on this update, and 0 on other updates.


Detailed Module Description
---------------------------
The relative position and velocity vector of the spacecraft with respect to the small body are obtained as noisy estimates. Therefore the desire is, for this module, to only update the pointing profile every so often, and to update it from an average of the filter states rather than from a single one (see `Batch Averaging of Filter States`_). The input parameter ``filterReadPeriods`` allows the user to specify the number of control periods between two subsequent updates, which is also the length of the averaging window. For every call of this module that happens between two consecutive updates, the reference attitude needs to be propagated from the last accepted update according to a dynamic model of the flyby.

The algorithm has no time input: it must be updated once per control period, and ``controlPeriod`` gives the length of that period in seconds. The algorithm counts elapsed time in whole control periods with unsigned integers, so the decision of when to re-read the filter is exact and free of floating-point rounding. Elapsed time in seconds, which the guidance equations below need, is computed as the number of elapsed control periods times ``controlPeriod``. If a re-read is rejected by the validity checks, the reference keeps being propagated from the last accepted read and the next re-read is attempted at the end of the next averaging window.

Rectilinear Motion Model
........................
In this case the flyby is modeled as rectilinear motion of the spacecraft, i.e., the spacecraft moves with a constant velocity. At every filter read, the relative position and velocity vectors :math:`\boldsymbol{r}_0` and :math:`\boldsymbol{v}_0` of the spacecraft with respect to the small body are provided. The following coefficients are defined: the flight path angle :math:`\gamma_0` of the spacecraft, and the ratio between velocity and radius magnitudes :math:`f_0 = \frac{v_0}{r_0}`. From these quantities, the rotation of the reference frame is given by the following equations:

.. math::
    \theta(t) = \arctan \left( \tan \gamma_0 + \frac{f_0}{\cos \gamma_0} t \right) - \gamma_0
.. math::
    \dot{\theta}(t) = \frac{f_0 \cos \gamma_0}{f_0^2 t^2 + 2 f_0 \sin \gamma_0 t + 1}
.. math::
    \ddot{\theta}(t) = -2 f_0^2 \cos \gamma_0 \frac{f_0t + \sin \gamma_0}{(f_0^2 t^2 + 2 f_0 \sin \gamma_0 t + 1)^2}

where :math:`t` is the time passed since the last accepted filter read. Note that using the flight path angle :math:`gamma_0` makes these equation always nonsingular. :math:`\theta(t)` is used to compute the additional frame rotation from the Hill frame computed at the read time. Such rotation happens about the angular momentum direction vector. :math:`\dot{\theta}(t)` and :math:`\ddot{\theta}(t)` projected onto the angular momentum direction vector give the angular rate and acceleration vectors of the reference frame.


Batch Averaging of Filter States
................................
The module receives a filter state at every control period, but only updates the pointing profile at the end of each
window of :math:`N` = ``filterReadPeriods`` control periods. Rather than using only the state received at the window
end, the module low-pass filters the filter output: each state received during the window is propagated to the window
end time :math:`T` with the rectilinear model, and the propagated states are averaged. The state received at control
period :math:`j = 1, \dots, N` of the window arrives :math:`(N - j)\,\Delta t` before :math:`T`, where :math:`\Delta t`
is ``controlPeriod``, so over the :math:`n \le N` usable states of the window

.. math::
    \bar{\boldsymbol{r}}(T) = \frac{1}{n} \sum_j \left[ \boldsymbol{r}_j + (N - j)\,\Delta t \; \boldsymbol{v}_j \right],
    \qquad
    \bar{\boldsymbol{v}} = \frac{1}{n} \sum_j \boldsymbol{v}_j

The velocity is averaged without propagation, since the rectilinear model holds it constant. The propagation time is
a whole number of control periods, so it carries no time rounding error. Because every state is aligned to :math:`T`
before averaging, the average has no time lag under the rectilinear model. The average
:math:`(\bar{\boldsymbol{r}}, \bar{\boldsymbol{v}})` is the candidate that goes through the validity checks below and,
if it passes, re-seeds the pointing profile.

- A state that is not finite, whose position or velocity is (near) zero, or so large that the products the module
  forms from it, :math:`\|\boldsymbol{r}\|\|\boldsymbol{v}\|` and :math:`(\|\boldsymbol{v}\| / \|\boldsymbol{r}\|)^2`,
  overflow, is unusable. It is left out of the
  average and flagged with ``inputSampleRejected`` for that control period. The attitude reference of that period is
  unaffected and stays valid, because it is propagated from the last accepted update and does not depend on the
  current state. The window still ends on time, and on its last control period ``rejectedSamplesInWindow`` gives the
  number of unusable states it received. A count equal to :math:`N` means the window had no usable state, so no
  re-read was attempted.
- Before the first seed there is no profile, so the reference is all zero. Apart from a guidance solution that is
  not finite (see `Zero Reference Fallback`_), this is the only case in which the reference is zero.
- Usable states can still average to an unusable state, for example velocities that cancel. Such a window makes no
  re-read attempt, and ``rejectedSamplesInWindow`` does not count it.
- Every window end starts a new window, whether the average was accepted, rejected, or no usable state was received.
  A rejected average is therefore retried after another full window, while the profile keeps being propagated from the
  last accepted update.
- The first usable, non-collinear state after a reset seeds the profile immediately, without averaging, and the first
  window starts after it. Changing the configuration discards the window in progress.
- With :math:`N = 1` the average is the latest state itself, so the module behaves as without averaging.

Averaging reduces the effect of noise on the update. For uncorrelated noise the reduction is about
:math:`1/\sqrt{n}`; it is smaller for a filter output that is already smoothed and therefore correlated from one
control period to the next. A longer window averages more states, but the small body's gravity, which the rectilinear
model neglects, makes the propagated states deviate from the true state at :math:`T` by more, so the window should be
kept short compared with the time over which that model is accurate.


Filter Solution Validity Checks
...............................
Before a new filter solution :math:`(\boldsymbol{r}, \boldsymbol{v})` replaces the current pointing profile, it is
checked as follows. Any failed check rejects the solution and raises the matching diagnostic flag; the profile then
keeps being propagated from the last accepted read.

1. **Collinearity.** If :math:`1 - |\hat{\boldsymbol{r}} \cdot \hat{\boldsymbol{v}}|` is below
   ``toleranceForCollinearity``, the trajectory is a collision course. The checks are independent, so such a solution
   usually also raises the rate and acceleration flags, because its closest-approach distance is close to zero. An
   exactly collinear solution (:math:`d_{CA} = 0`) has no closest-approach distance, so its rate and
   acceleration checks reject it outright.
2. **Predicted peak rate and acceleration.** On the candidate's own rectilinear trajectory, the frame rate and
   acceleration peak near closest approach, at the distance

   .. math::
       d_{CA} = \frac{\|\boldsymbol{r} \times \boldsymbol{v}\|}{\|\boldsymbol{v}\|} = \|\boldsymbol{r}\| \, |\cos \gamma|

   Minimizing the denominator of :math:`\dot{\theta}(t)` above gives the peak rate, and maximizing
   :math:`|\ddot{\theta}(t)|` gives the peak acceleration (reached 30 deg before and after closest approach):

   .. math::
       \dot{\theta}_{max} = \frac{\|\boldsymbol{v}\|}{d_{CA}}, \qquad
       |\ddot{\theta}|_{max} = \frac{3\sqrt{3}}{8} \left( \frac{\|\boldsymbol{v}\|}{d_{CA}} \right)^2

   These are compared with ``maximumRateThreshold`` and ``maximumAccelerationThreshold``. The check is applied whether
   the spacecraft is approaching or receding. Past closest approach, these values are upper bounds on the remaining
   profile.
3. **Position knowledge.** The position must lie within ``positionKnowledgeSigma`` of the rectilinear prediction
   :math:`\boldsymbol{r}_{first} + \Delta t \, \boldsymbol{v}_{first}` made from the first read.


Zero Reference Fallback
.......................
Like the other guidance modules, the module outputs no validity flag. When no guidance solution is available, the
reference attitude, rate and acceleration are all output as zero:

- **Before the first seed.** Filter states that are unusable, or collinear (a collision course, for which
  :math:`\hat{\boldsymbol{r}} \times \hat{\boldsymbol{v}}` defines no orbit normal), do not seed the profile. A
  collinear state raises ``collinearityTrigger``. A state with
  :math:`\|\hat{\boldsymbol{r}} \times \hat{\boldsymbol{v}}\| < 10^{-12}` counts as collinear whatever
  ``toleranceForCollinearity`` is, since its orbit-normal direction would be dominated by rounding.
- **Non-finite solution.** If the propagated profile is not finite, for example because :math:`f_0^2` in
  :math:`\ddot{\theta}` overflows single precision, the whole reference is zero for that update. The fallback is
  checked again at every update, so it does not latch: the next update with a finite solution is output normally.


Clohessy-Wiltshire Equations Model
..................................
T.B.D.


Module Assumptions and Limitations
----------------------------------
The limitations of this module are inherent to the geometry of the problem, which determines whether or not all the constraints can be satisfied. For example, as shown in  in R. Calaon, C. Allard and H. Schaub, "Attitude Reference Generation for Spacecraft with Rotating Solar Arrays and Pointing Constraints," In preparation for Journal of Spacecraft and Rockets, depending on the relative orientation of :math:`{}^\mathcal{B}h` and :math:`{}^\mathcal{B}a_1`, it may not be possible to  achieve perfect incidence angle on the solar arrays. Only when perfect incidence is obtained, it is possible to solve for the solution that also drives the body-fixed direction :math:`{}^\mathcal{B}a_2` close to the Sun. When perfect incidence is achievable, two solutions exist. If :math:`{}^\mathcal{B}a_2` is provided as input, this is used to determine which solution to pick. If this input is not provided, one of the two solution is chosen arbitrarily.

Due to the difficulty in developing an analytical formulation for the reference angular rate and angular acceleration vectors, these are computed via second-order finite differences. At every time step, the current reference attitude and time stamp are stored in a module variable and used in the following time updates to compute angular rates and accelerations via finite differences.

Algorithmically, there is an assumption that the first solution is somewhat trustworthy as it seeds the algorithm.
It will get overwritten by new measurements if they are valid. Because the algorithm needs a seed, the seed is only
checked for collinearity, which would leave the flyby frame undefined; the rate, acceleration and position-knowledge
checks apply from the first re-read on.

User Guide
----------
The required module configuration is::

    flybyGuid = flybyPointF32.FlybyPoint()
    flybyGuid.modelTag = "flybyPoint"
    flybyGuid.controlPeriod = 10.0      # [s] must match the task rate
    flybyGuid.filterReadPeriods = 6     # re-read the filter every 6 control periods (60 s)
    flybyGuid.toleranceForCollinearity = 1e-5
    flybyGuid.signOfOrbitNormalFrameVector = 1
    flybyGuid.maximumRateThreshold = 0.01
    flybyGuid.maximumAccelerationThreshold = 1e-7
    flybyGuid.positionKnowledgeSigma = 1e5
    unitTestSim.AddModelToTask(unitTaskName, flybyGuid)

The module is configurable with the following parameters:

.. list-table:: Module Parameters
   :widths: 25 25 50
   :header-rows: 1

   * - Parameter
     - Default
     - Description
   * - ``controlPeriod``
     - 0
     - [s] time between two consecutive module updates. Must match the task rate and be finite and greater than zero
   * - ``filterReadPeriods``
     - 1
     - [-] number of control periods between two consecutive filter reads, which is also the length of the averaging window. Must be at least 1; 1 re-reads the latest filter state at every update, without averaging
   * - ``toleranceForCollinearity``
     - 0
     - [-] tolerance on :math:`1 - |\hat{r} \cdot \hat{v}|` below which a filter solution is rejected as collinear (collision trajectory). Must be greater than zero
   * - ``signOfOrbitNormalFrameVector``
     - 1
     - Sign of the orbit normal rxv vector used to build the frame. If equal to 1, the frame is a traditional Hill frame if -1, it flips the orbit normal axis to point "down" relative to the orbtial momentum
   * - ``maximumRateThreshold``
     - 0
     - [deg/s] maximum allowable predicted rate at closest approach. If greater, the filter solution is discarded. Must be greater than zero
   * - ``maximumAccelerationThreshold``
     - 0
     - [deg/s^2] maximum allowable predicted acceleration at closest approach. If greater, the filter solution is discarded. Must be greater than zero
   * - ``positionKnowledgeSigma``
     - 0
     - [m] maximum allowable deviation of the filter position from the rectilinear prediction made from the first read. If greater, the filter solution is discarded. Must be greater than zero

Unit Test
---------
This unit test script tests the correctness of the reference attitude computed by :ref:`flybyPoint` in a scenario where the rectilinear flyby assumption is valid.

In this test, there is no gravity body, and the spacecraft is put onto a rectilinear trajectory about the origin.
With no gravity, the linear momentum of the spacecraft does not change, which means that the spacecraft proceeds along a rectilinear trajectory. The input message to the :ref:`flybyPoint` is the relative position and velocity of the spacecraft with respect to the body/asteroid, which coincides with the origin and is assumed to be static.
Correctness is tested assessing whether the computed hill frame moves according to the motion of the spacecraft.

The reference attitude :math:`\sigma_\mathcal{R/N}`, reference angular rates :math:`\omega_\mathcal{R/N}` and angular accelerations :math:`\dot{\omega}_\mathcal{R/N}` are tested. These are compared to the analytical results expected from the rectilinear motion described in the documentation of :ref:`flybyPoint`.
The reference attitude is mapped to the corresponding reference frame, and each axis of the reference frame is tested for correctness. The angular rate and acceleration vectors are tested against the analytical result, expressed in R-frame coordinates.
