Executive Summary
-----------------
This module computes a Hill-like reference attitude for a spacecraft flying past a small body, assuming the body's
gravity does not noticeably bend the trajectory (rectilinear motion). It is the small-body counterpart of
:ref:`hillPoint`.

The first pointing profile is computed from a ground-based position and velocity given in the configuration. Every
``filterReadPeriods`` updates, the module averages the filter states it received and re-seeds the profile from that
average if it passes three checks:

- the trajectory is not a collision course (r and v not collinear),
- the predicted peak frame rate and acceleration are within the spacecraft limits,
- the position agrees with the prediction from the last accepted state.

Between re-reads, the reference is propagated analytically from the last accepted state.

Message Connection Descriptions
-------------------------------
.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - filterInMsg
      - :ref:`NavTransMsgF32Payload`
      - Filter estimate of the spacecraft position ``r_BN_N`` and velocity ``v_BN_N`` relative to the small body, read
        every update. ``timeTag`` and ``vehAccumDV`` are not used.
    * - attRefOutMsg
      - :ref:`AttRefMsgF32Payload`
      - Reference attitude, angular rate and angular acceleration. All zero only if the guidance solution is not finite
        (numerical overflow); there is no separate validity flag.
    * - flybyDiagnosticOutMsg
      - :ref:`FlybyDiagnosticMsgF32Payload`
      - Written every update. ``collinearityTrigger``, ``maxRateTrigger``, ``maxAccelerationTrigger`` and
        ``positionKnowledgeExceedTrigger`` flag the checks that rejected the averaged state, on the update that ends a
        window. ``inputSampleRejected`` flags an unusable filter state on this update. ``rejectedSamplesInWindow``
        counts the unusable states of the window ending on this update (0 otherwise).

Detailed Module Description
---------------------------

Update Cycle
............
The module has no time input: it must be updated once per control period of ``controlPeriod`` seconds, and it counts
time in whole control periods. On every update it:

1. adds the filter state, if usable, to the current averaging window;
2. at the end of a window of :math:`N` = ``filterReadPeriods`` updates, checks the window average and, if it passes,
   re-seeds the pointing profile from it, then starts a new window;
3. outputs the reference propagated from the last accepted state;
4. advances the time since that state by one control period.

The ground-based state ``groundPosition_BN_N``, ``groundVelocity_BN_N`` seeds the profile on reset and is the state at
the first update that follows. That update outputs the profile at :math:`t = 0` and opens the first window, so the
first re-read happens on update :math:`N - 1`, counting from 0.

Rectilinear Motion Model
........................
From an accepted state :math:`(\boldsymbol{r}_0, \boldsymbol{v}_0)`, with flight path angle :math:`\gamma_0` and
:math:`f_0 = v_0 / r_0`, the reference frame rotates about the orbit normal
:math:`\hat{\boldsymbol{h}} = \widehat{\boldsymbol{r}_0 \times \boldsymbol{v}_0}` by

.. math::
    \theta(t) = \arctan \left( \tan \gamma_0 + \frac{f_0}{\cos \gamma_0} t \right) - \gamma_0

.. math::
    \dot{\theta}(t) = \frac{f_0 \cos \gamma_0}{f_0^2 t^2 + 2 f_0 \sin \gamma_0 t + 1}, \qquad
    \ddot{\theta}(t) = -2 f_0^2 \cos \gamma_0 \frac{f_0t + \sin \gamma_0}{(f_0^2 t^2 + 2 f_0 \sin \gamma_0 t + 1)^2}

where :math:`t` is the time since that state. The frame at :math:`t` is the Hill frame of the accepted state rotated by
:math:`\theta` about :math:`\hat{\boldsymbol{h}}`, and the angular rate and acceleration are
:math:`\dot{\theta}\,\hat{\boldsymbol{h}}` and :math:`\ddot{\theta}\,\hat{\boldsymbol{h}}`.
``signOfOrbitNormalFrameVector = -1`` turns the frame 180 deg about its first axis.

Batch Averaging of Filter States
................................
Each usable state of the window is propagated with the rectilinear model to the window end :math:`T` and averaged.
The state received on update :math:`j = 0, \dots, N - 1` of the window is :math:`(N - 1 - j)\,\Delta t` before
:math:`T`, so over the :math:`n` usable states

.. math::
    \bar{\boldsymbol{r}}(T) = \frac{1}{n} \sum_j \left[ \boldsymbol{r}_j + (N - 1 - j)\,\Delta t \; \boldsymbol{v}_j \right],
    \qquad
    \bar{\boldsymbol{v}} = \frac{1}{n} \sum_j \boldsymbol{v}_j

A state is unusable if it is not finite, if r or v is (near) zero, or if :math:`\|\boldsymbol{r}\|\|\boldsymbol{v}\|`
or :math:`(\|\boldsymbol{v}\| / \|\boldsymbol{r}\|)^2` overflows. It is left out of the average and does not affect
the output. A window with no usable state, or whose average is unusable, makes no re-read. With :math:`N = 1` the
module re-reads every state without averaging.

Filter Solution Validity Checks
...............................
The window average :math:`(\boldsymbol{r}, \boldsymbol{v})` is rejected, and the profile kept, if any check fails:

1. **Collinearity:** :math:`1 - |\hat{\boldsymbol{r}} \cdot \hat{\boldsymbol{v}}| <` ``toleranceForCollinearity``.
2. **Peak rate and acceleration:** with the closest-approach distance
   :math:`d_{CA} = \|\boldsymbol{r} \times \boldsymbol{v}\| / \|\boldsymbol{v}\|`, the peaks of the candidate's own
   trajectory

   .. math::
       \dot{\theta}_{max} = \frac{\|\boldsymbol{v}\|}{d_{CA}}, \qquad
       |\ddot{\theta}|_{max} = \frac{3\sqrt{3}}{8} \left( \frac{\|\boldsymbol{v}\|}{d_{CA}} \right)^2

   exceed ``maximumRateThreshold`` or ``maximumAccelerationThreshold``. A collinear state (:math:`d_{CA} = 0`) fails
   both.
3. **Position knowledge:** :math:`\boldsymbol{r}` is farther than ``positionKnowledgeSigma`` from the prediction
   :math:`\boldsymbol{r}_{read} + \Delta t\,\boldsymbol{v}_{read}` from the last accepted state, :math:`\Delta t`
   after it.

Module Assumptions and Limitations
----------------------------------
- The small body's gravity is neglected. Keep ``filterReadPeriods`` short compared with the time over which the
  rectilinear model is accurate: a longer window averages out more noise but adds more model error.
- The ground-based state is trusted. It is only checked, on reset, to be usable and not collinear, otherwise the reset
  throws an invalid-argument error. The other checks apply from the first re-read on.
- ``groundPositionCovarianceDiag_N`` and ``groundVelocityCovarianceDiag_N`` are validated but not yet used.

User Guide
----------
Example configuration::

    flybyGuid = flybyPointF32.FlybyPoint()
    flybyGuid.modelTag = "flybyPoint"
    flybyGuid.controlPeriod = 10.0      # [s] must match the task rate
    flybyGuid.filterReadPeriods = 6     # re-read the filter every 6 control periods (60 s)
    flybyGuid.toleranceForCollinearity = 1e-5
    flybyGuid.signOfOrbitNormalFrameVector = 1
    flybyGuid.maximumRateThreshold = 0.01
    flybyGuid.maximumAccelerationThreshold = 1e-7
    flybyGuid.positionKnowledgeSigma = 1e5
    flybyGuid.groundPosition_BN_N = [-5e7, 7.5e6, 5e5]          # [m]
    flybyGuid.groundVelocity_BN_N = [2e4, 0.0, 0.0]             # [m/s]
    flybyGuid.groundPositionCovarianceDiag_N = [1e8, 1e8, 1e8]  # [m^2]
    flybyGuid.groundVelocityCovarianceDiag_N = [1.0, 1.0, 1.0]  # [(m/s)^2]
    unitTestSim.AddModelToTask(unitTaskName, flybyGuid)

.. list-table:: Module Parameters
   :widths: 30 15 55
   :header-rows: 1

   * - Parameter
     - Default
     - Description
   * - ``controlPeriod``
     - 0
     - [s] time between updates; must match the task rate. Finite, > 0
   * - ``filterReadPeriods``
     - 1
     - [-] updates per averaging window, i.e. between re-reads. >= 1
   * - ``toleranceForCollinearity``
     - 0
     - [-] collinearity tolerance on :math:`1 - |\hat{r} \cdot \hat{v}|`. > 0
   * - ``signOfOrbitNormalFrameVector``
     - 1
     - +1 for a Hill frame, -1 to flip it 180 deg about its first axis
   * - ``maximumRateThreshold``
     - 0
     - [deg/s] maximum predicted peak frame rate. > 0
   * - ``maximumAccelerationThreshold``
     - 0
     - [deg/s^2] maximum predicted peak frame acceleration. > 0
   * - ``positionKnowledgeSigma``
     - 0
     - [m] maximum distance from the predicted position. > 0
   * - ``groundPosition_BN_N``
     - [0, 0, 0]
     - [m] ground-based position at the first update after reset, N frame. Required; usable and not collinear with
       the velocity
   * - ``groundVelocity_BN_N``
     - [0, 0, 0]
     - [m/s] ground-based velocity at the first update after reset, N frame. Required
   * - ``groundPositionCovarianceDiag_N``
     - [0, 0, 0]
     - [m^2] position variances of the ground-based covariance, N frame. Finite, >= 0. Not yet used
   * - ``groundVelocityCovarianceDiag_N``
     - [0, 0, 0]
     - [(m/s)^2] velocity variances of the ground-based covariance, N frame. Finite, >= 0. Not yet used
