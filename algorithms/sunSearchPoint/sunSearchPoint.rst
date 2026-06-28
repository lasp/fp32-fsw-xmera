.. raw:: latex

    {\LARGE \textbf{sunSearchPoint}}

Executive Summary
-----------------
This module drives the spacecraft's safe-mode startup in two phases: a scripted **sun search**
followed by closed-loop **sun pointing**. On reset it begins a sequence of constant-rate body
rotations that sweep the body-fixed coarse sun sensors (CSS) through different orientations. Once
the sun is acquired — signalled by the CSS observation count reaching a configurable threshold —
the module transitions to sun pointing, producing attitude tracking errors as Modified Rodrigues
Parameters (MRPs) and body-rate tracking errors that align a commanded body axis with the sun
heading. An optional constant spin rate about the sun heading axis can be specified. If the sun
direction is not available during pointing, the attitude error is zeroed and a configurable
body-fixed fallback rate is commanded instead.

The phase transition is **one-way**:

- The first rotation always runs to completion.
- After it, each update transitions to pointing when ``sizeOfObservations`` is at least
  ``observationThreshold``.
- If the full rotation sequence elapses without the sun being acquired, the module transitions to
  pointing regardless (forced).
- Once pointing, the module never returns to search (re-armed only by ``reset()``).

The output should be paired with a control module such as :ref:`rateControl` /
:ref:`mrpFeedback`.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.

.. list-table:: Module I/O Messages
    :widths: 20 30 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - sunDirectionInMsg
      - :ref:`NavAttMsgF32Payload`
      - input message containing the sun direction vector :math:`\mathbf s` in body frame coordinates
    * - rateInMsg
      - :ref:`NavAttMsgF32Payload`
      - input message containing the inertial body angular velocity :math:`\mathbf\omega_{B/N}`
    * - filterResidualsInMsg
      - :ref:`FilterResidualsMsgF32Payload`
      - input message whose ``sizeOfObservations`` field carries the number of valid CSS observations,
        used to detect sun acquisition
    * - attGuidanceOutMsg
      - :ref:`AttGuidMsgF32Payload`
      - output message of attitude tracking errors and reference frame states
    * - sunSearchPointFaultOutMsg
      - :ref:`SunSearchPointFaultMsgPayload`
      - output message whose ``faultDetected`` flag latches ``true`` if the search sequence elapses
        without acquiring the sun (the forced transition to pointing); re-armed only by ``reset()``

Module Parameters
-------------------------------
.. list-table:: Module Parameters
    :widths: 30 20 10 10 40
    :header-rows: 1

    * - Parameter Name
      - Type
      - Units
      - Default
      - Description
    * - sHatBdyCmd (required)
      - Eigen::Vector3f
      - [-]
      - zero
      - Commanded body-fixed unit direction vector :math:`\hat{\mathbf s}_{c}` to align with the sun
        (norm must be within ``1e-3`` of 1.0; renormalized on storage)
    * - sunAxisSpinRate
      - float
      - [rad/s]
      - 0
      - Desired constant spin rate :math:`\dot\theta` about the sun heading vector
    * - omega_RN_B
      - Eigen::Vector3f
      - [rad/s]
      - zero
      - Body-fixed fallback rate commanded in the pointing phase when no sun direction is available
    * - observationThreshold
      - int
      - [-]
      - 4
      - CSS observation count at or above which the sun is considered acquired
    * - rotations
      - RotationProperties[4]
      - --
      - 1 s, 0 rad/s, b1
      - Sun-search rotation sequence; each entry has a duration (> 0), a signed body rate, and an axis.
        Set via ``setRotation(index, rotation)``.

Module Assumptions and Limitations
-----------------------------------
- The input sun direction vector :math:`\mathbf s` can have any length. Only the exact zero vector is
  treated as sun-not-available; any non-zero magnitude is accepted.
- The commanded body-relative unit direction vector :math:`\hat{\mathbf s}_{c}` is fixed relative to the body.
- The sun-pointing condition is under-determined (2 DOF): the rotation about :math:`\mathbf s` is arbitrary.
- Each rotation's duration must be finite and strictly positive; the commanded rate must be finite
  (any sign). Invalid sequences are rejected when the configuration is installed.

Initialization
--------------
The module is configured by::

    module = sunSearchPointF32.SunSearchPoint()
    module.modelTag = "sunSearchPoint"
    module.sHatBdyCmd = [0.0, 0.0, 1.0]
    module.sunAxisSpinRate = 0.0
    module.omega_RN_B = [0.0, 0.0, 0.0]
    module.observationThreshold = 4

    rotation = sunSearchPointF32.RotationProperties()
    rotation.rotationDuration = 30.0
    rotation.rotationRate = 0.1
    rotation.rotationAxis = sunSearchPointF32.RotationAxis_b1Hat_B
    module.setRotation(0, rotation)
    # ... repeat for indices 1..3

Detailed Module Description
---------------------------

.. figure:: ./_Documentation/Figures/sunHeading.pdf
   :width: 50%
   :align: center

   Body Vector Illustrations

Phase State Machine
^^^^^^^^^^^^^^^^^^^
On the first update after ``reset()`` the module latches the current time as the search start and
enters the SEARCH phase. Let :math:`t_e` be the elapsed time since the start and
:math:`T_k` the cumulative end time of rotation :math:`k`. The transition to POINT occurs when

.. math::

   (t_e \geq T_0 \ \text{and}\ \texttt{sizeOfObservations} \geq \texttt{observationThreshold})
   \quad\text{or}\quad t_e \geq T_{N-1}

where :math:`N` is the number of rotations. POINT is terminal. When the transition is *forced* by
the second condition alone — the full sequence elapses (:math:`t_e \geq T_{N-1}`) without the
observation count ever reaching the threshold — the search has failed to acquire the sun. In that
case the module latches a fault: ``sunSearchPointFaultOutMsg.faultDetected`` is set ``true`` and stays
``true`` for the rest of the run (even if the sun is later seen), cleared only by ``reset()``.

Search Phase
^^^^^^^^^^^^
For each active rotation the module outputs a constant reference angular velocity
:math:`\mathbf\omega_{R/N}` along the corresponding body axis and zero attitude error
:math:`\mathbf\sigma_{B/R} = \mathbf 0`. The active rotation is the first whose cumulative end time
has not yet elapsed. The body-rate error is

.. math::

   \mathbf\omega_{B/R} = \mathbf\omega_{B/N} - \mathbf\omega_{R/N}

Pointing Phase — Good Sun Direction Vector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The principal rotation vector and angle aligning :math:`\hat{\mathbf s}_{c}` with the sun heading
:math:`\mathbf s` are

.. math::

   \hat{\mathbf e} = \frac{\mathbf s \times \hat{\mathbf s}_{c}}{|\mathbf s \times \hat{\mathbf s}_{c}|}
   \qquad
   \Phi = \arccos \left( \frac{\mathbf s \cdot \hat{\mathbf s}_{c}}{|\mathbf s|} \right)

The attitude error MRP and reference rate are

.. math::

   \mathbf\sigma_{B/R} = \tan\left(\frac{\Phi}{4}\right) \hat{\mathbf e}
   \qquad
   {}^{\mathcal{B}}\mathbf\omega_{R/N} = {}^{\mathcal{B}}\hat{\mathbf s}\,\dot\theta

with rate tracking error :math:`\mathbf\omega_{B/R} = \mathbf\omega_{B/N} - \mathbf\omega_{R/N}` and
zero reference acceleration :math:`\dot{\mathbf\omega}_{R/N} = \mathbf 0`.

Pointing Phase — No Sun Direction Vector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
If :math:`\mathbf s` is the zero vector, the attitude error is set to zero and the configured
fallback rate is commanded: :math:`\mathbf\sigma_{B/R} = \mathbf 0`,
:math:`\mathbf\omega_{R/N} = \texttt{omega\_RN\_B}`. This is the branch reached when the search
sequence elapses without acquiring the sun, keeping the vehicle slewing rather than at rest. The
branch is evaluated every update, so re-acquiring the sun resumes pointing immediately.

Collinear Commanded and Sun Heading Vectors
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
When :math:`\mathbf s \approx \hat{\mathbf s}_{c}` the cross product vanishes; ``stableNormalized``
returns the zero vector and :math:`\tan(\Phi/4) \to 0`, so :math:`\mathbf\sigma_{B/R} = \mathbf 0`
without a special case. When :math:`\mathbf s \approx -\hat{\mathbf s}_{c}` (180°), a fallback
eigen-axis orthogonal to :math:`\hat{\mathbf s}_{c}` is used:
:math:`\hat{\mathbf e}_{180} = \texttt{sHatBdyCmd.unitOrthogonal()}`.

Test Description
-------------------------------
The module is verified through regression tests against an independently coded reference
implementation of the pointing logic, property-based tests (MRP norm bounds, the rate identity
:math:`\mathbf\omega_{B/R} = \mathbf\omega_{B/N} - \mathbf\omega_{R/N}`, finiteness), edge-case
tests (collinear and 180° vectors, zero-vector sun-not-visible, the :math:`\hat{\mathbf e}_{180}`
fallback), and search/state-machine tests covering active-rotation selection, the first-rotation
commit window, the observation-threshold transition, the forced transition after the full sequence,
the terminal pointing phase, and the zero-sun fallback rate. The module adapter is exercised in
Python for input/output wiring, the search-to-point transition, and the ``reset()`` link checks.
