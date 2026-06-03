Executive Summary
-----------------
This module commands a sequence of constant-rate rotations about user-specified body axes.
The module supports ``kNumRotations`` (4) consecutive rotation maneuvers. Each rotation is defined
by a rotation axis, a commanded body rate magnitude, and a duration. The axes are user defined
and any combination of the three principal axes is allowed, including repeated rotations about
the same axis.

The module reads the spacecraft's current angular velocity and computes a guidance message
containing the reference rate ``omega_RN_B``, the body-rate error ``omega_BR_B``, and zero
reference angular acceleration. The output of this module should be paired with a rate control
module such as :ref:`rateControl`.


Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.

.. list-table:: Module I/O Messages
    :widths: 20 30 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - attNavInMsg
      - :ref:`NavAttMsgF32Payload`
      - Input message containing current spacecraft rates in body-frame coordinates.
    * - attGuidOutMsg
      - :ref:`AttGuidMsgF32Payload`
      - Output guidance message containing reference and error rates.


Algorithm Inputs and Outputs
----------------------------
Algorithm inputs:

.. list-table::
   :widths: 25 75
   :header-rows: 1

   * - Variable Name
     - Description
   * - callTime
     - Current simulation time in nanoseconds, used to determine which rotation is active.
   * - omega_BN_B
     - Measured spacecraft angular velocity in the body frame :math:`\boldsymbol{\omega}_{B/N}`.

Algorithm outputs:

.. list-table::
   :widths: 25 75
   :header-rows: 1

   * - Variable Name
     - Description
   * - omega_RN_B
     - Reference angular velocity in the body frame :math:`\boldsymbol{\omega}_{R/N}`.
       Held at the last rotation's commanded rate after the sequence completes.
   * - omega_BR_B
     - Body-rate error in the body frame :math:`\boldsymbol{\omega}_{B/R} = \boldsymbol{\omega}_{B/N} - \boldsymbol{\omega}_{R/N}`.


Detailed Module Description
---------------------------
This module is intended to be used when the spacecraft might not have a valid Sun direction
estimate. It commands a sequence of rotations to sweep the body-fixed Sun sensors through
different orientations until the Sun is found. The search pattern consists of exactly
``kNumRotations`` (4) consecutive rotation maneuvers. Each maneuver rotates the spacecraft
at a constant rate about a user-specified body axis for a user-specified duration. The four
rotations execute sequentially: rotation 1 runs for its full duration, then rotation 2
begins, and so on.

For each active rotation, the module outputs a constant reference angular velocity
``omega_RN_B`` along the corresponding body axis. The body-rate error is computed as:

.. math::

   \boldsymbol{\omega}_{B/R} = \boldsymbol{\omega}_{B/N} - \boldsymbol{\omega}_{R/N}

where :math:`\boldsymbol{\omega}_{B/N}` is the measured body rate from the navigation
message and :math:`\boldsymbol{\omega}_{R/N}` is the commanded reference rate.
The body-rate error is computed here because the ``attitudeTrackingError`` module is not
used in Safe mode.

After all four rotations have completed, the algorithm continues to command the last
rotation's angular velocity indefinitely. This keeps the spacecraft rotating if the Sun
has not been acquired by the end of the scripted sequence; the final rotation's rate and
axis therefore double as the indefinite-hold configuration.

Each rotation's ``rotationDuration`` must be finite and strictly greater than
zero. Zero, negative, NaN, and infinite values are rejected at configuration
time. Internally the algorithm accumulates rotation end-times in 64-bit integer
nanoseconds, so cumulative durations are exact at the precision of the input.


User Guide
----------
The required module configuration is::

    attGuid = sunSearchF32.SunSearch()

    rotationProp = sunSearchF32.RotationProperties()
    rotationProp.rotationDuration = 5.0
    rotationProp.rotationRate = 0.1
    rotationProp.rotationAxis = sunSearchF32.RotationAxis_b1Hat_B
    attGuid.setRotation(0, rotationProp)

    rotationProp = sunSearchF32.RotationProperties()
    rotationProp.rotationDuration = 10.0
    rotationProp.rotationRate = 0.2
    rotationProp.rotationAxis = sunSearchF32.RotationAxis_b2Hat_B
    attGuid.setRotation(1, rotationProp)

    rotationProp = sunSearchF32.RotationProperties()
    rotationProp.rotationDuration = 7.0
    rotationProp.rotationRate = 0.15
    rotationProp.rotationAxis = sunSearchF32.RotationAxis_b3Hat_B
    attGuid.setRotation(2, rotationProp)

    rotationProp = sunSearchF32.RotationProperties()
    rotationProp.rotationDuration = 3.0
    rotationProp.rotationRate = 0.05
    rotationProp.rotationAxis = sunSearchF32.RotationAxis_b1Hat_B
    attGuid.setRotation(3, rotationProp)

To retrieve or modify a rotation after it has been set::

    rotation = attGuid.getRotation(1)
    rotation.rotationRate = 0.3
    attGuid.setRotation(1, rotation)
