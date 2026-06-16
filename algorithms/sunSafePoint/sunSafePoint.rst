.. raw:: latex

    {\LARGE \textbf{sunSafePoint}}

Executive Summary
-----------------
This module computes the attitude guidance output for a sun-pointing mode, suitable for safe mode or power generation.
Given the sun direction vector (not necessarily normalized) and the body angular velocity, the module produces attitude
tracking errors as Modified Rodrigues Parameters (MRPs) and body rate tracking errors. The sun direction measurement is
crossed with a commanded body-fixed axis to obtain a principal rotation vector, and the dot product yields the principal
rotation angle. If the sun vector is not available (the zero vector), the attitude error is zeroed and a
configurable body-fixed search rate is used instead. An optional constant spin rate about the sun heading axis can be
specified.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg connection is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

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
    * - attGuidanceOutMsg
      - :ref:`AttGuidMsgF32Payload`
      - output message of attitude tracking errors and reference frame states

Module Parameters
-------------------------------
The following table lists all the module parameters that can be set. Required parameters must be
configured before the module is used.

.. list-table:: Module Parameters
    :widths: 40 20 10 10 30 30
    :header-rows: 1

    * - Parameter Name
      - Type
      - Units
      - Default
      - Description
      - Bounds
    * - sHatBdyCmd (required)
      - Eigen::Vector3f
      - [-]
      - zero
      - Commanded body-fixed unit direction vector :math:`\hat{\mathbf s}_{c}` to align with the sun
      - Norm must be within ``1e-3`` of 1.0 (checked in setter; vector is renormalized on storage)
    * - sunAxisSpinRate
      - float
      - [rad/s]
      - 0
      - Desired constant spin rate :math:`\dot\theta` about the sun heading vector
      - None
    * - omega_RN_B
      - Eigen::Vector3f
      - [rad/s]
      - zero
      - Body-fixed reference rate :math:`\mathbf\omega_{R/N}` used when no sun direction is available
      - None

Module Assumptions and Limitations
-----------------------------------
- The input sun direction vector :math:`\mathbf s` can have any length. Only the exact zero vector is treated as
  sun-not-available; any non-zero magnitude is accepted (normalization uses ``stableNormalized`` for numerical
  robustness with small inputs).
- The commanded body-relative unit direction vector :math:`\hat{\mathbf s}_{c}` is assumed to be fixed relative to the body.
- The sun-pointing condition is under-determined (2 DOF): the rotation about :math:`\mathbf s` is arbitrary.
  This is acceptable for power generation, which does not depend on orientation about the sun heading.

Initialization
--------------
The module is configured by::

    module = sunSafePointF32.SunSafePoint()
    module.modelTag = "sunSafePoint"
    module.sHatBdyCmd = [0.0, 0.0, 1.0]
    module.sunAxisSpinRate = 0.0
    module.omega_RN_B = [0.0, 0.0, 0.0]

Detailed Module Description
---------------------------

.. figure:: ./_Documentation/Figures/sunHeading.pdf
   :width: 50%
   :align: center

   Body Vector Illustrations

Algorithm Flow
^^^^^^^^^^^^^^
At every update cycle, the ``sunSafePoint`` module performs the following steps:

1. **Check sun visibility**: Normalize :math:`\mathbf s` using ``stableNormalized``. If the result is the zero
   vector (i.e. the input was exactly zero), set :math:`\mathbf\sigma_{B/R} = \mathbf 0`, use the configured
   ``omega_RN_B``, and skip to step 5.

2. **Compute the principal rotation angle** :math:`\Phi` between :math:`\mathbf s` and :math:`\hat{\mathbf s}_{c}`.

3. **Determine the eigen axis** :math:`\hat{\mathbf e}`:

   - If :math:`\pi - \Phi < \epsilon` (with :math:`\epsilon = 10^{-3}\,\text{rad}`): use a fallback axis
     :math:`\hat{\mathbf e}_{180}` orthogonal to :math:`\hat{\mathbf s}_{c}`, computed via Eigen's ``unitOrthogonal()``.
   - Otherwise: :math:`\hat{\mathbf e} = \mathbf s \times \hat{\mathbf s}_{c}`, normalized via ``stableNormalized``
     (which returns the zero vector when the cross product vanishes, so :math:`\Phi \approx 0` naturally yields
     :math:`\mathbf\sigma_{B/R} = \mathbf 0`).

4. **Compute attitude error MRP** :math:`\mathbf\sigma_{B/R} = \tan(\Phi/4)\,\hat{\mathbf e}` and the reference
   rate :math:`\mathbf\omega_{R/N} = \dot\theta\,\hat{\mathbf s}`.

5. **Compute rate tracking error** :math:`\mathbf\omega_{B/R} = \mathbf\omega_{B/N} - \mathbf\omega_{R/N}`.

Equations
^^^^^^^^^

Good Sun Direction Vector Case
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In the following mathematical developments all vectors are assumed to be
taken with respect to a body-fixed frame :math:`\mathcal B`. The attitude of
the body :math:`\mathcal B` relative to the reference frame :math:`\mathcal R`
is written as a principal rotation from :math:`\mathcal R` to
:math:`\mathcal B`. Thus, the associated principal rotation vector
:math:`\hat{\mathbf e}` is

.. math::

   \hat{\mathbf e} = \frac{\mathbf s \times \hat{\mathbf s}_{c}}{|\mathbf s \times \hat{\mathbf s}_{c}|}

Note that the sun direction vector :math:`\mathbf s` does not have to be a
normalized input vector.

The principal rotation angle between the two vectors is given through

.. math::

   \Phi = \arccos \left( \frac{\mathbf s  \cdot \hat{\mathbf s}_{c} }{|\mathbf s|} \right)

Next, this rotation from :math:`\mathcal R` to :math:`\mathcal B` is written as
a set of MRPs through

.. math::

   \mathbf\sigma_{B/R} = \tan\left(\frac{\Phi}{4}\right) \hat{\mathbf e}

The set :math:`\mathbf\sigma_{B/R}` is the attitude error of the output
attitude guidance message.

The module allows for a nominal spin rate about the sun heading axis by
specifying the module parameter ``sunAxisSpinRate`` called
:math:`\dot \theta` in this description. The nominal spin rate is thus
given by

.. math::

   {}^{\mathcal{B}}{\mathbf\omega}_{R/N} =
   {}^{\mathcal{B}}{\hat{\mathbf s}} \dot\theta

Note that this constant nominal spin is only for the case where the sun
is visible and the sun-heading vector measurement is available.

If the spacecraft is to be brought to rest
:math:`\mathbf\omega_{R/N} = \mathbf 0`, then :math:`\dot\theta` should be set
to zero. The tracking error angular velocity vector is computed using

.. math::

   \mathbf\omega_{B/R} = \mathbf\omega_{B/N} - \mathbf\omega_{R/N}

Finally, the attitude guidance message must specify the inertial
reference frame acceleration vector. This is set to zero as the roll
about the sun heading is assumed to have a constant magnitude and
inertial heading.

.. math::

   \dot{\mathbf \omega}_{R/N} = \mathbf 0

No Sun Direction Vector Case
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If :math:`\mathbf s` is the zero vector, then it is assumed that no good sun
heading vector is available and the attitude tracking error
:math:`\mathbf\sigma_{B/R}` is set to zero. Any non-zero magnitude is treated
as a valid sun direction; ``stableNormalized`` is used to handle small inputs
robustly.

Further, if the sun is not visible, the module allows for a non-zero
body rate to be prescribed. This allows the spacecraft to engage in a
constant rate tumble specified through the module configuration vector
``omega_RN_B``. In this case the tracking error rate is evaluated through

.. math::

   \mathbf\omega_{B/R} = \mathbf\omega_{B/N} - \mathbf\omega_{R/N}

and the output message reference rate is set equal to the prescribed
:math:`\mathbf\omega_{R/N}` while the reference acceleration vector
:math:`\dot{\mathbf \omega}_{R/N}` is set to zero.

Collinear Commanded and Sun Heading Vectors
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

First consider the case where :math:`\mathbf s \approx \hat{\mathbf s}_{c}`. In
this case the cross product :math:`\mathbf s \times \hat{\mathbf s}_{c}` vanishes
and is not a well-defined rotation axis. The module relies on
``stableNormalized``, which returns the zero vector when its input is too small
to safely normalize. The MRP is then computed as
:math:`\mathbf\sigma_{B/R} = \tan(\Phi/4)\,\hat{\mathbf e}`, where both factors
approach zero as :math:`\Phi \to 0`, so the resulting attitude error is
numerically zero without requiring an explicit small-angle branch.

However, if :math:`\mathbf s \approx -\hat{\mathbf s}_{c}`, then the cross
product is also near zero but :math:`\tan(\Phi/4)` is not, so a separate
fallback is required. An eigen-axis :math:`\hat{\mathbf e}` that is orthogonal
to :math:`\hat{\mathbf s}_{c}` must be determined. The choice of axis is
arbitrary (any vector perpendicular to :math:`\hat{\mathbf s}_{c}` produces a
valid 180° rotation), so the module delegates to Eigen's ``unitOrthogonal()``,
which returns a unit vector perpendicular to :math:`\hat{\mathbf s}_{c}` using a
numerically stable axis choice:

.. math::

   \hat{\mathbf e}_{180} = \texttt{sHatBdyCmd.unitOrthogonal()}

This fallback axis is computed inline in ``update()`` only on iterations where
:math:`\pi - \Phi < \epsilon`, with :math:`\epsilon = 10^{-3}\,\text{rad}`.

In this scenario the angular velocity tracking error is evaluated using
the same method as outlined in the Good Sun Direction Vector Case section.

Test Description
-------------------------------
The module is verified through regression tests against an independently coded reference implementation,
property-based tests covering MRP norm bounds, zero-error conditions, and the rate identity
:math:`\mathbf\omega_{B/R} = \mathbf\omega_{B/N} - \mathbf\omega_{R/N}`, as well as edge-case tests for collinear
vectors, 180-degree rotations, the zero-vector sun-not-visible case, and the
:math:`\hat{\mathbf e}_{180}` fallback axis.
