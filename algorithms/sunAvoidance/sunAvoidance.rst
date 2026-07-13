.. raw:: latex

    {\LARGE \textbf{sunAvoidance}}

Executive Summary
-----------------
This module produces a Sun-avoidance maneuver-adjusted attitude reference. Given the measured body attitude, an input
reference frame, a body-fixed *sensitive* axis to keep off the Sun, and the spacecraft and Sun inertial positions, the
module superimposes a decaying rotation onto the input reference. The adjusted reference therefore starts at the current
body attitude and slews to the input reference at a configured rate, choosing the short or long way around so that the
sensitive body axis does not sweep across the Sun during the slew. The adjusted reference is written as an attitude
reference message intended to feed :ref:`attTrackingError` downstream, which forms the attitude tracking error from it
and the navigation attitude.

The Sun-avoidance maneuver is only computed when both optional inputs -- the spacecraft translational state and the Sun
ephemeris -- are connected and a valid Sun direction is available. Otherwise, and once the maneuver has decayed to zero,
the input reference is passed through unchanged.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages. The module msg connection is set by the user from
python. The msg type contains a link to the message structure definition, while the description provides information on
what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 20 30 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - attNavInMsg
      - :ref:`NavAttMsgF32Payload`
      - input message with the measured body attitude :math:`\mathbf\sigma_{B/N}`
    * - attRefInMsg
      - :ref:`AttRefMsgF32Payload`
      - input reference frame :math:`(\mathbf\sigma_{R/N},\ \mathbf\omega_{R/N},\ \dot{\mathbf\omega}_{R/N})`
    * - transNavInMsg
      - :ref:`NavTransMsgF32Payload`
      - optional input with the spacecraft inertial position :math:`\mathbf r_{B/N}`
    * - ephemerisInMsg
      - :ref:`EphemerisMsgF32Payload`
      - optional input with the Sun inertial position :math:`\mathbf r_{S/N}`
    * - attRefOutMsg
      - :ref:`AttRefMsgF32Payload`
      - output maneuver-adjusted reference frame

The Sun-avoidance maneuver is enabled only when **both** ``transNavInMsg`` and ``ephemerisInMsg`` are connected. When
either is left unconnected the module outputs the input reference unchanged.

Module Parameters
-----------------
The following table lists the module parameters that can be set. They must be configured before ``reset()`` is called.

.. list-table:: Module Parameters
    :widths: 30 20 10 10 30 30
    :header-rows: 1

    * - Parameter Name
      - Type
      - Units
      - Default
      - Description
      - Bounds
    * - sensitiveHat_B
      - Eigen::Vector3f
      - [-]
      - zero
      - Body-fixed sensitive axis :math:`\hat{\mathbf a}_B` to keep off the Sun
      - Must be finite; renormalized on storage
    * - angleRate
      - float
      - [rad/s]
      - 0
      - Rate :math:`\dot\Phi` at which the maneuver slews toward the input reference
      - Must be finite

Module Notes
------------
- The Sun-avoidance maneuver is enabled only when both optional messages (``transNavInMsg`` and ``ephemerisInMsg``) are
  connected. The module's behavior when they are absent, or when the geometry is degenerate, is described under Edge
  Case Handling.
- The spacecraft and Sun positions are consumed in **double precision**: the large inertial vectors are differenced in
  double and only the resulting unit Sun direction is reduced to single precision, which avoids catastrophic
  cancellation. All other computation is single precision (fp32).
- The maneuver is initialized **once**, on the first ``updateState`` after ``reset()``, and fed forward on subsequent
  calls; ``reInitialize()`` clears the state so the next update recomputes it.
- The short-versus-long-way choice is a discrete decision. Near its boundary the choice is sensitive to rounding; both
  choices are valid maneuvers, but two independent implementations may select opposite ones.

Initialization
--------------
The module is configured by::

    module = sunAvoidanceF32.SunAvoidance()
    module.modelTag = "sunAvoidance"
    module.sensitiveHat_B = [0.0, -1.0, 0.0]
    module.angleRate = 0.017453  # 1 deg/s

    # connect attNavInMsg and attRefInMsg (always); connect transNavInMsg and
    # ephemerisInMsg as well to enable the Sun-avoidance maneuver.

Detailed Module Description
---------------------------
On the first update the maneuver is initialized from the current geometry; every update then superimposes the residual
maneuver on the input reference. Initialization proceeds in two phases.

Phase 1 -- The Maneuver
^^^^^^^^^^^^^^^^^^^^^^^^^
The maneuver is the principal rotation from the current body attitude :math:`\mathcal B` to the input reference
:math:`\mathcal R`. With :math:`[BN] = \mathrm{dcm}(\mathbf\sigma_{B/N})` and :math:`[RN] = \mathrm{dcm}(\mathbf\sigma_{R/N})`,

.. math::

   [BR] = [BN]\,[RN]^\top, \qquad
   \boldsymbol{\Phi}_{B/R} = \mathrm{PRV}([BR]), \qquad
   \Phi = |\boldsymbol{\Phi}_{B/R}|, \qquad
   \hat{\mathbf e} = \frac{\boldsymbol{\Phi}_{B/R}}{|\boldsymbol{\Phi}_{B/R}|}

:math:`\Phi` is the maneuver angle and :math:`\hat{\mathbf e}` its axis. The normalization uses ``stableNormalized``,
which returns the zero vector (not ``NaN``) when :math:`\boldsymbol{\Phi}_{B/R}\approx\mathbf 0`, i.e. when the body already
coincides with the reference and no maneuver is needed.

Phase 2 -- Short vs Long Way
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The short-way maneuver of Phase 1 is reversed to the long way around when it would sweep the sensitive axis across the
Sun. Let :math:`\hat{\mathbf s}` be the inertial Sun direction and let the sensitive axis be expressed in inertial
components at the initial (body) and final (reference) attitudes:

.. math::

   \hat{\mathbf s} = \frac{\mathbf r_{S/N} - \mathbf r_{B/N}}{|\mathbf r_{S/N} - \mathbf r_{B/N}|}, \qquad
   \mathbf a_i = [BN]^\top \hat{\mathbf a}_B, \qquad
   \mathbf a_f = [RN]^\top \hat{\mathbf a}_B

The sensitive axis sweeps in the plane whose normal is the sweep axis :math:`\hat{\mathbf n}`; the Sun is projected into
that plane as :math:`\hat{\mathbf p}`:

.. math::

   \hat{\mathbf n} = \frac{\mathbf a_i \times \mathbf a_f}{|\mathbf a_i \times \mathbf a_f|}, \qquad
   \hat{\mathbf p} = \frac{\hat{\mathbf s} - (\hat{\mathbf n}\cdot\hat{\mathbf s})\,\hat{\mathbf n}}
                          {|\hat{\mathbf s} - (\hat{\mathbf n}\cdot\hat{\mathbf s})\,\hat{\mathbf n}|}

The extent of the sweep and the Sun's angular position along it (from the initial sensitive direction) are

.. math::

   \beta = \arccos(\mathbf a_i \cdot \mathbf a_f), \qquad
   \alpha = \arccos(\mathbf a_i \cdot \hat{\mathbf p})

Let :math:`\hat{\mathbf a}_{i\times s}` be the axis that rotates the sensitive direction toward the Sun, and let
:math:`\hat{\mathbf e}_{i\to r}` be the maneuver's initial-to-reference axis in inertial components -- the negative of
the stored (reference-to-initial) axis :math:`\hat{\mathbf e}`:

.. math::

   \hat{\mathbf a}_{i\times s} = \frac{\mathbf a_i \times \hat{\mathbf s}}{|\mathbf a_i \times \hat{\mathbf s}|}, \qquad
   \hat{\mathbf e}_{i\to r} = -[BN]^\top \hat{\mathbf e}

The maneuver turns the sensitive axis toward the Sun when :math:`\hat{\mathbf a}_{i\times s}\cdot\hat{\mathbf e}_{i\to r}
> 0`. The short-way maneuver is reversed to the long way when it turns toward the Sun **and** the Sun lies within the
swept arc:

.. math::

   \big(\hat{\mathbf a}_{i\times s}\cdot\hat{\mathbf e}_{i\to r} > 0\big) \ \wedge\ (\alpha < \beta)
   \quad\Longrightarrow\quad
   \Phi \leftarrow 2\pi - \Phi, \qquad \hat{\mathbf e} \leftarrow -\hat{\mathbf e}

The reversal is applied only when the three directions it depends on -- the sweep axis :math:`\hat{\mathbf n}`, the
in-plane Sun direction :math:`\hat{\mathbf p}`, and the sensitive-to-Sun axis :math:`\hat{\mathbf a}_{i\times s}` -- are
all non-zero. If any is zero the geometry is ambiguous and the short way is kept; this single guard makes "any
degenerate direction keeps the short way" explicit and covers both the parallel and anti-parallel sensitive-axis cases
(see Edge Case Handling).

Adjusted Reference
^^^^^^^^^^^^^^^^^^^
On every update the residual maneuver angle is fed forward at the configured rate and clamped at zero:

.. math::

   \Phi_r(t) = \max\!\big(0,\ \Phi - \dot\Phi\,(t - t_0)\big)

where :math:`t_0` is the time the maneuver began. The adjusted reference attitude is the input reference rotated by the
residual maneuver, and while the maneuver is active a feed-forward rate is added; the reference acceleration is passed
through:

.. math::

   [R_cN] = \mathrm{dcm}(\Phi_r\,\hat{\mathbf e})\,[RN], \qquad
   \mathbf\sigma_{R_c/N} = \mathrm{MRP}([R_cN])

.. math::

   \mathbf\omega_{R_c/N} =
   \begin{cases}
   \mathbf\omega_{R/N} - \dot\Phi\,[BN]^\top\hat{\mathbf e}, & \Phi_r > 0 \\
   \mathbf\omega_{R/N}, & \Phi_r = 0
   \end{cases}
   \qquad
   \dot{\mathbf\omega}_{R_c/N} = \dot{\mathbf\omega}_{R/N}

When the maneuver is disabled or has decayed (:math:`\Phi_r = 0`), the adjusted reference equals the input reference.

Edge Case Handling
^^^^^^^^^^^^^^^^^^^
Every degenerate geometry falls back to a well-defined maneuver rather than producing ``NaN``. All normalizations use
``stableNormalized``, which returns the zero vector (not ``NaN``) for a near-zero argument, so the output is always
finite; the reversal test is evaluated only when its geometry is well defined.

.. list-table:: Edge Cases
    :widths: 45 55
    :header-rows: 1

    * - Condition
      - Handling
    * - Optional messages absent, zero Sun position, or Sun coincident with the spacecraft (no usable Sun direction)
      - Maneuver disabled; the adjusted reference equals the input reference (pass-through).
    * - Body already at the reference (:math:`\boldsymbol{\Phi}_{B/R} \approx \mathbf 0`)
      - Zero maneuver angle; the adjusted reference equals the input reference.
    * - Initial and final sensitive axes parallel or anti-parallel (:math:`\hat{\mathbf n}` undefined)
      - No well-defined sweep plane; keep the short way.
    * - Sun parallel to the sweep axis (:math:`\hat{\mathbf p}` undefined)
      - The sweep never approaches the Sun; keep the short way.
    * - Sun parallel to the initial sensitive axis (:math:`\hat{\mathbf a}_{i\times s}` undefined)
      - Toward/away is undefined; keep the short way.
    * - Maneuver decayed (:math:`\Phi_r = 0`)
      - The adjusted reference equals the input reference.

Test Description
----------------
The algorithm is verified through regression tests against an independently coded reference implementation (compared
via the reference-attitude DCM, which is invariant to the MRP shadow set), Config validation and getter setup tests,
property tests (pass-through when the maneuver is disabled, bounded and finite output while maneuvering, return to the
input reference after decay, and ``reInitialize`` restarting the maneuver), and edge-case tests for the degenerate
geometries (missing Sun information, body at the reference, Sun along the sensitive axis, Sun perpendicular to the sweep
plane, and anti-parallel sensitive axes). The maneuver path is additionally regression-fuzzed with realistic Sun
geometry; the shared regression helper skips inputs near a degeneracy or near the discrete short/long-way decision
boundary, where an independent fp32 reference can select the opposite (equally valid) maneuver. A separate integrated
test pins the combined ``sunAvoidance`` :math:`\rightarrow` :ref:`attTrackingError` pipeline against a reference model
of the combined behavior.
