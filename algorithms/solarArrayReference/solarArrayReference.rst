.. raw:: latex

    {\LARGE \textbf{solarArrayReference}}

Executive Summary
-----------------
This module computes the reference rotation angle :math:`\theta_R` for a single-axis solar array drive. In the
default ``AUTO_TRACK`` mode, :math:`\theta_R` is the angle that aligns the solar array surface normal with the Sun
direction as well as possible (perfect incidence is achievable when the drive axis and the Sun direction are
perpendicular). In ``SPECIFIED_ANGLE`` mode, the module ignores the Sun direction and outputs a user-supplied fixed
angle. In both modes, an optional offset angle is added before the result is wrapped to :math:`[-\pi, \pi]`.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg connection is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 30 30 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - attNavInMsg
      - :ref:`NavAttMsgF32Payload`
      - input attitude navigation message containing :math:`\mathbf\sigma_{\mathcal{B}/\mathcal{N}}` and the Sun
        direction in body-frame components
    * - attRefInMsg
      - :ref:`AttRefMsgF32Payload`
      - input attitude reference message containing :math:`\mathbf\sigma_{\mathcal{R}/\mathcal{N}}`
    * - hingedRigidBodyInMsg
      - :ref:`HingedRigidBodyMsgF32Payload`
      - input hinged rigid body message containing the current panel angle :math:`\theta_C`
    * - hingedRigidBodyRefOutMsg
      - :ref:`HingedRigidBodyMsgF32Payload`
      - output hinged rigid body reference message containing the reference angle :math:`\theta_R`

Module Parameters
-------------------------------
The following table lists all the module parameters than can be set. The parameters are optional unless indicated
(if not specified default is used).

.. list-table:: Module Parameters
    :widths: 40 20 10 10 30 30
    :header-rows: 1

    * - Parameter Name
      - Type
      - Units
      - Default
      - Description
      - Bounds
    * - driveAxis (required)
      - Eigen::Vector3f
      - [-]
      - zero
      - Solar array drive axis :math:`{}^\mathcal{B}\hat{\mathbf a}_1` in body frame, set via
        ``setSolarArrayAxes_B``
      - Norm must be within ``1e-3`` of 1.0 (checked in setter; renormalized on storage)
    * - surfaceNormal (required)
      - Eigen::Vector3f
      - [-]
      - zero
      - Solar array surface normal :math:`{}^\mathcal{B}\hat{\mathbf a}_2` at zero rotation, set via
        ``setSolarArrayAxes_B``
      - Norm must be within ``1e-3`` of 1.0 and orthogonal to ``driveAxis`` (absolute value of dot product
        less than ``1e-5`` after normalization); re-orthogonalized against ``driveAxis`` on storage
    * - alignmentThreshold
      - float
      - [rad]
      - ``1e-3``
      - Threshold angle :math:`\epsilon_a` between the Sun direction and the drive axis below which the Sun is
        considered aligned with the drive axis (no preferred rotation)
      - Must be in :math:`[10^{-3},\, \pi/2]` (checked in setter). The lower bound matches the fp32 precision
        floor of the alignment check: unit-vector rounding of :math:`\mathcal{O}(\varepsilon_{\text{f32}})
        \approx 10^{-7}` produces a dot product :math:`1 - \mathcal{O}(10^{-7})`, and :math:`\arccos`
        amplifies this to :math:`\sqrt{2\cdot 10^{-7}} \approx 5\times 10^{-4}` rad, so any threshold below
        :math:`10^{-3}` rad is below the noise floor.
    * - trackingMode
      - TrackingMode
      - [-]
      - ``AUTO_TRACK``
      - Selects between Sun-tracking (``AUTO_TRACK``) and fixed-angle (``SPECIFIED_ANGLE``) reference computation
      - N/A
    * - specifiedArrayAngle
      - float
      - [rad]
      - 0
      - Reference angle returned when ``trackingMode`` is ``SPECIFIED_ANGLE``
      - Must be in :math:`[-\pi, \pi]` (checked in setter)
    * - offsetAngle
      - float
      - [rad]
      - 0
      - Offset added to the computed reference angle before wrapping
      - Must be in :math:`[-\pi, \pi]` (checked in setter)

Module Assumptions and Limitations
----------------------------------
This module computes the rotation angle required to achieve the best incidence angle between the Sun direction and
the solar array surface. This does not mean that perfect incidence (Sun direction perpendicular to array surface)
is guaranteed: perfect incidence is only achievable when the drive axis and the Sun direction are perpendicular.
Conversely, when they are parallel, no power generation is possible, and in that case the reference is set to the
current panel angle to avoid pointless rotation.

The drive axis :math:`\hat{\mathbf a}_1` and surface normal :math:`\hat{\mathbf a}_2` are assumed to be fixed in the
body frame. The Sun direction in body-frame components is extracted from ``attNavInMsg`` and is mapped into the
reference frame using the body and reference attitudes from ``attNavInMsg`` and ``attRefInMsg``, so the reference
angle is computed in the frame that the spacecraft will occupy at the end of the active slew.

The output reference angle :math:`\theta_R` is always wrapped to :math:`[-\pi, \pi]`.

Initialization
--------------
The module is configured by::

    module = solarArrayReferenceF32.SolarArrayReference()
    module.modelTag = "solarArrayReference"
    module.setSolarArrayAxes_B([1.0, 0.0, 0.0], [0.0, 1.0, 0.0])
    module.alignmentThreshold = 1e-3
    module.offsetAngle = 0.0

For ``AUTO_TRACK`` mode (the default), no further configuration is required. To use ``SPECIFIED_ANGLE`` mode::

    module.trackingMode = solarArrayReferenceF32.TrackingMode_SPECIFIED_ANGLE
    module.specifiedArrayAngle = 0.5

Detailed Module Description
---------------------------
For this module to operate, the user provides two body-fixed unit directions:

- :math:`{}^\mathcal{B}\hat{\mathbf a}_1`: drive axis, about which the array rotates;
- :math:`{}^\mathcal{B}\hat{\mathbf a}_2`: surface normal at zero rotation.

These vectors must be (near-)unit length and orthogonal. The setter ``setSolarArrayAxes_B`` validates the norms,
verifies orthogonality, normalizes both inputs, and re-orthogonalizes :math:`\hat{\mathbf a}_2` against
:math:`\hat{\mathbf a}_1` for exact orthogonality on storage.

Algorithm Flow
^^^^^^^^^^^^^^
At every update cycle, the ``solarArrayReference`` module performs the following steps:

1. **Select tracking mode**:

   - If ``trackingMode`` is ``AUTO_TRACK``, compute :math:`\theta_R` from the Sun direction (steps 2-4).
   - If ``trackingMode`` is ``SPECIFIED_ANGLE``, set :math:`\theta_R = \theta_{\text{specified}}` and skip to step 5.

2. **Map Sun direction to reference frame**: normalize the body-frame Sun direction
   :math:`{}^{\mathcal{B}_C}\hat{\mathbf r}_S` (using ``stableNormalized``), then map it into the reference frame
   using the DCM :math:`[\mathcal{R}\mathcal{B}] = [\mathcal{R}\mathcal{N}][\mathcal{B}\mathcal{N}]^T`:

   .. math::

      {}^\mathcal{R}\hat{\mathbf r}_S = [\mathcal{R}\mathcal{B}]\, {}^{\mathcal{B}_C}\hat{\mathbf r}_S

3. **Check Sun/drive-axis alignment**: compute the angle between the Sun direction and the drive axis,

   .. math::

      \alpha = \arccos\!\left( \left|\, {}^\mathcal{R}\hat{\mathbf r}_S \cdot \hat{\mathbf a}_1\, \right| \right)

   If :math:`\alpha < \epsilon_a` (or the Sun direction is the zero vector), the Sun is aligned with the drive axis
   and there is no preferred rotation: set :math:`\theta_R = \theta_C` (the current panel angle from
   ``hingedRigidBodyInMsg``, with no offset applied) and skip to the wrap in step 5.

4. **Compute reference angle** (Sun not aligned with drive axis): expressed in the body-fixed orthonormal frame
   :math:`\{\hat{\mathbf a}_1,\, \hat{\mathbf a}_2,\, \hat{\mathbf a}_3\}` with
   :math:`\hat{\mathbf a}_3 = \hat{\mathbf a}_1 \times \hat{\mathbf a}_2`, the signed rotation about
   :math:`\hat{\mathbf a}_1` that carries :math:`\hat{\mathbf a}_2` toward the in-plane projection of the Sun
   direction is

   .. math::

      \theta_R = \operatorname{atan2}\!\left(
        \hat{\mathbf a}_3 \cdot {}^\mathcal{R}\hat{\mathbf r}_S,\;
        \hat{\mathbf a}_2 \cdot {}^\mathcal{R}\hat{\mathbf r}_S
      \right)

   The :math:`\hat{\mathbf a}_1`-component of :math:`{}^\mathcal{R}\hat{\mathbf r}_S` drops out because
   :math:`\hat{\mathbf a}_2` and :math:`\hat{\mathbf a}_3` are both perpendicular to :math:`\hat{\mathbf a}_1`,
   so no explicit projection/normalization is required. :math:`\hat{\mathbf a}_3` is computed once in the setter
   and cached.

5. **Apply offset and wrap**: when the Sun was not aligned with the drive axis (and in ``SPECIFIED_ANGLE`` mode),
   add the configured offset angle. In all cases, wrap the result to :math:`[-\pi, \pi]`:

   .. math::

      \theta_R \leftarrow \operatorname{atan2}\!\left( \sin(\theta_R + \theta_{\text{offset}}),\, \cos(\theta_R + \theta_{\text{offset}}) \right)

   In the Sun-aligned fallback the offset is intentionally not applied — only the wrap step is performed.

Wrapping and the Sun-Aligned Case
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The :math:`\operatorname{atan2}` in step 4 produces values in :math:`[-\pi, \pi]` with the correct quadrant
directly. Adding the offset can push the sum outside this range, so the final
:math:`\operatorname{atan2}(\sin(\cdot), \cos(\cdot))` step guarantees the output range is always
:math:`[-\pi, \pi]`.

When the Sun direction is (nearly) aligned with the drive axis, its projection onto the :math:`(\hat{\mathbf a}_2,
\hat{\mathbf a}_3)` plane vanishes and :math:`\theta_R` is undefined. The module detects this via the
``alignmentThreshold`` check and falls back to returning the current panel angle :math:`\theta_C` directly,
without applying the offset (the offset is meaningful only against the Sun-tracking solution). Because the wrap is
applied unconditionally at the end, this also normalizes :math:`\theta_C` (which the caller may have provided
unwrapped) to :math:`[-\pi, \pi]`.

Specified Angle Mode
^^^^^^^^^^^^^^^^^^^^
In ``SPECIFIED_ANGLE`` mode the module bypasses the Sun-tracking computation entirely and returns the
user-supplied ``specifiedArrayAngle``. The offset angle and the final :math:`[-\pi, \pi]` wrap are still applied,
so:

.. math::

   \theta_R = \operatorname{atan2}\!\left( \sin(\theta_{\text{specified}} + \theta_{\text{offset}}),\, \cos(\theta_{\text{specified}} + \theta_{\text{offset}}) \right)

This mode is useful for parking the array at a known orientation (e.g. during slews or eclipse) without depending
on a valid Sun direction measurement.
