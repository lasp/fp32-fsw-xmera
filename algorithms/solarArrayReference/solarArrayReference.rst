.. raw:: latex

    {\LARGE \textbf{solarArrayReference}}

Executive Summary
-----------------
This module computes the reference rotation angle :math:`\theta_R` for a single-axis solar array drive. In the
default ``AUTO_TRACK`` mode, :math:`\theta_R` is the angle that aligns the solar array surface normal with the Sun
direction as well as possible (perfect incidence is achievable when the drive axis and the Sun direction are
perpendicular). The module then adds an optional offset angle. In ``SPECIFIED_ANGLE`` mode, the module ignores the
Sun direction and the offset angle, and outputs a user-supplied fixed angle. In both modes, the module wraps the
result to :math:`[-\pi, \pi]` and keeps it as the fallback for the next update. All quantities are computed in
single precision (float).

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
    * - sunHeadingInMsg
      - :ref:`InertialHeadingMsgF32Payload`
      - input Sun heading message containing the Sun direction in inertial-frame components
    * - attRefInMsg
      - :ref:`AttRefMsgF32Payload`
      - input attitude reference message containing :math:`\mathbf\sigma_{\mathcal{R}/\mathcal{N}}`
    * - solarArrayRefOutMsg
      - :ref:`MotorAngleRefMsgF32Payload`
      - output solar array reference message containing the reference angle :math:`\theta_R`

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
      - Solar array drive axis :math:`{}^\mathcal{B}\hat{\mathbf a}_1` in body frame
      - Norm must be within ``1e-3`` of 1.0 (validated at reset(); renormalized on storage)
    * - surfaceNormal (required)
      - Eigen::Vector3f
      - [-]
      - zero
      - Solar array surface normal :math:`{}^\mathcal{B}\hat{\mathbf a}_2` at zero rotation
      - Norm must be within ``1e-3`` of 1.0 and orthogonal to ``driveAxis`` (absolute value of dot product
        less than ``1e-5`` after normalization); re-orthogonalized against ``driveAxis`` on storage
    * - alignmentThreshold
      - float
      - [rad]
      - ``1e-3``
      - Threshold angle :math:`\epsilon_a` between the Sun direction and the drive axis below which the Sun is
        considered aligned with the drive axis (no preferred rotation)
      - Must be in :math:`[10^{-3},\, \pi/2]` (validated at reset()). The lower bound matches the fp32 precision
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
      - Must be in :math:`[-\pi, \pi]` (validated at reset())
    * - offsetAngle
      - float
      - [rad]
      - 0
      - Offset added to the Sun-tracking reference angle in ``AUTO_TRACK`` mode. Not applied in
        ``SPECIFIED_ANGLE`` mode.
      - Must be in :math:`[-\pi, \pi]` (validated at reset())

Module Assumptions and Limitations
----------------------------------
This module computes the rotation angle required to achieve the best incidence angle between the Sun direction and
the solar array surface. This does not mean that perfect incidence (Sun direction perpendicular to array surface)
is guaranteed: perfect incidence is only achievable when the drive axis and the Sun direction are perpendicular.
Conversely, when they are parallel, no power generation is possible, and in that case the module holds the
reference angle computed by the previous update to avoid pointless rotation.

The drive axis :math:`\hat{\mathbf a}_1` and surface normal :math:`\hat{\mathbf a}_2` are assumed to be fixed in the
body frame. The module maps the inertial Sun direction from ``sunHeadingInMsg`` into the reference frame with the
reference attitude from ``attRefInMsg``. Thus the module computes the reference angle in the frame that the
spacecraft will occupy at the end of the active slew.

The output reference angle :math:`\theta_R` is always wrapped to :math:`[-\pi, \pi]`.

The module assumes that the array drive reaches the commanded reference angle between two updates. The reference
angle :math:`\theta_R^-` from the previous update is therefore also the current panel angle, and the module uses
it when the Sun gives no preferred rotation angle.

Module Architecture
-------------------
The module is split into a pure algorithm (``SolarArrayReferenceAlgorithm``, Eigen-typed and free of any messaging
or framework dependencies) and an Xmera adapter (``SolarArrayReference``, a ``SysModel``) that owns the message I/O
and converts between the single-precision message payloads and Eigen types at the boundary. The adapter follows a
two-phase initialization pattern: the configuration properties and message connections are set first, then
``reset()`` builds and validates the immutable ``SolarArrayReferenceConfig`` (canonicalizing the drive axis and
surface normal into a right-handed orthonormal frame) and constructs the algorithm. An invalid parameter throws
``fsw::invalid_argument`` at ``reset()``. The algorithm holds the reference angle from the previous update; a new
algorithm starts at zero, and ``reInitialize()`` returns the retained angle to zero without rebuilding the
configuration. A C shim (``solarArrayReferenceAlgorithm_c.h``) exposes the algorithm to Ada via ``extern "C"``
bindings.

Initialization
--------------
The configuration properties and input messages must be set before ``reset()`` is called. The module is
configured by::

    module = solarArrayReferenceF32.SolarArrayReference()
    module.modelTag = "solarArrayReference"
    module.driveAxis = [1.0, 0.0, 0.0]
    module.surfaceNormal = [0.0, 1.0, 0.0]
    module.alignmentThreshold = 1e-3
    module.offsetAngle = 0.0

    module.sunHeadingInMsg.subscribeTo(sun_heading_msg)
    module.attRefInMsg.subscribeTo(att_ref_msg)

For ``AUTO_TRACK`` mode (the default), no further configuration is required. To use ``SPECIFIED_ANGLE`` mode::

    module.trackingMode = solarArrayReferenceF32.SPECIFIED_ANGLE
    module.specifiedArrayAngle = 0.5

Detailed Module Description
---------------------------
For this module to operate, the user provides two body-fixed unit directions:

- :math:`{}^\mathcal{B}\hat{\mathbf a}_1`: drive axis, about which the array rotates;
- :math:`{}^\mathcal{B}\hat{\mathbf a}_2`: surface normal at zero rotation.

These vectors must be (near-)unit length and orthogonal. When the configuration is built at ``reset()``, the
factory validates the norms, verifies orthogonality, normalizes both inputs, and re-orthogonalizes
:math:`\hat{\mathbf a}_2` against :math:`\hat{\mathbf a}_1` for exact orthogonality.

Algorithm Flow
^^^^^^^^^^^^^^
At every update cycle, the ``solarArrayReference`` module performs the following steps:

1. **Select tracking mode**:

   - If ``trackingMode`` is ``AUTO_TRACK``, compute :math:`\theta_R` from the Sun direction (steps 2-4).
   - If ``trackingMode`` is ``SPECIFIED_ANGLE``, set :math:`\theta_R = \theta_{\text{specified}}` and skip to the
     wrap in step 5.

2. **Map Sun direction to reference frame**: normalize the inertial-frame Sun direction
   :math:`{}^{\mathcal{N}}\hat{\mathbf r}_S` (using ``stableNormalized``), then map it into the reference frame
   with the DCM :math:`[\mathcal{R}\mathcal{N}]`:

   .. math::

      {}^\mathcal{R}\hat{\mathbf r}_S = [\mathcal{R}\mathcal{N}]\, {}^{\mathcal{N}}\hat{\mathbf r}_S

3. **Check Sun/drive-axis alignment**: compute the angle between the Sun direction and the drive axis,

   .. math::

      \alpha = \arccos\!\left( \left|\, {}^\mathcal{R}\hat{\mathbf r}_S \cdot \hat{\mathbf a}_1\, \right| \right)

   If :math:`\alpha < \epsilon_a` (or the Sun direction is the zero vector), the Sun is aligned with the drive axis
   and there is no preferred rotation: set :math:`\theta_R = \theta_R^-` (the reference angle from the previous
   update, with no offset applied) and skip to the wrap in step 5.

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
   so no explicit projection/normalization is required. :math:`\hat{\mathbf a}_3` is computed once when the
   configuration is built and cached.

5. **Apply offset and wrap**: in ``AUTO_TRACK`` mode, if the Sun is not aligned with the drive axis, add the
   offset angle. In all cases, wrap the result to :math:`[-\pi, \pi]`:

   .. math::

      \theta_R \leftarrow \operatorname{atan2}\!\left( \sin(\theta_R + \theta_{\text{offset}}),\, \cos(\theta_R + \theta_{\text{offset}}) \right)

   In the Sun-aligned fallback and in ``SPECIFIED_ANGLE`` mode, the module does only the wrap step.

Wrapping and the Sun-Aligned Case
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The :math:`\operatorname{atan2}` in step 4 produces values in :math:`[-\pi, \pi]` with the correct quadrant
directly. Adding the offset can push the sum outside this range, so the final
:math:`\operatorname{atan2}(\sin(\cdot), \cos(\cdot))` step guarantees the output range is always
:math:`[-\pi, \pi]`.

When the Sun direction is (nearly) aligned with the drive axis, its projection onto the :math:`(\hat{\mathbf a}_2,
\hat{\mathbf a}_3)` plane vanishes and :math:`\theta_R` is undefined. The module detects this via the
``alignmentThreshold`` check and falls back to the reference angle :math:`\theta_R^-` from the previous update,
without applying the offset (the offset is meaningful only against the Sun-tracking solution). The array therefore
stays where the last valid solution put it, instead of rotating to an arbitrary angle.

Retained Reference Angle
^^^^^^^^^^^^^^^^^^^^^^^^
At the end of every update, in both tracking modes, the module stores the wrapped output :math:`\theta_R` as
:math:`\theta_R^-`. This is the only runtime state the module holds, and step 3 is its only consumer. The stored
value is always in :math:`[-\pi, \pi]`, because the module stores the output after the wrap.

The state starts at zero, so an aligned Sun on the first update gives :math:`\theta_R = 0`. ``reset()`` builds a
new algorithm and thus returns the state to zero. ``reInitialize()`` returns the state to zero without a rebuild.

Specified Angle Mode
^^^^^^^^^^^^^^^^^^^^
In ``SPECIFIED_ANGLE`` mode the module bypasses the Sun-tracking computation entirely and returns the
user-supplied ``specifiedArrayAngle``. The module does not add the offset angle, because the offset is meaningful
only against the Sun-tracking solution. The module still applies the final :math:`[-\pi, \pi]` wrap, so:

.. math::

   \theta_R = \operatorname{atan2}\!\left( \sin(\theta_{\text{specified}}),\, \cos(\theta_{\text{specified}}) \right)

Because ``specifiedArrayAngle`` is validated to :math:`[-\pi, \pi]`, the wrap does not change the value.

This mode is useful for parking the array at a known orientation (e.g. during slews or eclipse) without depending
on a valid Sun direction measurement.
