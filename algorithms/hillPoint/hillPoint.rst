Executive Summary
-----------------

The Hill Point attitude guidance module computes the reference attitude, angular rate, and angular acceleration
needed to align the spacecraft body frame with the orbital Hill frame :math:`\mathcal{H}` of the spacecraft about a
primary celestial body. It outputs the MRP attitude :math:`\boldsymbol{\sigma}_{R/N}`, along with the angular rate
:math:`{}^{\mathcal{N}}\boldsymbol{\omega}_{R/N}` and angular acceleration
:math:`{}^{\mathcal{N}}\dot{\boldsymbol{\omega}}_{R/N}`, with the latter two expressed in inertial-frame components.

The orbit can be any Keplerian motion -- circular, elliptical, parabolic, or hyperbolic. The primary celestial body's
inertial state is optional; when not connected the body is treated as fixed at the inertial origin.

This is the FP32 port of the Xmera ``hillPointCpp`` module. Position and velocity inputs remain double-precision
to preserve orbit-scale accuracy; the attitude, rate, and acceleration outputs are single-precision (FP32).

Hill Frame Definition
~~~~~~~~~~~~~~~~~~~~~~

The Hill reference frame takes the spacecraft's orbital plane as the principal one and has its origin at the
center of the spacecraft. It is defined by the right-handed set of axes
:math:`\mathcal{H}: \{\hat{\boldsymbol{\imath}}_r, \hat{\boldsymbol{\imath}}_\theta, \hat{\boldsymbol{\imath}}_h\}`,
where

* :math:`\hat{\boldsymbol{\imath}}_r` points radially outward, in the direction connecting the center of the
    primary body to the spacecraft;
* :math:`\hat{\boldsymbol{\imath}}_h` is normal to the orbital plane, in the direction of the orbit angular
    momentum;
* :math:`\hat{\boldsymbol{\imath}}_\theta` completes the right-handed triad.

.. _fig1_hillPoint:

.. figure:: _Documentation/Figures/hillPointFig1.jpg
    :width: 60%
    :align: center

    Illustration of the Hill orbit frame :math:`\mathcal{H}: \{\hat{\boldsymbol{\imath}}_r,
    \hat{\boldsymbol{\imath}}_\theta, \hat{\boldsymbol{\imath}}_h\}` and the inertial frame
    :math:`\mathcal{N}: \{\hat{n}_1, \hat{n}_2, \hat{n}_3\}`, showing the spacecraft position
    :math:`{}^N\boldsymbol{r}_{B/N}`, the primary body position :math:`{}^N\boldsymbol{r}_{P/N}`,
    and the relative position :math:`\boldsymbol{r} = {}^N\boldsymbol{r}_{B/P}`.

Module Architecture
-------------------

The module is split into two layers:

- The **adapter** (``hillPoint.h``/``.cpp``) is the SysModel-derived class that handles message I/O, validates
  that the required input is connected at ``reset()`` time, and constructs the algorithm via two-phase
  initialization.
- The **algorithm** (``hillPointAlgorithm.h``/``.cpp``) is a pure C++ class with no framework dependencies. It
  takes position/velocity inputs, computes the reference attitude, rate, and acceleration, and returns a
  payload struct as output.

A pure-C shim (``hillPointAlgorithm_c.h``/``.cpp``) wraps the algorithm class for use by Ada/Adamant components
via ``extern "C"`` bindings.


Adapter Layer
---------------------------

The adapter inherits from ``SysModel``. It owns the input / output message hooks, validates that the required
input is connected at ``reset()`` time, then constructs the algorithm via the two-phase init pattern.

.. list-table:: Module I/O Messages
    :widths: 25 30 45
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - ``transNavInMsg``
      - :ref:`NavTransMsgF32Payload`
      - spacecraft inertial translational state (required)
    * - ``celBodyInMsg``
      - :ref:`EphemerisMsgF32Payload`
      - primary celestial body inertial state (optional; defaults to origin if not connected)
    * - ``attRefOutMsg``
      - :ref:`AttRefMsgF32Payload`
      - Hill-frame reference attitude / rate / acceleration

Configuration
~~~~~~~~~~~~~

``HillPointAlgorithm`` has no tunable parameters or configuration; all inputs come from the module messages.

.. list-table:: Configuration parameters
    :widths: 25 25 50
    :header-rows: 1

    * - Parameter
      - Valid range
      - Description
    * - *(none)*
      - --
      - The Hill Point algorithm has no tunable parameters.

Two-Phase Initialization
~~~~~~~~~~~~~~~~~~~~~~~~

The Python usage follows the standard adapter lifecycle: subscribe inputs, call ``reset()`` once, then drive
``updateState()`` each cycle. ::

    module = hillPointF32.HillPoint()
    module.transNavInMsg.subscribeTo(nav_msg)
    module.celBodyInMsg.subscribeTo(cel_body_msg)  # optional

    sim.AddModelToTask(task_name, module)
    sim.InitializeSimulation()
    sim.ExecuteSimulation()

If ``transNavInMsg`` has not been connected when ``reset()`` runs, an ``std::invalid_argument`` is thrown.
If ``updateState()`` is called before ``reset()``, an ``XmeraLifecycleException`` is thrown.

Mathematical Formulation
------------------------

The output reference frame :math:`\mathcal{R}` is taken to coincide with the orbital Hill frame
:math:`\mathcal{H}: \{ \hat{\boldsymbol{\imath}}_r, \hat{\boldsymbol{\imath}}_\theta, \hat{\boldsymbol{\imath}}_h \}`
defined above.

Relative State
~~~~~~~~~~~~~~

Given the spacecraft inertial state :math:`({}^N\boldsymbol{r}_{B/N}, {}^N\boldsymbol{v}_{B/N})` and the primary
body inertial state :math:`({}^N\boldsymbol{r}_{P/N}, {}^N\boldsymbol{v}_{P/N})`, define the spacecraft position
and velocity relative to the primary body as

.. math::

   \boldsymbol{r} \equiv {}^N\boldsymbol{r}_{B/P} = {}^N\boldsymbol{r}_{B/N} - {}^N\boldsymbol{r}_{P/N}, \qquad
   \boldsymbol{v} \equiv {}^N\boldsymbol{v}_{B/P} = {}^N\boldsymbol{v}_{B/N} - {}^N\boldsymbol{v}_{P/N},

When ``celBodyInMsg`` is not connected, :math:`{}^N\boldsymbol{r}_{P/N}` and :math:`{}^N\boldsymbol{v}_{P/N}`
default to zero.

Frame Construction
~~~~~~~~~~~~~~~~~~

.. math::

   \hat{\boldsymbol{\imath}}_r = \frac{\boldsymbol{r}}{r}, \qquad
   \hat{\boldsymbol{\imath}}_h
       = \frac{\boldsymbol{r} \times \boldsymbol{v}}{\| \boldsymbol{r} \times \boldsymbol{v} \|}, \qquad
   \hat{\boldsymbol{\imath}}_\theta = \hat{\boldsymbol{\imath}}_h \times \hat{\boldsymbol{\imath}}_r.

The DCM from inertial to reference frame is then

.. math::

   [RN] = \begin{bmatrix}
      \hat{\boldsymbol{\imath}}_r^T \\
      \hat{\boldsymbol{\imath}}_\theta^T \\
      \hat{\boldsymbol{\imath}}_h^T
   \end{bmatrix},

and the corresponding MRP attitude set is :math:`\boldsymbol{\sigma}_{R/N} = \mathrm{C2MRP}([RN])`.

Angular Rate and Acceleration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Because :math:`\mathcal{R}` is rigidly attached to :math:`\mathcal{H}`, its rate and acceleration with respect to
the inertial frame are entirely along :math:`\hat{\boldsymbol{\imath}}_h`:

.. math::

   {}^{\mathcal{R}}\boldsymbol{\omega}_{R/N} = \begin{bmatrix} 0 \\ 0 \\ \dot{f} \end{bmatrix}, \qquad
   {}^{\mathcal{R}}\dot{\boldsymbol{\omega}}_{R/N} = \begin{bmatrix} 0 \\ 0 \\ \ddot{f} \end{bmatrix},

where :math:`f` is the true anomaly. For unperturbed two-body (Keplerian) motion, the specific angular momentum
magnitude :math:`h = \| \boldsymbol{r} \times \boldsymbol{v} \|` is conserved. Writing the velocity in polar
components about the orbital plane, :math:`\boldsymbol{v} = \dot{r}\,\hat{\boldsymbol{\imath}}_r +
r\dot{f}\,\hat{\boldsymbol{\imath}}_\theta`, so that

.. math::

   \boldsymbol{r} \times \boldsymbol{v} = r\,\hat{\boldsymbol{\imath}}_r \times
       \left( \dot{r}\,\hat{\boldsymbol{\imath}}_r + r\dot{f}\,\hat{\boldsymbol{\imath}}_\theta \right)
       = r^2 \dot{f}\, \hat{\boldsymbol{\imath}}_h,
   \qquad \Rightarrow \qquad
   \dot{f} = \frac{h}{r^2}.

Differentiating and using that :math:`h` is constant along the orbit,

.. math::

   \ddot{f} = \frac{d}{dt}\!\left( \frac{h}{r^2} \right) = -\frac{2h\dot{r}}{r^3}
     = -2 \frac{\dot{r}}{r} \dot{f},

and since :math:`\dot{r} = \boldsymbol{v} \cdot \hat{\boldsymbol{\imath}}_r` (the radial component of velocity),

.. math::

   \ddot{f} = -2 \frac{\boldsymbol{v} \cdot \hat{\boldsymbol{\imath}}_r}{r} \dot{f}.

The outputs are then rotated into inertial-frame components via :math:`[NR] = [RN]^T`:

.. math::

   {}^{\mathcal{N}}\boldsymbol{\omega}_{R/N} = [NR] \, {}^{\mathcal{R}}\boldsymbol{\omega}_{R/N}, \qquad
   {}^{\mathcal{N}}\dot{\boldsymbol{\omega}}_{R/N} = [NR] \, {}^{\mathcal{R}}\dot{\boldsymbol{\omega}}_{R/N}.

Robustness
~~~~~~~~~~

Three degenerate-geometry cases are guarded against, each leaving the attitude, rate, and acceleration outputs at
zero rather than dividing by zero or propagating non-finite values:

* the relative orbital radius :math:`r` falls below the :math:`1.0\,\mathrm{m}` robustness threshold, protecting
  against the :math:`r \to 0` singularity. Note that the original Xmera comment described this threshold as
  "1 km" but the value is :math:`1.0` in the same units as :math:`{}^N\boldsymbol{r}_{B/N}`, which is meters;
  the FP32 port preserves the original numerical behavior.
* the relative velocity :math:`\boldsymbol{v}` is exactly zero. Eigen normalizes a zero vector to zero rather
  than producing NaN, so without this explicit check a zero :math:`\boldsymbol{v}` would silently compute a
  spurious well-defined (but meaningless) 90-degree separation angle instead of triggering the collinearity
  guard below.
* the relative position and velocity vectors are (nearly) collinear, i.e. the angle between
  :math:`\hat{\boldsymbol{\imath}}_r` and the line along :math:`\boldsymbol{v}` -- computed as
  :math:`\arccos(|\hat{\boldsymbol{\imath}}_r \cdot \hat{\boldsymbol{v}}|)`, so both parallel and antiparallel
  :math:`\boldsymbol{r}`/:math:`\boldsymbol{v}` count as collinear -- is below :math:`1.0 \times 10^{-3}\,\mathrm{rad}`.
  In this case :math:`\boldsymbol{r} \times \boldsymbol{v} \to 0`, leaving the orbit normal
  :math:`\hat{\boldsymbol{\imath}}_h` undefined, therefore the Hill frame is undefined.

Assumptions and Limitations
---------------------------

* Position and velocity inputs are read in double precision to preserve orbit-scale accuracy. The attitude,
  rate, and acceleration outputs are single-precision; expect a relative accuracy of :math:`\sim 10^{-5}` on
  :math:`\boldsymbol{\sigma}_{R/N}`, :math:`\boldsymbol{\omega}_{R/N}`, and :math:`\dot{\boldsymbol{\omega}}_{R/N}`
  against the double-precision reference implementation.
* The angular acceleration :math:`\ddot{f}` assumes :math:`h` stays constant. For a perturbed or thrusting
  trajectory, :math:`\dot{f}` is still valid, but :math:`\ddot{f}` does not account for changes in
  :math:`h` (the :math:`\dot{h}/r^2` term).
* The module assumes ``transNavInMsg`` and ``celBodyInMsg`` are time synchronized and no time alignment is performed
  between the two inputs, so any timing difference may introduce errors in the relative position and velocity used
  to construct the Hill frame.
* ``celBodyInMsg`` is optional and the module does not distinguish between this being intentional or a missing
  connection, so the expected message connections should be verified during integration.

Test Description
-----------------

The module is verified through regression tests that compare the algorithm results against a
double-precision reference implementation. A setup test checks that the algorithm constructs
without throwing. Fuzz tests are added for the regression and property tests, where the spacecraft and
primary-body position/velocity are randomized over physically reasonable ranges (bounded by heliosphere-scale
distances and solar-system-scale velocities).

Regression Tests
^^^^^^^^^^^^^^^^^

- ReferenceTestPlanetAtOrigin
    - Checks the algorithm output against the double-precision reference implementation for a
      circular equatorial orbit with the primary body fixed at the inertial origin.

- ReferenceTestPlanetOffset
    - Checks the algorithm output against the reference implementation when the primary body itself has a
      nonzero inertial position and velocity.

- ``CircularOrbit``, ``EllipticalOrbit``, ``ParabolicOrbit``, and ``HyperbolicOrbit``
    - Verify the Hill-frame attitude, angular velocity, and angular acceleration against
      analytical conic-orbit relations for eccentricities :math:`e = 0.0`, :math:`0.3`,
      :math:`1.0`, and :math:`1.5`, respectively.
    - The spacecraft state is constructed from the semi-latus rectum, eccentricity, and
      true anomaly using the radial and tangential velocity components.
    - The non-circular cases have nonzero radial velocity, allowing :math:`\ddot{f}` to be
      verified against a nonzero analytical value derived from the conic-orbit relations.

Property Tests
^^^^^^^^^^^^^^

- OutputIsFinite
    - Checks that all output components are finite for valid inputs.

- SigmaNormBounded
    - Checks that the output MRP is bounded by 1 (inner MRP set) for any inputs.

Edge Case Tests
^^^^^^^^^^^^^^^

- BelowThresholdRadius
    - Checks that the algorithm returns zero attitude, rate, and acceleration when the relative orbital radius is
      below the robustness threshold (1m).

- BelowSmallAngleThreshold
    - Checks that the algorithm returns zero attitude, rate, and acceleration when the relative position and
      velocity are collinear (:math:`\boldsymbol{h} = \boldsymbol{r} \times \boldsymbol{v} \approx \boldsymbol{0}`),
      leaving the orbital plane undefined.

- ZeroVelocityAtValidRadius
    - Checks that the algorithm returns zero attitude, rate, and acceleration when the relative velocity is
      exactly zero, even though the orbital radius itself is valid.
