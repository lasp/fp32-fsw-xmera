Executive Summary
-----------------

The dvGuidance module produces a time-varying attitude reference frame for an orbit-correction delta-V burn. The
reference is constructed from the commanded delta-V direction and a rotation-axis seed, then rotated at a constant
rate about the resulting 3rd burn-frame axis. The output is the MRP attitude
:math:`\boldsymbol{\sigma}_{R/N}`, the angular rate :math:`\boldsymbol{\omega}_{R/N}`, and the angular acceleration
:math:`\dot{\boldsymbol{\omega}}_{R/N}` of the reference frame :math:`\mathcal{R}` with respect to the inertial frame
:math:`\mathcal{N}`, all in inertial-frame components.

The reference frame is constructed in two stages: first a base burn frame :math:`\mathcal{B}_{u,b}` aligned with the
commanded delta-V direction, then a current burn frame :math:`\mathcal{B}_{u,t}` obtained by rotating
:math:`\mathcal{B}_{u,b}` about its 3rd axis at a constant rate.

This is the FP32 port of the Xmera ``dvAttGuidance`` module. Inputs and outputs are single-precision (FP32); the
algorithm is single-precision throughout.

Module Architecture
-------------------

The module is split into a thin adapter (``DvGuidance``) that handles framework integration and an algorithm class
(``DvGuidanceAlgorithm``) that contains the pure math.

Adapter Layer
~~~~~~~~~~~~~

The adapter inherits from ``SysModel``. It owns the input / output message hooks, validates that the required input
is connected at ``reset()`` time, then constructs the algorithm via the two-phase init pattern.

.. list-table:: Module I/O Messages
    :widths: 25 30 45
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - ``burnDataInMsg``
      - :ref:`DvBurnCmdMsgF32Payload`
      - Commanded delta-V direction, rotation seed axis, rotation rate, and burn start time.
    * - ``attRefOutMsg``
      - :ref:`AttRefMsgF32Payload`
      - Reference attitude / rate / acceleration of :math:`\mathcal{R}` relative to :math:`\mathcal{N}`.

Configuration
~~~~~~~~~~~~~

``DvGuidanceAlgorithm`` has no tunable parameters or configuration; all inputs come from ``burnDataInMsg``.

.. list-table:: Configuration parameters
    :widths: 25 25 50
    :header-rows: 1

    * - Parameter
      - Valid range
      - Description
    * - *(none)*
      - --
      - The dvGuidance algorithm has no tunable parameters; all inputs come from ``burnDataInMsg``.

Two-Phase Initialization
~~~~~~~~~~~~~~~~~~~~~~~~

The Python usage follows the standard adapter lifecycle: subscribe inputs, call ``reset()`` once, then drive
``updateState()`` each cycle. ::

    module = dvGuidanceF32.DvGuidance()
    module.burnDataInMsg.subscribeTo(burn_in_msg)

    sim.AddModelToTask(task_name, module)
    sim.InitializeSimulation()
    sim.ExecuteSimulation()

If ``burnDataInMsg`` has not been connected when ``reset()`` runs, an ``std::invalid_argument`` is thrown.
If ``updateState()`` is called before ``reset()``, an ``XmeraLifecycleException`` is thrown.

Mathematical Formulation
------------------------

All vector components in this section are taken in a common inertial frame :math:`\mathcal{N}`.

Base Burn Frame
~~~~~~~~~~~~~~~

Let :math:`\mathcal{B}_{u,b} = \{\hat{\boldsymbol{b}}_{u_b,1}, \hat{\boldsymbol{b}}_{u_b,2}, \hat{\boldsymbol{b}}_{u_b,3}\}`
be the inertially fixed base burn frame. Its DCM relative to :math:`\mathcal{N}` is

.. math::

   [B_{u,b}N] = \begin{bmatrix}
       \hat{\boldsymbol{b}}_{u_b,1}^T \\
       \hat{\boldsymbol{b}}_{u_b,2}^T \\
       \hat{\boldsymbol{b}}_{u_b,3}^T
   \end{bmatrix}.

The first base axis aligns with the commanded delta-V direction:

.. math::

   \hat{\boldsymbol{b}}_{u_b,1} = \frac{\Delta\boldsymbol{v}}{\| \Delta\boldsymbol{v} \|}.

A seed vector :math:`\boldsymbol{r}` (``dvRotVecUnit``) is normalized to
:math:`\hat{\boldsymbol{r}}` and used to construct the remaining burn-frame axes.
The resulting 3rd burn-frame axis is the component of the seed direction orthogonal to the
commanded delta-V direction. When the seed is already orthogonal to
:math:`\Delta\boldsymbol{v}`, :math:`\hat{\boldsymbol{b}}_{u_b,3} = \hat{\boldsymbol{r}}`.

.. math::

   \hat{\boldsymbol{b}}_{u_b,2} = \frac{\hat{\boldsymbol{r}} \times \hat{\boldsymbol{b}}_{u_b,1}}{\| \hat{\boldsymbol{r}}
   \times \hat{\boldsymbol{b}}_{u_b,1} \|}, \qquad
   \hat{\boldsymbol{b}}_{u_b,3} = \frac{\hat{\boldsymbol{b}}_{u_b,1} \times \hat{\boldsymbol{b}}_{u_b,2}}
                                       {\| \hat{\boldsymbol{b}}_{u_b,1} \times \hat{\boldsymbol{b}}_{u_b,2} \|}.

Burn Time
~~~~~~~~~

The burn command provides a start time :math:`t_{\text{start}}` (``burnStartTime``). The elapsed burn time at the
current call time :math:`t` is

.. math::

   \Delta t = t - t_{\text{start}}.

A negative :math:`\Delta t` is valid: prior to the burn start, the reference frame rotates *toward* the nominal
attitude that will hold at :math:`t = t_{\text{start}}`.

Current Burn Frame
~~~~~~~~~~~~~~~~~~

The current burn frame :math:`\mathcal{B}_{u,t}` is obtained by rotating :math:`\mathcal{B}_{u,b}` about its 3rd axis
by

.. math::

   \theta(t) = \dot\theta\, \Delta t,

where :math:`\dot\theta` is ``dvRotVecMag``. The DCM from base to current burn frame is

.. math::

   [B_{u,t}B_{u,b}] = \begin{bmatrix}
       \hphantom{-}\cos\theta & \sin\theta & 0 \\
       -\sin\theta & \cos\theta & 0 \\
       0 & 0 & 1
   \end{bmatrix},

and the inertial-to-current-burn-frame DCM is

.. math::

   [RN] = [B_{u,t}N] = [B_{u,t}B_{u,b}] [B_{u,b}N].

The implementation builds :math:`[B_{u,t}B_{u,b}]` via the principal-rotation-vector form
:math:`\boldsymbol{\Phi} = (0,\,0,\,\theta)` and the standard PRV-to-DCM map. The MRP output is

.. math::

   \boldsymbol{\sigma}_{R/N} = \mathrm{C2MRP}\!\left([RN]\right).

Angular Rate and Acceleration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Because :math:`\mathcal{B}_{u,b}` is inertially fixed, :math:`\boldsymbol{\omega}_{B_{u,b}/N} = \boldsymbol{0}`. The
relative rate between the current and base burn frames is

.. math::

   \boldsymbol{\omega}_{B_{u,t}/B_{u,b}} = \dot\theta\, \hat{\boldsymbol{b}}_{u_b,3}
                                         = \dot\theta\, \hat{\boldsymbol{b}}_{u_t,3},

since :math:`\hat{\boldsymbol{b}}_{u_b,3}` and :math:`\hat{\boldsymbol{b}}_{u_t,3}` coincide (the rotation is about
that axis). The reference rate is therefore

.. math::

   \boldsymbol{\omega}_{R/N} = \boldsymbol{\omega}_{B_{u,t}/N} = \dot\theta\, \hat{\boldsymbol{b}}_{u_t,3}.

In code, :math:`\hat{\boldsymbol{b}}_{u_t,3}` is read from the third row of :math:`[RN]`.

Because :math:`\dot\theta` is constant and :math:`\hat{\boldsymbol{b}}_{u_t,3}` is inertially fixed, the angular
acceleration is identically zero:

.. math::

   \dot{\boldsymbol{\omega}}_{R/N} = \boldsymbol{0}.

Robustness
~~~~~~~~~~

Three invalid-input or degenerate-geometry cases are guarded against, each leaving the attitude, rate, and
acceleration outputs at the safe default (identity attitude with zero rates):

* A non-finite or near-zero :math:`\Delta\boldsymbol{v}` does not define a valid burn direction. The algorithm
  returns the safe default when the input is non-finite or when
  :math:`\lVert\Delta\boldsymbol{v}\rVert^2` falls below ``kMinNormSq``.

* A non-finite or degenerate rotation-axis seed cannot be used to complete the burn frame. If the seed is zero, or
  its direction is sufficiently close to parallel or antiparallel with
  :math:`\hat{\boldsymbol{b}}_{u_b,1}`, the cross product used to construct
  :math:`\hat{\boldsymbol{b}}_{u_b,2}` collapses. The algorithm returns the safe default when the squared magnitude
  of this cross product falls below ``kMinCrossSq``.

* A non-finite rotation rate :math:`\dot\theta` (``dvRotVecMag``) returns the safe default.

For otherwise valid inputs, rotation magnitudes below ``kSmallAngle`` are handled separately. When
:math:`\lvert\dot\theta\,\Delta t\rvert <` ``kSmallAngle``, the incremental rotation is not applied and the
base burn-frame attitude is returned, while :math:`\boldsymbol{\omega}_{R/N}` still reflects the commanded rate.
This introduces a bounded kinematic inconsistency between the reported attitude and angular rate while the
incremental rotation remains below ``kSmallAngle``.

Assumptions and Limitations
---------------------------

* ``dvRotVecMag`` is constant over the burn interval; time-varying rotation rates are not supported.
* ``dvRotVecUnit`` is used as a seed to construct the burn frame rather than as an independently enforced rotation
  axis. Its component parallel to the commanded delta-V direction does not affect the resulting 3rd burn-frame axis.
* The algorithm operates in single precision (FP32).

Numerical conditioning
----------------------

``kMinCrossSq`` bounds the FP32 reference error rather than being an arbitrary cutoff. The base-frame
construction normalizes :math:`\hat{\boldsymbol{r}} \times \hat{\boldsymbol{b}}_{u_b,1}`, whose magnitude is
:math:`\sin(\text{angle})`; this amplifies the relative round-off by :math:`\sim 1/\sin(\text{angle})`. With a
float epsilon of :math:`\sim 1.2\times10^{-7}` and the additional amplification of the second Gram-Schmidt
normalization, the matrix product, and the DCM\ :math:`\leftrightarrow`\ MRP round-trip, the per-element DCM error
reaches :math:`\sim 10^{-5}` near :math:`\sin(\text{angle}) \approx 1.7\times10^{-2}`. ``kMinCrossSq`` =
:math:`9\times10^{-4}` (i.e. :math:`\sin(\text{angle}) \approx 3\times10^{-2}`, about :math:`1.7^\circ`) is the
boundary below which the frame is no longer trusted to FP32 reference accuracy; the unit tests verify the algorithm
matches a double-precision reference to :math:`10^{-5}` only outside this guard region.

``kMinNormSq`` = :math:`10^{-12}` (\ :math:`\sim 10^{-6}` m/s) is not a derived round-off bound -- it is a floor
below which the commanded delta-V is treated as effectively zero and the burn direction as undefined.

``kSmallAngle`` = :math:`10^{-5}` rad is the threshold below which the incremental burn-frame rotation is not applied.

Test Description
-----------------

The module is verified through regression tests that compare the algorithm results against a double-precision
reference implementation, property tests, and edge-case tests covering degenerate and non-finite inputs. Fuzz
tests are added for the regression and property tests, randomizing the burn-command inputs and elapsed burn
time over physically reasonable ranges.

Regression Tests
^^^^^^^^^^^^^^^^^

- ReferenceTestAtBurnStart
    - Checks the algorithm output against the double-precision reference implementation sampled at the burn
      start time, where the incremental rotation is zero.
- ReferenceTestMidBurn
    - Checks the algorithm output against the reference implementation partway through the burn, where the
      incremental rotation is positive.
- ReferenceTestPrelaunch
    - Checks the algorithm output against the reference implementation before the burn start time, where the
      incremental rotation is negative.

Guidance Behavior Tests
^^^^^^^^^^^^^^^^^^^^^^^

- ZeroRotationRate
    - Checks that a zero commanded rotation rate leaves the reference attitude fixed and the reported rate and
      acceleration at zero, regardless of elapsed time.
- AngularVelocityMagnitudeMatchesDvRotVecMag
    - Checks that the magnitude of the reported angular rate equals the commanded rotation rate.
- BelowSmallAngleThresholdUsesBaseAttitude
    - Checks that a rotation magnitude below ``kSmallAngle`` leaves the reported attitude at the base burn-frame
      attitude rather than advancing it.

Property Tests
^^^^^^^^^^^^^^

- OutputIsFinite
    - Checks that all output components are finite for valid inputs.
- SigmaNormBounded
    - Checks that the output MRP norm remains bounded by 1 (inner MRP set).

Input Validation Tests
^^^^^^^^^^^^^^^^^^^^^^

- ZeroDeltaVCommand, RotAxisParallelToDeltaV, ZeroRotationAxis, RotAxisAntiParallelToDeltaV
    - Check that the safe default (identity attitude, zero rates) is returned for a zero delta-V command, and
      for a rotation-axis seed that is parallel, zero, or antiparallel to the commanded delta-V direction.
- InfiniteDvInrtlCmdReturnsDefault, InfiniteDvRotVecUnitReturnsDefault, InfiniteRotationRateReturnsDefault
    - Check that the safe default is returned when the delta-V command, rotation-axis seed, or rotation rate,
      respectively, is non-finite.
- DeltaVNormBoundary
    - Checks that a delta-V command exactly at the ``kMinNormSq`` threshold is accepted, while the next
      representable magnitude below it returns the safe default.
- CrossBoundary
    - Checks that a rotation-axis seed exactly at the ``kMinCrossSq`` threshold is accepted, while the next
      representable seed below it returns the safe default.
