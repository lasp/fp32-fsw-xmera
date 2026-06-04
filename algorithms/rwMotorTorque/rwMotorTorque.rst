.. raw:: latex

    {\LARGE \textbf{rwMotorTorque}}

Executive Summary
-----------------

This module maps a desired torque to control the spacecraft, and maps it to the available reaction wheels using a
minimum norm inverse fit. All numeric computation uses single-precision (``float`` / fp32).

The optional wheel availability message is used to include or exclude particular reaction wheels from the torque
solution. The desired control torque can be mapped onto particular orthogonal control axes to implement a partial
solution for the overall attitude control torque.

The module also provides an optional reaction-wheel null-space despin term that drives the wheel speeds toward a
desired set without producing any net body torque. This is enabled by a non-zero ``omegaGain`` together with the
optional RW speed messages; with the default zero gain (or no speed message linked) the module reduces to the pure
control-torque mapping.

The module follows the standard three-layer pattern: the pure ``RwMotorTorqueAlgorithm`` (Eigen-typed, no messaging),
the ``RwMotorTorque`` Xmera adapter (owns message I/O and payload conversion), and a C-linkage shim
(``rwMotorTorqueAlgorithm_c.*``) exposing the algorithm to Ada via FFI. The control axes mapping matrix is supplied as
a two-phase configuration property and validated when ``reset()`` builds the algorithm configuration.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg connection is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - rwMotorTorqueOutMsg
      - :ref:`RwMotorTorqueMsgPayload`
      - RW motor torque output message
    * - vehControlInMsg
      - :ref:`CmdTorqueBodyMsgPayload`
      - commanded vehicle control torque input message
    * - vehControlIn2Msg
      - :ref:`CmdTorqueBodyMsgPayload`
      - (optional) additional commanded vehicle control torque input message
    * - rwParamsInMsg
      - :ref:`RWArrayConfigMsgPayload`
      - RW array configuration input message
    * - rwAvailInMsg
      - :ref:`RWAvailabilityMsgPayload`
      - (optional) RW device availability message
    * - rwSpeedsInMsg
      - :ref:`RWSpeedMsgPayload`
      - (optional) current RW speeds, used by the null-space despin term
    * - rwDesiredSpeedsInMsg
      - :ref:`RWSpeedMsgPayload`
      - (optional) desired RW speeds for the null-space despin term (defaults to zero)

Module Parameters
-------------------------------
The following table lists all the module parameters than can be set. The parameters are optional unless indicated
(if not specified default is used).

.. list-table:: Module Parameters
    :widths: 30 30 10 10 30 30
    :header-rows: 1

    * - Parameter Name
      - Type
      - Units
      - Default
      - Description
      - Bounds
    * - controlAxes_B (required)
      - Eigen::Matrix3f
      - [-]
      - :math:`[0]_{3x3}`
      - array of the control unit axes (axis in each row)
      - N/A
    * - omegaGain
      - float
      - [-]
      - :math:`0`
      - RW null-space despin feedback gain; ``0`` disables the despin term
      - :math:`\ge 0`

Model Assumptions and Limitations
---------------------------------
This code makes the following assumptions:

- The control mapping matrix :math:`[CB][G_{s}]` (constructed using only the available wheels) must be well conditioned:
  every control axis must be reachable by the available wheels, and the smallest-to-largest singular value ratio must
  exceed :math:`10^{-2}` (condition number below 100). A rank-deficient or ill-conditioned mapping would amplify the
  command and single-precision round-off, so such a configuration is rejected with an invalid argument exception when
  the configuration is created.
- It is assumed that the availability of the reaction wheels is known at the time of reset, and does not change after
  that.
- The control axes and RW spin axes are expected to be unit vectors. They are validated when the configuration is
  created (a non-unit spin axis is rejected) and canonicalized before use: the RW spin axes are normalized and the
  control axes are orthonormalized (Gram-Schmidt).
- The null-space despin term requires more than three *available* reaction wheels spanning 3-D for a non-trivial
  null space. With three (or fewer) available spanning wheels the null space is empty, :math:`[\tau]` is zero, and
  the despin term has no effect. A zero ``omegaGain`` (the default) likewise disables the despin term. The
  available-wheel geometry :math:`[G_{s}]` must also be well conditioned (condition number below 100); a near-coplanar
  array of more than three wheels is rejected when the configuration is created.

Initialization
--------------
The module is configured by::

    module = rwMotorTorque.RwMotorTorque()
    module.modelTag = "rwMotorTorque"
    control_axes_B = [[1, 0, 0], [0, 1, 0], [0, 0, 0]]
    module.controlAxes_B = control_axes_B

Detailed Model Description
--------------------------

Module Input and Output Behavior
================================
This module takes an attitude control torque
:math:`{}^{\mathcal{B}}{\mathbf L_{r}}` and maps the
vector onto the specific control axes, :math:`\hat{\mathbf c}_{i}`. This
allows only a subset of
:math:`{}^{\mathcal{B}}{\mathbf L_{r}}` to be
implemented with the Reaction Wheels (RWs). The next step is to map
reduced control torque onto the available RW spin
axes,\ :math:`\hat{\mathbf g}_{s_j}`. The module accounts for the
availability of the reaction wheels in the case that not all wheels are
functioning appropriately or are undergoing independent analysis.

Assume in this documentation that the number of available RWs is
:math:`m`, while the number of desired control axes
:math:`\hat{\mathbf c}_{i}` is :math:`n`. The number of installed RWs is
:math:`N`.

The commanded torque message is a required input message and is read in
every time step. The RW configuration message is also required, but only
read in during reset. If the RW spin axes :math:`\hat{\mathbf b}_{s_{i}}`
change then reset() must be called again. The RW availability message is
optional. If the availability message is not used, then all installed
RWs are assumed to be available. The output message is always the array
of RW motor torques.

Torque Mapping
==============
The ``rwMotorTorque`` module receives a desired attitude control torque
in the body frame :math:`{}^{\mathcal{B}}{\mathbf L}
_r`. If two control torques are provided via the second optional control
input message, these two are added together to compute
:math:`{}^{\mathcal{B}}{\mathbf L}
_r`. This torque is the net control torque that should be applied to the
spacecraft. Let :math:`\hat{\mathbf g}_{s_{i}}` be the individual RW spin
axis, while :math:`\mathbf u_{s}` is the :math:`m`-dimensional array of
motor torques. The :math:`3\times m` projection matrix :math:`[G_{s}]`
then maps the control torque on motor torques using

.. math::

   	[G_{s}] \mathbf u_{s} = (-
   	{}^{\mathcal{B}}{\mathbf L}
   _r)

The projection matrix is defined as

.. math::

   	[G_{s}] = \begin{bmatrix}
   		\hat{\mathbf g}_{s_{1}} & \cdots & \hat{\mathbf g}_{s_{m}}
   	\end{bmatrix}

Note that here :math:`\mathbf u_{s}` is the array of available motor
torques. The installed set of RWs could be larger than :math:`m`.

The projection matrix to map a vector in the body frame :math:`\cal B`
onto the set of control axes :math:`\hat{\mathbf c}_{i}` is given by

.. math::

   	[CB] = \begin{bmatrix}
                {}^{\mathcal{B}}{\hat{\mathbf c}}_{1}^{T} \\
   		        \vdots \\
                {}^{\mathcal{B}}{\hat{\mathbf c}}_{n}^{T}
   	        \end{bmatrix}

where :math:`n \le 3`.

To map the requested torque onto the control axes,
the first equation is pre-multifplied by :math:`[CB]` to
yield

.. math::

   	[CB][G_{s}] \mathbf u_{s} = [CG_{s}] \mathbf u_{s} = [CB](-
   	{}^{\mathcal{B}}{\mathbf L}
   _r) =
   	{}^{\mathcal{C}}{\mathbf L}
   _{r}

Note that :math:`[CG_{s}]` is a :math:`n\times m` matrix.

The module assumes that :math:`m\ge n` such that there are enough RWs
available to implement :math:`{}^{\mathcal{C}}{\mathbf L}
_{r}`. If not, then the output motor torques are set to zero with
:math:`\mathbf u_{s} = \mathbf 0`.

To invert the equation above, the minimum-norm solution

.. math::

   \mathbf u_{s}  = [CG]^T \left([CG][CG]^T\right)^{-1}\
   	{}^{\mathcal{C}}{\mathbf L}
   _{r}

is used. It is evaluated through the Moore-Penrose pseudo-inverse of :math:`[CG_{s}]` computed from a singular value
decomposition (a truncated SVD, :math:`[V][\Sigma]^{-1}[U]^{T}`, with singular values below a relative tolerance
treated as zero) rather than by forming and inverting the normal-equation Gram matrix :math:`[CG_{s}][CG_{s}]^{T}`. The
SVD form avoids squaring the condition number and is the same construction used by ``forceTorqueThrForceMapping``. The
controllability and conditioning of :math:`[CG_{s}]` are checked from this same decomposition (see Model Assumptions).

The final step is to map the array of available motor torques
:math:`\mathbf u_{s}` onto the output array of installed RW motor torques.

RW Availability
===============

If the input message ``rwAvailInMsg`` is linked, then the RW
availability message is read in. The torque mapping is only used for
RW’s whose availability setting is ``AVAILABLE``. If it is
``UNAVAILABLE`` then that RW output torque is set to zero.

RW Null-Space Despin
====================
On top of the control-torque solution, the module can add a null-space despin torque that drives the RW speeds
toward a desired set without disturbing the spacecraft attitude. This capability was previously provided by a
separate ``rwNullSpace`` module and is now part of ``rwMotorTorque``.

Let :math:`N` be the number of installed RWs and :math:`\hat{\mathbf g}_{s_{i}}` the :math:`i^{\text{th}}` RW spin
axis. The :math:`3 \times N` spin-axis matrix :math:`[G_{s}]` holds the **available** wheels' spin axes in their
installed columns and zeros for unavailable wheels:

.. math:: [G_{s}] = [\hat{\mathbf g}_{s_{1}} \cdots \hat{\mathbf g}_{s_{N}}]

The null-space projection matrix :math:`[\tau]` is the orthogonal projector onto the null space of :math:`[G_{s}]`,

.. math:: [\tau] = [I_{N\times N}] - [V_{r}][V_{r}]^{T}

where the columns of :math:`[V_{r}]` are the right singular vectors of :math:`[G_{s}]` with non-zero singular values
(a basis for its row space), obtained from a singular value decomposition. Building :math:`[\tau]` from orthonormal
singular vectors makes :math:`[G_{s}][\tau] = [0]` hold to machine precision regardless of conditioning, which avoids
the single-precision torque leakage incurred by the algebraically equivalent normal-equation form
:math:`[I] - [G_{s}]^{T}([G_{s}][G_{s}]^{T})^{-1}[G_{s}]`. By construction any torque produced by :math:`[\tau]` lies
in the RW null space and exerts no net torque on the spacecraft, leaving the attitude control solution unaffected.

Let :math:`\Omega_{i}` be the RW spin speeds and :math:`\Omega_{i,d}` the desired speeds, with tracking error
:math:`\Delta\Omega_i = \Omega_i - \Omega_{i,d}`. The desired despin torque array is

.. math:: \mathbf d = -K \, \Delta\bm\Omega

with feedback gain :math:`K = \texttt{omegaGain} \ge 0`. Mapping it through :math:`[\tau]` and adding it to the
control solution :math:`\mathbf u_{s,\text{cont}}` gives the module output

.. math:: \mathbf u_{s} = \mathbf u_{s,\text{cont}} + [\tau] \, \mathbf d

The projection matrix :math:`[\tau]` is precomputed at reset, and its rows for unavailable wheels are zeroed so
the despin term never commands torque to a wheel the control mapping excluded. A null space exists only when more
than three wheels are available and span 3-D; with three (or fewer) available wheels :math:`[\tau]` is zero and the
despin term vanishes. When more than three wheels are available their geometry must be well conditioned (condition
number below 100); a near-coplanar array is rejected when the configuration is created rather than producing an
unreliable despin. The current and desired RW speeds are read each update from the optional ``rwSpeedsInMsg`` and
``rwDesiredSpeedsInMsg`` (unlinked speeds default to zero).

This availability handling is a deliberate departure from the legacy ``rwNullSpace`` module, which had no concept
of RW availability and built :math:`[\tau]` from all configured wheels. For configurations with all wheels
available the two are identical.

Model Functions
---------------
The code performs the following functions:

- **Maps control torque vector onto available reaction wheels**: Takes a
  desired body-frame torque from ``CmdTorqueBodyIntMsg`` and maps it
  onto the RW axes.

- **Removes torque from unavailable reaction wheels**: The module
  observes the availability of the RWs and maps the torques to only
  available reaction wheels.

- **Adds an optional RW null-space despin torque**: When ``omegaGain`` is
  non-zero, the module superimposes a null-space torque that drives the RW
  speeds toward their desired values without producing any net body torque.
