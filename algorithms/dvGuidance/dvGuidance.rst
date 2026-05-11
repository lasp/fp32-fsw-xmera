Executive Summary
-----------------

The dvGuidance module produces a time-varying attitude reference frame for an orbit-correction delta-V burn whose
direction rotates at a constant rate about a designated body axis. The output is the MRP attitude
:math:`\boldsymbol{\sigma}_{R/N}`, the angular rate :math:`\boldsymbol{\omega}_{R/N}`, and the angular acceleration
:math:`\dot{\boldsymbol{\omega}}_{R/N}` of the reference frame :math:`\mathcal{R}` with respect to the inertial frame
:math:`\mathcal{N}`, all in inertial-frame components.

The reference frame is constructed in two stages: first a base burn frame :math:`\mathcal{B}_{u,b}` aligned with the
commanded delta-V direction, then a current burn frame :math:`\mathcal{B}_{u,t}` obtained by rotating
:math:`\mathcal{B}_{u,b}` about its 3rd axis at a constant rate. The resulting reference attitude makes the spacecraft
1st body axis track the rotating delta-V direction while the rest of the body precesses about it.

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

``DvGuidanceAlgorithm`` has no tunable parameters. The empty ``DvGuidanceConfig`` class is provided to keep the
algorithm consistent with the standard two-phase init pattern in this codebase.

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

A seed unit vector :math:`\hat{\boldsymbol{r}}` (``dvRotVecUnit``) is supplied to define the rotation axis. The
remaining base axes are

.. math::

   \hat{\boldsymbol{b}}_{u_b,2} = \frac{\hat{\boldsymbol{r}} \times \Delta\boldsymbol{v}}
                                       {\| \hat{\boldsymbol{r}} \times \Delta\boldsymbol{v} \|}, \qquad
   \hat{\boldsymbol{b}}_{u_b,3} = \frac{\hat{\boldsymbol{b}}_{u_b,1} \times \hat{\boldsymbol{b}}_{u_b,2}}
                                       {\| \hat{\boldsymbol{b}}_{u_b,1} \times \hat{\boldsymbol{b}}_{u_b,2} \|}.

When :math:`\hat{\boldsymbol{r}}` is already orthogonal to :math:`\Delta\boldsymbol{v}`, the construction above
yields :math:`\hat{\boldsymbol{b}}_{u_b,3} = \hat{\boldsymbol{r}}` -- i.e., :math:`\hat{\boldsymbol{r}}` is the axis
about which :math:`\Delta\boldsymbol{v}` rotates during the burn.

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

Assumptions and Limitations
---------------------------

* The commanded delta-V direction is assumed to drive the spacecraft's downstream body 1 axis. Pair this module with
  :ref:`attTrackingError` (or analogous) when a different body axis must align with :math:`\mathcal{R}`.
* :math:`\hat{\boldsymbol{r}}` (``dvRotVecUnit``) must not be parallel to :math:`\Delta\boldsymbol{v}`; otherwise
  :math:`\hat{\boldsymbol{r}} \times \Delta\boldsymbol{v}` collapses to zero and the normalize step propagates
  ``NaN`` through the output. The module does **not** guard against this case (it preserves the original Xmera
  semantics).
* :math:`\Delta\boldsymbol{v}` must have nonzero norm. A zero-magnitude commanded delta-V causes the same NaN
  propagation through the first normalize step.
* :math:`\dot\theta` is constant for the entire burn; the module does not support time-varying rotation rates.
* All math is single-precision (FP32). For burns with very small :math:`\dot\theta\,\Delta t` rotations, numerical
  noise on :math:`[B_{u,t}B_{u,b}]` can dominate the small-angle deviation; downstream consumers should apply their
  own threshold logic if needed.
