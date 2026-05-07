Executive Summary
-----------------

The Hill Point attitude guidance module computes a reference attitude that aligns the spacecraft body frame with
the orbital Hill frame :math:`\mathcal{H}` of the spacecraft about a primary celestial body. It outputs the MRP
attitude :math:`\boldsymbol{\sigma}_{R/N}`, the angular rate :math:`\boldsymbol{\omega}_{R/N}`, and the angular
acceleration :math:`\dot{\boldsymbol{\omega}}_{R/N}` of the reference frame :math:`\mathcal{R}` with respect to
the inertial frame :math:`\mathcal{N}`, all in inertial-frame components.

The orbit can be any Keplerian motion -- circular, elliptical, or hyperbolic. The primary celestial body's
inertial state is optional; when not connected the body is treated as fixed at the inertial origin.

This is the FP32 port of the Xmera ``hillPointCpp`` module. Position and velocity inputs remain double-precision
to preserve orbit-scale accuracy; the attitude / rate output is single-precision (FP32).

Module Architecture
-------------------

The module is split into a thin adapter (``HillPoint``) that handles framework integration and an algorithm class
(``HillPointAlgorithm``) that contains the pure math.

Adapter Layer
~~~~~~~~~~~~~

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

``HillPointAlgorithm`` has no tunable parameters. The empty ``HillPointConfig`` class is provided to keep the
algorithm consistent with the standard two-phase init pattern in this codebase.

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
:math:`\mathcal{H}: \{ \hat{\boldsymbol{\imath}}_r, \hat{\boldsymbol{\imath}}_\theta, \hat{\boldsymbol{\imath}}_h \}`,
where

* :math:`\hat{\boldsymbol{\imath}}_r` is the radial unit vector pointing from the primary body to the spacecraft,
* :math:`\hat{\boldsymbol{\imath}}_h` is the orbit angular momentum unit vector normal to the orbital plane,
* :math:`\hat{\boldsymbol{\imath}}_\theta = \hat{\boldsymbol{\imath}}_h \times \hat{\boldsymbol{\imath}}_r` completes
  the right-handed triad.

Relative State
~~~~~~~~~~~~~~

Given the spacecraft inertial state :math:`(\boldsymbol{R}_S, \boldsymbol{v}_S)` and the primary body inertial
state :math:`(\boldsymbol{R}_P, \boldsymbol{v}_P)`, the relative state is

.. math::

   \boldsymbol{r} = \boldsymbol{R}_S - \boldsymbol{R}_P, \qquad
   \boldsymbol{v} = \boldsymbol{v}_S - \boldsymbol{v}_P.

When ``celBodyInMsg`` is not connected, :math:`\boldsymbol{R}_P` and :math:`\boldsymbol{v}_P` default to zero.

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

where the true-anomaly rate and acceleration follow from the standard astrodynamics relations

.. math::

   \dot{f} = \frac{h}{r^2}, \qquad
   \ddot{f} = -2 \frac{\boldsymbol{v} \cdot \hat{\boldsymbol{\imath}}_r}{r} \dot{f},
   \quad h = \| \boldsymbol{r} \times \boldsymbol{v} \|.

The outputs are then rotated into inertial-frame components via :math:`[NR] = [RN]^T`:

.. math::

   {}^{\mathcal{N}}\boldsymbol{\omega}_{R/N} = [NR] \, {}^{\mathcal{R}}\boldsymbol{\omega}_{R/N}, \qquad
   {}^{\mathcal{N}}\dot{\boldsymbol{\omega}}_{R/N} = [NR] \, {}^{\mathcal{R}}\dot{\boldsymbol{\omega}}_{R/N}.

Robustness
~~~~~~~~~~

If the relative orbital radius :math:`r` falls below :math:`1.0\,\mathrm{m}`, the angular rate and acceleration
are forced to zero rather than dividing by an essentially-zero denominator. Note that the original Xmera comment
described the threshold as "1 km" but the value is :math:`1.0` in the same units as :math:`\boldsymbol{R}_S`,
which is meters; the FP32 port preserves the original numerical behavior.

Assumptions and Limitations
---------------------------

* The relative position vector :math:`\boldsymbol{r}` and the relative velocity vector :math:`\boldsymbol{v}` must
  not be collinear. If :math:`\boldsymbol{r} \times \boldsymbol{v} = \boldsymbol{0}`, the orbital plane is not
  defined and :math:`\hat{\boldsymbol{\imath}}_h` is unobtainable; the algorithm will produce non-finite output
  in that degenerate case.
* The spacecraft must not be coincident with the primary body. Inputs satisfying :math:`r \le 1\,\mathrm{m}` are
  routed to the zero-rate branch instead.
* Position and velocity inputs are read in double precision to preserve orbit-scale accuracy. The attitude /
  rate output is single-precision; expect a relative accuracy of :math:`\sim 10^{-7}` on
  :math:`\boldsymbol{\sigma}_{R/N}` and :math:`\boldsymbol{\omega}_{R/N}`.
