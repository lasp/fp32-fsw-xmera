Executive Summary
-----------------

The dvGuidance module produces an inertially fixed attitude reference frame :math:`\mathcal{R}` for a delta-V burn.
The burn frame is constructed from the commanded delta-V vector and a preferred
direction for its 2nd axis: the 3rd axis points along the commanded delta-V vector, and the 2nd axis points as close as
possible to the preferred direction.
The output is the MRP attitude :math:`\boldsymbol{\sigma}_{R/N}` of :math:`\mathcal{R}` with respect to the inertial
frame :math:`\mathcal{N}`. Because :math:`\mathcal{R}` is inertially fixed, the published reference angular rate
:math:`\boldsymbol{\omega}_{R/N}` and angular acceleration :math:`\dot{\boldsymbol{\omega}}_{R/N}` are zero.

The module does not rotate the burn frame. When a rotation is required, the output of dvGuidance is used as the base
frame :math:`\mathcal{R}_0` of the :ref:`mrpRotation` module, which applies the rotation to produce
:math:`\mathcal{R}`.

This module derives from the Xmera ``dvAttGuidance`` module. Unlike the original, the burn frame is inertially fixed,
and the commanded delta-V vector and preferred 2nd-axis direction are configuration parameters instead of inputs read
from a burn command message. Inputs and outputs are single-precision (FP32); the algorithm is single-precision
throughout.

Module Architecture
-------------------

The module is split into a thin adapter (``DvGuidance``) that handles framework integration and an algorithm class
(``DvGuidanceAlgorithm``) that contains the pure math.

Adapter Layer
~~~~~~~~~~~~~

The adapter inherits from ``SysModel`` and owns the output message. It follows the two-phase init pattern: the public
configuration properties are set after construction, and ``reset()`` validates them and constructs the algorithm. Each
``updateState()`` publishes the algorithm's :math:`\boldsymbol{\sigma}_{R/N}` with zero reference rate and acceleration.

.. list-table:: Module I/O Messages
    :widths: 25 30 45
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - ``attRefOutMsg``
      - :ref:`AttRefMsgF32Payload`
      - Reference attitude / rate / acceleration of :math:`\mathcal{R}` relative to :math:`\mathcal{N}`.

Configuration
~~~~~~~~~~~~~

All parameters are validated once in ``DvGuidanceConfig::create()`` when the adapter's ``reset()`` or
``reconfigure()`` runs; the algorithm never sees an invalid configuration.

.. list-table:: Configuration parameters
    :widths: 14 26 24 12 38
    :header-rows: 1

    * - Parameter Name
      - Type
      - Valid range
      - Default
      - Description
    * - ``cmdDv_N``
      - Eigen::Vector3f
      - finite, :math:`\lVert\Delta\boldsymbol{v}\rVert \geq 10^{-6}` [m/s]
      - :math:`\boldsymbol{0}` (must be set)
      - commanded delta-V vector in inertial-frame components; defines the 3rd axis of :math:`\mathcal{R}`
    * - ``rHatPref_N``
      - Eigen::Vector3f
      - finite, within :math:`10^{-3}` of unit length, at least :math:`\sim 0.06^\circ` from parallel or
        antiparallel to ``cmdDv_N``
      - :math:`\boldsymbol{0}` (must be set)
      - preferred unit direction of the 2nd axis of :math:`\mathcal{R}` in inertial-frame components; normalized
        when the configuration is built, and only its component perpendicular to ``cmdDv_N`` affects the frame

Two-Phase Initialization
~~~~~~~~~~~~~~~~~~~~~~~~

The Python usage follows the standard adapter lifecycle: set the public configuration properties, call ``reset()``
once, then drive ``updateState()`` each cycle::

    module = dvGuidanceF32.DvGuidance()
    module.cmdDv_N = [5.0, 5.0, 5.0]
    module.rHatPref_N = [1.0, 0.0, 0.0]

    sim.AddModelToTask(task_name, module)
    sim.InitializeSimulation()
    sim.ExecuteSimulation()

If ``cmdDv_N`` or ``rHatPref_N`` is invalid when ``reset()`` runs, including when either is left at its zero default,
an ``fsw::invalid_argument`` is thrown. If ``updateState()`` is called before ``reset()``, an
``XmeraLifecycleException`` is thrown. To change the configuration after ``reset()``, set the properties and call
``reconfigure()``; the new values are used from the next ``updateState()``.

Mathematical Formulation
------------------------

All vector components in this section are taken in a common inertial frame :math:`\mathcal{N}`.

Burn Reference Frame
~~~~~~~~~~~~~~~~~~~~

Let :math:`\mathcal{R} = \{\hat{\boldsymbol{r}}_1, \hat{\boldsymbol{r}}_2, \hat{\boldsymbol{r}}_3\}` be the burn
reference frame. Its DCM relative to :math:`\mathcal{N}` is

.. math::

   [RN] = \begin{bmatrix}
       \hat{\boldsymbol{r}}_1^T \\
       \hat{\boldsymbol{r}}_2^T \\
       \hat{\boldsymbol{r}}_3^T
   \end{bmatrix}.

The axes are constructed from the commanded delta-V vector :math:`\Delta\boldsymbol{v}` (``cmdDv_N``) and the
preferred 2nd-axis unit direction :math:`\hat{\boldsymbol{r}}_{2,\text{pref}}` (``rHatPref_N``):

* :math:`\hat{\boldsymbol{r}}_3` points along the commanded delta-V vector,

  .. math::

     \hat{\boldsymbol{r}}_3 = \frac{\Delta\boldsymbol{v}}{\| \Delta\boldsymbol{v} \|},

* :math:`\hat{\boldsymbol{r}}_1` is the cross product of the preferred direction with :math:`\hat{\boldsymbol{r}}_3`,

  .. math::

     \hat{\boldsymbol{r}}_1 = \frac{\hat{\boldsymbol{r}}_{2,\text{pref}} \times \hat{\boldsymbol{r}}_3}
                                   {\| \hat{\boldsymbol{r}}_{2,\text{pref}} \times \hat{\boldsymbol{r}}_3 \|},

* :math:`\hat{\boldsymbol{r}}_2` completes the right-handed frame,

  .. math::

     \hat{\boldsymbol{r}}_2 = \hat{\boldsymbol{r}}_3 \times \hat{\boldsymbol{r}}_1.

The resulting 2nd axis is the component of :math:`\hat{\boldsymbol{r}}_{2,\text{pref}}` perpendicular to the commanded
delta-V vector, normalized: the direction closest to the preferred direction that is perpendicular to
:math:`\Delta\boldsymbol{v}`. When :math:`\hat{\boldsymbol{r}}_{2,\text{pref}}` is already perpendicular to
:math:`\Delta\boldsymbol{v}`, :math:`\hat{\boldsymbol{r}}_2 = \hat{\boldsymbol{r}}_{2,\text{pref}}`.
The construction is illustrated in Figure 1.

.. _fig1_dvGuidance:

.. figure:: _Documentation/_Images/dvGuidanceFig1.pdf
    :width: 75%
    :align: center

    Construction of the burn reference frame :math:`\mathcal{R}` from the commanded delta-V vector
    :math:`\Delta\boldsymbol{v}` and the preferred 2nd-axis direction :math:`\hat{\boldsymbol{r}}_{2,\text{pref}}`.
    The dotted line is the component of :math:`\hat{\boldsymbol{r}}_{2,\text{pref}}` along
    :math:`\Delta\boldsymbol{v}` that is removed to obtain :math:`\hat{\boldsymbol{r}}_2`.

The MRP output is

.. math::

   \boldsymbol{\sigma}_{R/N} = \mathrm{C2MRP}\!\left([RN]\right).

Because :math:`\mathcal{R}` depends only on the configuration, :math:`[RN]` and :math:`\boldsymbol{\sigma}_{R/N}` are
computed once in ``setConfig()``, when a configuration is installed at construction or by ``reconfigure()``. Each
``update()`` returns the stored :math:`\boldsymbol{\sigma}_{R/N}`.

Angular Rate and Acceleration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The burn reference frame depends only on the configuration, so it is inertially fixed between configuration changes.
The algorithm does not output rates; the adapter publishes

.. math::

   \boldsymbol{\omega}_{R/N} = \boldsymbol{0}, \qquad \dot{\boldsymbol{\omega}}_{R/N} = \boldsymbol{0}.

Configuration Validation
~~~~~~~~~~~~~~~~~~~~~~~~

Two degenerate geometries cannot define the burn reference frame, and ``DvGuidanceConfig::create()`` rejects them by
throwing ``fsw::invalid_argument``:

* A near-zero :math:`\Delta\boldsymbol{v}` does not define a burn direction. The configuration is rejected when
  :math:`\lVert\Delta\boldsymbol{v}\rVert` falls below ``kMinNorm``.

* A preferred 2nd-axis direction sufficiently close to parallel or antiparallel with
  :math:`\hat{\boldsymbol{r}}_3`, collapses the cross product used to construct :math:`\hat{\boldsymbol{r}}_1`. The
  configuration is rejected when the magnitude of
  :math:`\hat{\boldsymbol{r}}_{2,\text{pref}} \times \hat{\boldsymbol{r}}_3` falls below ``kMinCross``.

``rHatPref_N`` must also be within :math:`10^{-3}` of unit length, the tolerance used for the unit-vector
configuration parameters of the other FP32 algorithms, and ``create()`` stores it normalized. Non-finite values
are rejected as well. Because an invalid configuration cannot be constructed, the frame
construction has no degenerate-input branch.

Assumptions and Limitations
---------------------------

* The spacecraft body 3rd axis is assumed to be the thrust direction: :math:`\hat{\boldsymbol{r}}_3` is aligned with
  :math:`+\Delta\boldsymbol{v}`, so tracking :math:`\mathcal{R}` produces the commanded delta-V only if the thrust acts
  along the body :math:`+\hat{\boldsymbol{b}}_3` axis.
* ``rHatPref_N`` is a preference, not an enforced axis. The achieved 2nd axis is the projection of ``rHatPref_N`` onto
  the plane perpendicular to the commanded delta-V vector, so it differs from ``rHatPref_N`` by the angle between them;
  near the ``kMinCross`` limit the two are almost perpendicular.
* The burn direction is held inertially fixed for the whole burn. A burn direction that should change during the burn
  (for example, to follow the orbit) must be provided by updating the configuration through ``reconfigure()``.

Numerical conditioning
----------------------

``kMinCross`` = :math:`10^{-3}` (an angle of about :math:`0.06^\circ`) is consistent with the small-angle threshold
used by other FP32 guidance algorithms that construct reference frames from cross products. As ``rHatPref_N``
approaches alignment with ``cmdDv_N``, the cross-product magnitude decreases, increasing sensitivity to FP32 round-off.
This threshold prevents frame construction when the two vectors are nearly parallel.

``kMinNorm`` = :math:`10^{-6}` m/s is not set by FP32 round-off. The frame construction normalizes ``cmdDv_N`` using a
scaled norm, which maintains numerical accuracy even for very small magnitudes. Commands below this threshold are
treated as effectively zero and do not define a meaningful burn direction.
