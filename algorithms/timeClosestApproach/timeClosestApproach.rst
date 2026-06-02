Executive Summary
-----------------
This module computes the time of closest approach (TCA) and its uncertainty during a rectilinear flyby. It is based on "Attitude Uncertainty Quantification of Rectilinear Asteroid Flybys (Teil & Calaon, 2025)".

The adapter reads position, velocity, and covariance from a single ``filterInMsg`` (a 6-state filter message). The first three state elements are the relative position :math:`\mathbf{r}_{BN}^N` and the last three are the relative velocity :math:`\mathbf{v}_{BN}^N`.

Message Connection Descriptions
-------------------------------

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - filterInMsg
      - :ref:`FilterMsgF32Payload`
      - Input message providing the 6-element state vector (position and velocity) and 6×6 covariance matrix.
    * - tcaOutMsg
      - :ref:`TimeClosestApproachMsgF32Payload`
      - Output message containing the TCA estimate and its standard deviation. Both fields are zero when the input guard fails.

Detailed Module Description
---------------------------

Algorithm Inputs and Outputs
............................

**Inputs** (from ``filterInMsg``):

- :math:`\mathbf{r}_{BN}^N \in \mathbb{R}^3` — relative position vector (``state[0:3]``)
- :math:`\mathbf{v}_{BN}^N \in \mathbb{R}^3` — relative velocity vector (``state[3:6]``)
- :math:`P \in \mathbb{R}^{6 \times 6}` — filter covariance (``covar``)

**Outputs**:

- :math:`t_{CA}` — time of closest approach estimate [s]
- :math:`\sigma_{t_{CA}}` — standard deviation of the TCA estimate [s]

Both outputs are zero when the input guard fails (see Module Assumptions). Given a positive-definite covariance, :math:`\sigma_{t_{CA}} = 0` unambiguously signals a failed guard.

Rectilinear Motion Model
........................
The flyby is modeled as rectilinear motion: the spacecraft moves with constant velocity. At every filter read the relative position and velocity vectors :math:`\boldsymbol{r}` and :math:`\boldsymbol{v}` are available. The flight path angle is :math:`\gamma_0 = \theta - \frac{\pi}{2}` and the ratio :math:`f_0 = \frac{v_0}{r_0}`. The angle :math:`\theta` between :math:`-\boldsymbol{r}` and :math:`\boldsymbol{v}` is:

.. math::
    \theta = \arccos\left( -  \mathbf{\hat{r}}  \cdot  \mathbf{\hat{v}} \right )

The time of closest approach is:

.. math::
    t_{CA} = - \frac{\sin(\gamma_0)}{f_0}

The TCA uncertainty, where :math:`P` is the 6×6 state covariance, is:

.. math::
    \sigma_{t_{CA}}^2 = \frac{1}{f_0^2}  \left[ \frac{\mathbf{\hat{v}}^T}{\|\mathbf{r}\|} \quad \frac{1}{\|\mathbf{v}\|}  \left( \mathbf{\hat{r}}^T - \sin \gamma_0 \mathbf{\hat{v}}^T \right) \right] P \left[ \frac{\mathbf{\hat{v}}^T}{\|\mathbf{r}\|} \quad \frac{1}{\|\mathbf{v}\|}  \left( \mathbf{\hat{r}}^T - \sin\gamma_0 \mathbf{\hat{v}}^T \right) \right]^T

Note: A naive implementation computes :math:`\theta = \arccos(-\mathbf{\hat{r}} \cdot \mathbf{\hat{v}})` and then evaluates :math:`\sin(\theta - \pi/2)`, which loses significant digits when :math:`\theta \approx \pi/2`. The implementation instead uses the identity

.. math::
    \sin(\gamma_0) = \sin \left(\theta - \frac{\pi}{2}\right) = -\cos\theta = \mathbf{\hat{r}} \cdot \mathbf{\hat{v}}

and computes ``sinFPA = r_hat.dot(v_hat)`` directly, bypassing the trigonometric functions entirely.

Module Assumptions and Limitations
----------------------------------
This module assumes the flyby to be a rectilinear motion of the spacecraft, meaning the spacecraft moves with a constant velocity. The limitations of this module are inherent to the geometry of the problem, which determines whether or not all the constraints can be satisfied. For example, one constraint for this module to work is that the target should have a small gravitational influence, implying its mass is relatively low. Another constraint is that the distance between the target and the spacecraft should be large enough to minimize the gravitational effects of the target on the spacecraft.

For valid output, the algorithm assumes:

- :math:`\|\mathbf{r}_{BN}^N\| \geq 10^{-3}` and :math:`\|\mathbf{v}_{BN}^N\| \geq 10^{-3}` — the position and velocity vector must be greater than the :math:`10^{-3} m` and :math:`10^{-3} m/s` thresholds, respectively.
- :math:`P \in \mathbb{R}^{6 \times 6}` is positive definite — the covariance must be full-rank. Under this assumption :math:`\sigma_{t_{CA}} > 0` whenever the computation succeeds, so a zero output unambiguously signals an invalid output.

User Guide
----------
The required module configuration is::

    tca_module = timeClosestApproach.TimeClosestApproach()
    tca_module.modelTag = "TimeClosestApproach"
    tca_module.filterInMsg.subscribeTo(filterMsg)
    unitTestSim.AddModelToTask(unitTaskName, tca_module)

This module has no configurable parameters; all inputs are supplied through the single required message above.
