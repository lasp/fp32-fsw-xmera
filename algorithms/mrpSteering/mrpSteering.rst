.. raw:: latex

    {\LARGE \textbf{mrpSteering}}

Executive Summary
-----------------
The intend of this module is to implement an MRP attitude steering law where the control output is a vector of
commanded body rates.  To use this module it is required to use a separate rate tracking servo control
module, such as :ref:`rateServoFullNonlinear`, as well.

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
    * - guidInMsg
      - :ref:`AttGuidMsgPayload`
      - Attitude guidance input message
    * - rateCmdOutMsg
      - :ref:`RateCmdMsgPayload`
      - Rate command output message

Module Parameters
-------------------------------
The following table lists all the module parameters than can be set. The parameters are optional unless indicated
(if not specified default is used).

.. list-table:: Module Parameters
    :widths: 60 30 30 30
    :header-rows: 1

    * - Parameter Name
      - Default
      - Description
      - Bounds
    * - K1
      - 0
      - Proportional gain applied to MRP errors
      - Must not be negative (checked in setter)
    * - K3
      - 0
      - Cubic gain applied to MRP errors
      - Must not be negative (checked in setter)
    * - omegaMax (required)
      - 0
      - Maximum rate command of steering control
      - Must be greater than zero (checked in setter)
    * - ignoreOuterLoopFeedforward
      - false
      - Indicates if feedforward term should be included
      - N/A

Module Assumptions and Limitations
----------------------------------
This control assumes the spacecraft is rigid, and that a fast enough rate control sub-servo system is present.

Initialization
--------------
The module is configured by::

    module = mrpSteering.MrpSteering()
    module.modelTag = "mrpSteering"
    module.K1 = K1
    module.K3 = K3
    module.omegaMax = omega_max

If the feed-forward term should be ignored::

    module.ignoreOuterLoopFeedforward = True

Detailed Module Description
---------------------------
The following text describes the mathematics behind the ``mrpSteering`` module.  Further information can also be
found in the journal paper `Speed-Constrained Three-Axes Attitude Control Using Kinematic Steering
<http://dx.doi.org/10.1016/j.actaastro.2018.03.022>`_.

Steering Law Goals
^^^^^^^^^^^^^^^^^^
The goal of MRP Steering is to drive a body frame
:math:`{\mathcal B}:\{ \hat{\mathbf b}_1, \hat{\mathbf b}_2, \hat{\mathbf b}_3 \}` towards a time varying reference
frame :math:`{\mathcal R}:\{ \hat{\mathbf r}_1, \hat{\mathbf r}_2, \hat{\mathbf r}_3 \}`:

.. math::
    :label: eq:MS:1

	\mathbf\sigma_{\mathcal{B}/\mathcal{R}} \rightarrow 0

The attitude error :math:`\mathbf\sigma_{\mathcal{B}/\mathcal{R}}` is provided by an upstream module such as
:ref:`attTrackingError`.

MRP Steering Law
^^^^^^^^^^^^^^^^
To create a kinematic steering law, let :math:`{\mathcal{B}}^{\ast}` be the desired body orientation,
and :math:`\mathbf\omega_{{\mathcal{B}}^{\ast}/\mathcal{R}}` be the desired angular velocity vector of
this body orientation relative to the reference frame :math:`\mathcal{R}`.  The steering law requires
an algorithm for the desired body rates :math:`\mathbf\omega_{{\mathcal{B}}^{\ast}/\mathcal{R}}`
relative to the reference frame.

The desired body rate is computed by

.. math::
    :label: eq:MS:2

	{}^{B}{\mathbf\omega}_{{\mathcal{B}}^{\ast}/\mathcal{R}} = - {\mathbf f}(\mathbf\sigma_{\mathcal{B}/\mathcal{R}})

where :math:`{\mathbf f}(\mathbf\sigma)` is an even function

.. math::
    :label: eq:MS:3

	 f( \sigma_{i}) = \arctan \left(
		(K_{1} \sigma_{i} +K_{3} \sigma_{i}^{3}) \frac{ \pi}{2  \omega_{\text{max}}}
	\right) \frac{2 \omega_{\text{max}}}{\pi}

and with

.. math::

    \mathbf\sigma_{\mathcal{B}/\mathcal{R}} = (\sigma_{1}, \sigma_{2}, \sigma_{3})^{T}

and

.. math::
    :label: eq:MS:15.0

	{\mathbf f}(\mathbf\sigma_{\mathcal{B}/\mathcal{R}}) = \begin{bmatrix}
		f(\sigma_{1})\\ f(\sigma_{2})\\ f(\sigma_{3})
		\end{bmatrix}

The required velocity servo loop design is aided by knowing the body-frame derivative of
:math:`{}^{B}{\mathbf\omega}_{{\mathcal{B}}^{\ast}/\mathcal{R}}` to implement a feed-forward component.
Using the :math:`{\mathbf f}()` function definition, this requires the time
derivatives of :math:`f(\sigma_{i})`.

.. math::

    \frac{{}^{B}{\text{d} ({}^{B}{\mathbf\omega}_{{\mathcal{B}}^{\ast}/\mathcal{R}} ) }}{\text{d} t} =
    {\mathbf\omega}_{{\mathcal{B}}^{\ast}/\mathcal{R}} '
    = - \frac{\partial {\mathbf f}}{\partial \mathbf\sigma_{{\mathcal{B}}^{\ast}/\mathcal{R}}} \dot{\mathbf\sigma}_{{\mathcal{B}}^{\ast}/\mathcal{R}}
    = - \left[ \begin{matrix}
        \frac{\partial  f}{\partial  \sigma_{1}} \dot{ \sigma}_{1} \\
		\frac{\partial  f}{\partial  \sigma_{2}} \dot{ \sigma}_{2} \\
		\frac{\partial  f}{\partial  \sigma_{3}} \dot{ \sigma}_{3}
    \end{matrix} \right]

where

.. math::
    \dot{\mathbf\sigma}	_{{\mathcal{B}}^{\ast}/\mathcal{R}} =
    \left[ \begin{matrix}
        \dot\sigma_{1}\\
		\dot\sigma_{2}\\
		\dot\sigma_{3}
    \end{matrix} \right] =
    \frac{1}{4}[B(\mathbf\sigma_{{\mathcal{B}}^{\ast}/\mathcal{R}})]
    {}^{B}{\mathbf\omega}_{{\mathcal{B}}^{\ast}/\mathcal{R}}

Using the :math:`f()` definition, its sensitivity with respect to :math:`\sigma_{i}` is

.. math::
    \frac{
		\partial f
	}{
		\partial \sigma_{i}
	} =
    \frac{
	(K_{1}  + 3 K_{3} \sigma_{i}^{2})
	}{
	1+(K_{1}\sigma_{i} + K_{3} \sigma_{i}^{3})^{2} \left(\frac{\pi}{2 \omega_{\text{max}}}\right)^{2}
	}

If ignoreOuterLoopFeedforward == true, then :math:`{\mathbf\omega}_{{\mathcal{B}}^{\ast}/\mathcal{R}} ' = 0` and the
computation of the feed-forward component is skipped.

.. figure:: ./_Documentation/Images/fSigmaOptionsA.jpg
   :width: 50%
   :align: center

   :math:`\omega_{\text{max}}` dependency with :math:`K_{1} = 0.1`, :math:`K_{3} = 1`

.. figure:: ./_Documentation/Images/fSigmaOptionsB.jpg
    :width: 50 %
    :align: center

    :math:`K_{1}` dependency with :math:`\omega_{\text{max}}` = 1 deg/s, :math:`K_{3} = 1`

.. figure:: ./_Documentation/Images/fSigmaOptionsC.jpg
    :width: 50 %
    :align: center

    :math:`K_{3}` dependency with :math:`\omega_{\text{max}}` = 1 deg/s, :math:`K_{1} = 0.1`

Figures 1-3 illustrate how the parameters :math:`\omega_{\text{max}}`, :math:`K_{1}` and :math:`K_{3}`
impact the steering law behavior.  The maximum steering law rate commands are easily set through the
:math:`\omega_{\text{max}}` parameters.  The gain :math:`K_{1}` controls the linear stiffness when
the attitude errors have become small, while :math:`K_{3}` controls how rapidly the steering law
approaches the speed command limit.
