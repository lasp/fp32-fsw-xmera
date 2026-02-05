.. raw:: latex

    {\LARGE \textbf{rwMotorTorque}}

Executive Summary
-----------------

This module maps a desired torque to control the spacecraft, and maps it to the available reaction wheels using a
minimum norm inverse fit.

The optional wheel availability message is used to include or exclude particular reaction wheels from the torque
solution. The desired control torque can be mapped onto particular orthogonal control axes to implement a partial
solution for the overall attitude control torque.

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

Model Assumptions and Limitations
---------------------------------
This code makes the following assumptions:

- The rank of the control mapping matrix :math:`[CB][G_{s}]` (constructed using only the available wheels) must be equal
  or greater than the number of control axes. If this is not the case, an invalid argument exception is thrown. This is
  checked during reset.
- It is assumed that the availability of the reaction wheels is known at the time of reset, and does not change after
  that.

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

To invert the equation above, a minimum norm solution is used
yielding:

.. math::

   \mathbf u_{s}  = [CG]^T \left([CG][CG]^T\right)^{-1}\
   	{}^{\mathcal{C}}{\mathbf L}
   _{r}

The final step is to map the array of available motor torques
:math:`\mathbf u_{s}` onto the output array of installed RW motor torques.

RW Availability
===============

If the input message ``rwAvailInMsg`` is linked, then the RW
availability message is read in. The torque mapping is only used for
RW’s whose availability setting is ``AVAILABLE``. If it is
``UNAVAILABLE`` then that RW output torque is set to zero.

Model Functions
---------------
The code performs the following functions:

- **Maps control torque vector onto available reaction wheels**: Takes a
  desired body-frame torque from ``CmdTorqueBodyIntMsg`` and maps it
  onto the RW axes.

- **Removes torque from unavailable reaction wheels**: The module
  observes the availability of the RWs and maps the torques to only
  available reaction wheels.
