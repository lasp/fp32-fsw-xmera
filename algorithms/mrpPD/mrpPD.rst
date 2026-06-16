Executive Summary
-----------------
This module provides a MRP based PD attitude control module. It is similar to :ref:`mrpFeedback`, but without the RW or the integral feedback option. The feedback control is able to asymptotically track a reference attitude if there are no unknown dynamics and the attitude control torque is implemented with a thruster set. All quantities are single precision (float).

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg variable name is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - guidInMsg
      - :ref:`AttGuidMsgF32Payload`
      - Attitude guidance input message (sigma_BR, omega_BR_B, domega_RN_B)
    * - vehConfigInMsg
      - :ref:`VehicleConfigMsgF32Payload`
      - Vehicle configuration input message (provides the spacecraft inertia ISCPntB_B)
    * - cmdTorqueOutMsg
      - :ref:`CmdTorqueBodyMsgF32Payload`
      - Commanded torque output message


Module Parameters
-----------------

.. list-table:: Module Parameters
    :widths: 25 20 10 45
    :header-rows: 1

    * - Parameter Name
      - Type
      - Default
      - Description
    * - K
      - float
      - 0
      - proportional gain applied to MRP errors; must be >= 0
    * - P
      - float
      - 0
      - rate-error feedback (derivative) gain; must be >= 0
    * - knownTorquePntB_B
      - Eigen::Vector3f
      - zero
      - known external torque in body-frame components; must be finite

The spacecraft inertia ``ISCPntB_B`` is not a user property: it is read from ``vehConfigInMsg`` at ``reset()`` (an
identity matrix is used if that message has not been written) and must be a valid inertia matrix. All four values are
validated when the configuration is built in ``reset()``.


Detailed Module Description
---------------------------
This attitude feedback module using the MRP feedback control related to the control in section 8.4.1 in `Analytical Mechanics of Space Systems <http://dx.doi.org/10.2514/4.105210>`_:

.. math::
    {\mathbf L}_{r} =  -K \mathbf\sigma - [P] \delta\mathbf\omega   + [I]\dot{\mathbf\omega}_{r}
            - \mathbf L

Note that this control solution creates an external control torque which must be produced with a cluster of thrusters.  No reaction wheel information is used here.  Further, the feedback control component is a simple proportional and derivative feedback formulation.  As shown in `Analytical Mechanics of Space Systems <http://dx.doi.org/10.2514/4.105210>`_, this control can asymptotically track a general reference trajectory given by the reference frame :math:`\mathcal R`.
The torque provided does not compute gyroscopic terms.

Module Assumptions and Limitations
----------------------------------
This control assumes the spacecraft is rigid and that the inertia tensor does not vary with time.
The module does not add gyroscopic terms to the output torque.

User Guide
----------
The required module configuration is::

    module = mrpPDF32.MrpPD()
    module.modelTag = "mrpPD"
    module.K = K
    module.P = P
    module.knownTorquePntB_B = knownTorquePntB_B

    module.guidInMsg.subscribeTo(guidance_msg)
    module.vehConfigInMsg.subscribeTo(vehicle_config_msg)

The gains and known torque must be set, and the input messages connected, before ``reset()`` is called: the module
builds and validates its immutable configuration at reset (reading the inertia from ``vehConfigInMsg``).
