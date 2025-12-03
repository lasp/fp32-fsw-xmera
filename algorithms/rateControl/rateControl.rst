======================
Rate Control Algorithm
======================

Introduction
============
The rate control flight software module computes the required body frame control torque to achieve a desired angular velocity. Each time a new guidance message is written to the module,
the torque vector is computed, taking into consideration the reference frame rotation, angular acceleration, and any known external disturbances.
The module has a feedback term to stabilize the spacecraft by applying an opposite torque to damp the rotational rate error. It also has a coupling and feedforward terms to account for
the gyroscopic effects and the natural motion of the rotating reference frame, respectively. The output torque is written in the body frame where it can be sent to the downstream actuator's module.


Module Input/Output Messages
============================
The following table lists all the module input and output messages. The Msg type contains a link to the message
structure definition, while the description provides information on what this message is used for.

.. list-table:: Module I/O Messages
   :widths: 15 30 60
   :header-rows: 1

   * - **Msg Variable Name**
     - **Msg Type**
     - **Description**
   * - ``guidInMsg``
     - :ref:`AttGuidMsgF32Payload`
     - Input message containing the attitude guidance message, including Modified Rodrigues Parameters (MRP), rate error, reference angular rate, and angular acceleration.
   * - ``vehConfigInMsg``
     - :ref:`VehicleConfigMsgF32Payload`
     - Input message containing the spacecraft configuration, including its inertia tensor, mass and Center Of Mass (COM).
   * - ``cmdTorqueOutMsg``
     - :ref:`CmdTorqueBodyMsgF32Payload`
     - Output message containing the commanded torque expressed in the body frame components.

Algorithm Flow
==============
Each time a new attitude guidance message is read, the module computes the control torque
vector :math:`\mathbf{L}_r` expressed in the body frame. The control is similar to :ref:`mrpPD`,
but it does not feed back on the orientation error, instead, it  purely damps the rate error and take
into account the reference motion and any external torques.

The control torque is calculated as

.. math::

    \mathbf{L}_r = - P \, \boldsymbol{\omega}_{BR,B}
      + \boldsymbol{\omega}_{RN,B} \times [I] \boldsymbol{\omega}_{BN,B}
      + [I] (\dot{\boldsymbol{\omega}_{RN,B}} - \boldsymbol{\omega}_{BN,B} \times \boldsymbol{\omega}_{RN,B})
      - \boldsymbol{L}_{known}

where:

- :math:`P` is a positive, user-defined scalar feedback gain [N.m.s].
- :math:`\boldsymbol{\omega}_{BR,B}` is the angular velocity of the body frame relative to the reference frame expressed in the body frame components [rad/s].
- :math:`\boldsymbol{\omega}_{RN,B}` is the angular velocity of the reference frame relative to the inertial frame expressed in the body frame components [rad/s].
- :math:`[I]` is the spacecraft inertia about point B expressed in the body frame components [kg.m^2].
- :math:`\boldsymbol{\omega}_{BN,B}` is the total angular velocity of the body frame relative to the inertial frame expressed in the body frame components, and its defined as:


  .. math::
      \boldsymbol{\omega}_{BN,B} =
      \boldsymbol{\omega}_{BR,B} + \boldsymbol{\omega}_{RN,B} \quad [\text{rad/s}]

- :math:`\dot{\boldsymbol{\omega}_{RN,B}}` is the angular acceleration of the reference frame relative to the inertial frame, expressed in the body frame [rad/s^2].
- :math:`\boldsymbol{\tau}_{\text{known}}` represents any known external disturbance torques expressed in the body frame [N.m].

The first term is the feedback law, that applies a negative torque to damp the rate error.
The second and third terms are the coupling and feedforward components where they capture the gyroscopic effects, and ensures the controller is taking into account the motion of the rotating
reference frame, respectively.
The last term subtracts any external torques that might act on the spacecraft.


Controller Functions
====================
Below is a list of functions that this flight software module performs:

- Reads the incoming attitude guidance message that contains the rate error, reference angular rate, and angular acceleration.
- Reads the vehicle configuration message that includes the spaceraft inertia tensor and mass properties.
- Computes the control torque required to compensate for the rate error.
- Writes the computed body frame control torque to the output message.


Controller Assumptions and Limitations
======================================
- This module only controls rate and does not stabilize the spacecraft attitude (orientation). These can be handled using other modules like :ref:`mrpPD`.
- The inertia must be correctly written in the body frame when read from the attitude guidance message.
- The feedback gain must be positive and tuned properly to control the spacecraft's angular rates.
- The controller does not account for the actuators limitations, these must be handled downstream actuator modules.


Test Description and Success Criteria
=====================================
The rate control test is located in ``algorithms\rateControl\_tests\test_rateControl.py``. This test verifies that the module works
correctly for a given set of guidance and spacecraft configuration paramters. Two cases are tested, one with zero external disturbance torque and another with a :math:`\boldsymbol{\tau}_{\text{known}} = [0.1, 0.2, 0.3]^T` N·m applied.
In both cases, the computed control torque is compared with the analytical control torque equation written within the test script.

The test runs for :math:`1` second with :math:`0.5` second time step. All the attitude guidance and spacecraft configuration parameters are defined and connected to the messages.
The success criteria for this test is to have the commanded control torque match the analytical value with a tolerance of :math:`10^{-7}` [N·m] for both test cases.
