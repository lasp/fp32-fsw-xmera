======================
Rate Control Algorithm
======================

Introduction
============
The rate control flight software module computes the required body frame control torque to achieve a desired angular velocity. Each time a new guidance data is written to the module,
the torque vector is computed, taking into consideration the reference frame rotation, angular acceleration, and any known external disturbances.
The module has a feedback term to stabilize the spacecraft by applying an opposite torque to damp the rotational rate error and a reference angular-acceleration feedforward term.
The output torque is written in the body frame where it can be sent to the downstream actuator's module.


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
   * - ``omega_BR_B``
     - Eigen::Vector3f
     - Input rate error in the body frame components [rad/s].
   * - ``domega_RN_B``
     - Eigen::Vector3f
     - Input reference angular acceleration in the body frame components [rad/(s^2)].
   * - ``L_r``
     - Eigen::Vector3f
     - Output commanded torque expressed in the body frame components [Nm].

Algorithm Flow
==============
Each time a new attitude guidance data is read, the module computes the control torque
vector :math:`\mathbf{L}_r` expressed in the body frame. The control is similar to :ref:`mrpPD`,
but it does not feed back on the orientation error, instead, it  purely damps the rate error and take
into account the reference motion and any external torques.

The control torque is calculated as

.. math::

    \mathbf{L}_r = - P \, \boldsymbol{\omega}_{BR,B}
      + [I] (\dot{\boldsymbol{\omega}_{RN,B}})
      - \boldsymbol{L}_{known}

where:

- :math:`P` is a positive, user-defined scalar feedback gain [N.m.s].
- :math:`\boldsymbol{\omega}_{BR,B}` is the angular velocity of the body frame relative to the reference frame expressed in the body frame components [rad/s].
- :math:`[I]` is the spacecraft inertia about point B expressed in the body frame components [kg.m^2].
- :math:`\dot{\boldsymbol{\omega}_{RN,B}}` is the angular acceleration of the reference frame relative to the inertial frame, expressed in the body frame [rad/s^2].
- :math:`\boldsymbol{\tau}_{\text{known}}` represents any known external disturbance torques expressed in the body frame [N.m].

The first term is the feedback law, that applies a negative torque to damp the rate error.
The second term is a feedforward component taking into account the motion of the rotating reference frame.
The last term subtracts any external torques that might act on the spacecraft.


Controller Functions
====================
Below is a list of functions that this flight software module performs:

- Reads the incoming attitude guidance message that contains the rate error and reference angular acceleration.
- Reads the vehicle configuration message that includes the spaceraft inertia tensor and mass properties.
- Computes the control torque required to compensate for the rate error.


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

Fuzz Test Description
=====================
The fuzz test is located in ``algorithms/rateControl/_tests/test_rateControl_fuzz.cpp``. It uses
`FuzzTest <https://github.com/google/fuzztest>`_ to generate a wide range of valid inputs and check
that the single-precision algorithm output agrees with a double-precision reference computation
evaluated on the same float-valued test case.

Input Domains
-------------
The fuzzer drives ``fuzzAdapterRateControl`` with the following parameter ranges:

.. list-table:: Fuzz Input Domains
   :widths: 30 20 50
   :header-rows: 1

   * - **Parameter**
     - **Type / Range**
     - **Description**
   * - ``ev1``, ``ev2``
     - ``float`` [1, 1×10\ :sup:`6`]
     - Eigenvalue magnitudes used to construct a valid positive-definite inertia matrix with ``generateValidInertiaMatrix``.
   * - ``sigma1``, ``sigma2``, ``sigma3``
     - ``float`` [1×10\ :sup:`-6`, 1]
     - Off-diagonal shaping parameters for the inertia matrix.
   * - ``DerivativeGainP``
     - ``double`` [0, 1×10\ :sup:`6`]
     - Derivative feedback gain :math:`P`.
   * - ``knownTorque_a``
     - ``double[3]`` [−1×10\ :sup:`6`, 1×10\ :sup:`6`]
     - Known external disturbance torque :math:`\boldsymbol{L}_{\text{known}}` [N·m].
   * - ``omega_BR_a``
     - ``double[3]`` [−1×10\ :sup:`6`, 1×10\ :sup:`6`]
     - Angular rate error :math:`\boldsymbol{\omega}_{BR,B}` [rad/s].
   * - ``domega_RN_a``
     - ``double[3]`` [−1×10\ :sup:`6`, 1×10\ :sup:`6`]
     - Reference angular acceleration :math:`\dot{\boldsymbol{\omega}}_{RN,B}` [rad/s\ :sup:`2`].

Test Method
-----------
The algorithm operates in single precision, so the fuzzed inputs are first converted to ``float`` to
form the actual test case. The test then performs two computations:

#. Run ``RateControlAlgorithm::update()`` in ``float``.
#. Convert those same float values to ``double`` and evaluate the same formula with the independent
   helper ``referenceUpdate()``.

This checks whether the flight software implementation matches a higher-precision evaluation of the
same float-valued inputs.

.. code-block:: cpp

    const Eigen::Vector3f out_alg = alg.update(omega_BR_B_f, domega_RN_B_f);

    const Eigen::Vector3d out_ref_d =
        referenceUpdate(spacecraftInertia_f.cast<double>(),
                        static_cast<double>(derivativeGainP_f),
                        knownTorquePntB_B_f.cast<double>(),
                        omega_BR_B_f.cast<double>(),
                        domega_RN_B_f.cast<double>());

Comparison Tolerance
--------------------
The comparison uses ``EXPECT_NEAR`` with a small absolute tolerance based on the magnitudes of the
terms in each torque component:

.. math::

    \text{tol}_i = 8 \, \varepsilon_{\text{float}}
    \Bigl(
    |P \omega_i|
    + \sum_j |I_{ij}\dot{\omega}_j|
    + |L_i|
    \Bigr)
    + \text{float\_min}

This tolerance is used instead of ``EXPECT_FLOAT_EQ`` because large terms can nearly cancel, making
the final result much smaller than the intermediate products. In that case, a fixed ULP-based check
can be too strict, while the bound above correctly scales with the size of the contributing terms.
Each torque component is formed from 4 float multiplications and about 4 float add/subtract operations.
Since each rounded float operation contributes error on the order of machine epsilon times the magnitude of the terms involved, a factor of 8 is a simple conservative bound for the total accumulated roundoff.

The fuzz test passes when every output component from the single-precision algorithm agrees with the
double-precision reference within this tolerance.

Analysis of Gyroscopic Terms
============================
The control law omits two gyroscopic coupling terms that appear in the full Euler equation:

.. math::

    \boldsymbol{\tau}_{\text{gyro,1}} = \boldsymbol{\omega}_{RN,B} \times [I]\,\boldsymbol{\omega}_{BN,B}

    \boldsymbol{\tau}_{\text{gyro,2}} = [I]\bigl(-\boldsymbol{\omega}_{BN,B} \times \boldsymbol{\omega}_{RN,B}\bigr)

This section bounds their magnitude against the per-axis torque authority limits (25 N·m about X,
14 N·m about Y and Z) to justify that omission.

Inertia
-------
The spacecraft inertia about the centre of mass in the body frame is assembled from
``spacecraft_props.yaml`` (deployed configuration, BOL propellant loading) by
``dynamics.py::set_spacecraft_hub()``:

.. list-table::
   :widths: 20 20 20
   :header-rows: 0

   * - 2261.0
     - −85.0
     - 29.9
   * - −85.0
     - 4898.6
     - 86.1
   * - 29.9
     - 86.1
     - 4077.9

Units: kg·m²

The largest principal moment of inertia is :math:`I_{\max} = 4899` kg·m².

Magnitude Bound
---------------
During rate damping, both :math:`\boldsymbol{\omega}_{BN,B}` and :math:`\boldsymbol{\omega}_{RN,B}`
are assumed bounded by :math:`\omega_{\max} = 1` deg/s = 0.01745 rad/s.

Using the vector-norm inequality :math:`\|\mathbf{a} \times \mathbf{b}\| \le \|\mathbf{a}\|\,\|\mathbf{b}\|`:

.. math::

    \|\boldsymbol{\tau}_{\text{gyro,1}}\| \le \|\boldsymbol{\omega}_{RN,B}\|\,\|[I]\,\boldsymbol{\omega}_{BN,B}\|
    \le \omega_{\max} \cdot I_{\max} \cdot \omega_{\max}
    = (0.01745)^2 \times 4899 \approx 1.49 \text{ N·m}

    \|\boldsymbol{\tau}_{\text{gyro,2}}\| \le I_{\max}\,\|\boldsymbol{\omega}_{BN,B} \times \boldsymbol{\omega}_{RN,B}\|
    \le I_{\max} \cdot \omega_{\max}^2
    \approx 1.49 \text{ N·m}

The combined worst-case magnitude is therefore **2.98 N·m**, which is:

- 11.9 % of the 25 N·m X-axis torque limit
- 21.3 % of the 14 N·m Y/Z-axis torque limit
