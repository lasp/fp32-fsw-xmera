.. raw:: latex

    {\LARGE \textbf{gimbalAnglesToMotorAngles}}

Executive Summary
-----------------

Given an incoming set of gimbal angles :math:`(\psi, \phi)`, this module computes the corresponding stepper motor
angles :math:`(\theta_1, \theta_2)` which map to the requested gimbal angles. The motor angles are determined through
interpolation, using provided gimbal-to-motor lookup data tables. The algorithm flow is outlined in Figure 1 below.

.. _fig_algorithmFlow:

.. figure:: _Documentation/_Images/algorithmFlow.pdf
    :width: 50%
    :align: center

    Illustration of the algorithm flow.

Module Architecture
-------------------

The module is split into two layers:

- The **adapter** (``gimbalAnglesToMotorAngles.h``/``.cpp``) is the SysModel-derived class that handles message
  I/O, validates configuration, builds an immutable ``GimbalAnglesToMotorAnglesConfig`` from public properties, and
  constructs the algorithm via two-phase initialization.
- The **algorithm** (``gimbalAnglesToMotorAnglesAlgorithm.h``/``.cpp``) is a pure C++23 class with no framework
  dependencies. It takes the requested gimbal angles as input, computes the corresponding stepper motor angles,
  and returns a payload struct as output. It must not throw from ``update()``.

A pure-C shim (``gimbalAnglesToMotorAnglesAlgorithm_c.h``/``.cpp``) wraps the algorithm class for use by Ada/Adamant
components via ``extern "C"`` bindings.

Adapter Layer
-------------

The adapter consumes the following messages and public configuration properties:

.. list-table:: Module I/O Messages
    :widths: 24 40 36
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - twoAxisGimbalOutMsg
      - :ref:`TwoAxisGimbalMsgF32Payload`
      - Input for the two gimbal angles
    * - motor1AngleOutMsg
      - :ref:`HingedRigidBodyMsgF32Payload`
      - Output for motor 1 angles
    * - motor2AngleOutMsg
      - :ref:`HingedRigidBodyMsgF32Payload`
      - Output for motor 2 angles

.. list-table:: Module Configuration Properties
    :widths: 30 17 6 8 20 19
    :header-rows: 1

    * - Parameter Name
      - Type
      - Units
      - Default
      - Description
      - Bounds
    * - minAngle
      - float
      - [rad]
      - 0
      - Lower bound of the motor's travel range; reference angles below this are rejected
      - Must be in :math:`[-2\pi,\, 2\pi]` and strictly less than ``maxAngle`` (validated at reset())
    * - maxAngle
      - float
      - [rad]
      - :math:`2\pi`
      - Upper bound of the motor's travel range; reference angles above this are rejected
      - Must be in :math:`[-2\pi,\, 2\pi]` and strictly greater than ``minAngle`` (validated at reset())
    * - gimbalToMotor1AngleData
      - std::array<float, 4086>
      - [rad]
      - [-]
      - Gimbal-to-motor 1 angle interpolation table
      - Values must be within the provided motor angle range (minAngle, maxAngle). Gimbal angles cannot exceed :math:`\pm` 90 degrees
    * - gimbalToMotor2AngleData
      - std::array<float, 4086>
      - [rad]
      - [-]
      - Gimbal-to-motor 2 angle interpolation table
      - Values must be within the provided motor angle range (minAngle, maxAngle). Gimbal angles cannot exceed :math:`\pm` 90 degrees
    * - rowStartStrideIndices
      - std::array<int, 109>
      - [-]
      - [-]
      - Stride indices for the starting location of the table rows
      - Values cannot be negative and entries must be ascending
    * - rowStartColIndices
      - std::array<int, 109>
      - [-]
      - [-]
      - Column indices for the starting location of the table rows
      - Values cannot be negative or exceed the number of table columns
    * - tipColIdxOffset
      - int
      - [-]
      - [-]
      - Table index corresponding to zero gimbal angle 1
      - Cannot exceed number of table columns
    * - tiltRowIdxOffset
      - int
      - [-]
      - [-]
      - Table index corresponding to zero gimbal angle 2
      - Cannot exceed number of table rows
    * - tableStepAngle
      - float
      - [rad]
      - [-]
      - Interpolation table motor discretization step angle
      - Cannot be negative or zero

Algorithm Layer
---------------

Mathematical Formulation
^^^^^^^^^^^^^^^^^^^^^^^^

The stepper motor angels corresponding to the requested gimbal angles are determined using two gimbal-to-motor
angle lookup tables. One lookup table is provided for each stepper motor. The structure of these tables is
outlined in Figure 2.

.. _fig_lookupTableFormat:

.. figure:: _Documentation/_Images/lookupTableFormat.pdf
    :width: 75%
    :align: center

    Gimbal-to-motor angle lookup table data format.

Implicitly, each table varies the gimbal tip and tilt angles in ascending order along the columns and rows,
respectively. Both tables discretize the gimbal angles by the fixed configuration input value given.
The motor angles corresponding to the discreet combination of gimbal angles are explicitly provided in each table.
Accordingly, the motor angles corresponding to the minimum gimbal angles are located in the upper-left entry of
each table and the angles corresponding to the maximum gimbal angles are located in the bottom-right entry of each
table. The bilinear interpolation helper function is used to obtain the motor angles.

**Determine Bounding Angles for Interpolation**

    First, the upper and lower bounds for both gimbal angles are determined as the nearest multiple of the fixed
    table step angle :math:`\delta`.

    .. math::

        \psi_{\text{L}} \leq \psi \leq \psi_{\text{U}} \quad \text{where} \quad \psi_{\text{L}} = \delta \left\lfloor \frac{\psi}{\delta} \right\rfloor, \quad \psi_{\text{U}} = \delta \left\lceil \frac{\psi}{\delta} \right\rceil \\
        \phi_{\text{L}} \leq \phi \leq \phi_{\text{U}} \quad \text{where} \quad \phi_{\text{L}} = \delta \left\lfloor \frac{\phi}{\delta} \right\rfloor, \quad \phi_{\text{U}} = \delta \left\lceil \frac{\phi}{\delta} \right\rceil \\

**Pull Motor Angles From Tables**

    Using the bounding gimbal angles, the corresponding bounding motor angles are extracted from the provided tables
    using the :math:`\texttt{pullAngles}` method.

    .. math::

        (\theta_{1, \text{LL}}, \theta_{2, \text{LL}}) = \texttt{pullAngles}(\psi_{\text{L}}, \phi_{\text{L}}) \\
        (\theta_{1, \text{LU}}, \theta_{2, \text{LU}}) = \texttt{pullAngles}(\psi_{\text{L}}, \phi_{\text{U}}) \\
        (\theta_{1, \text{UL}}, \theta_{2, \text{UL}}) = \texttt{pullAngles}(\psi_{\text{U}}, \phi_{\text{L}}) \\
        (\theta_{1, \text{UU}}, \theta_{2, \text{UU}}) = \texttt{pullAngles}(\psi_{\text{U}}, \phi_{\text{U}})


    The :math:`\texttt{pullAngles}` method first determines the integer table indices required to pull the correct
    motor angles from the lookup tables. The required table row and column indices :math:`i` and :math:`j` are
    determined as:

    .. math::
        (i, j) = \left( \frac{\phi + \text{tiltRowIdxOffset} \delta}{\delta}, \frac{\psi + \text{tipColIdxOffset} \delta}{\delta} \right)

    Note from the equation above that the incoming gimbal angles must be shifted to determine the correct row and
    column lookup table indices. The motor angles corresponding to zero gimbal angles are located at the center of the
    lookup tables (For :math:`(\psi, \phi) = (0,0)`, the required table row and column indices are
    :math:`(i,j) = (\text{tiltRowIdxOffset}, \text{tipColIdxOffset})`).

**Perform Interpolation**

Using (1) the required gimbal angles :math:`(\psi, \phi)`, (2) bounding gimbal angles
:math:`(\psi_{\text{L}}, \psi_{\text{U}}, \phi_{\text{L}}, \phi_{\text{U}})`, and (3) the
bounding motor angles, interpolation is performed using the bilinear interpolation helper method.
Note that additional logic is implemented in the helper method if either linear interpolation or no interpolation are
instead required. The bilinear and linear interpolation formula are shown below.

- Bilinear interpolation:

    .. math::

        \begin{aligned}
        \theta = \frac{1}{ (\psi_{\text{U}} - \psi_{\text{L}}) (\phi_{\text{U}} - \phi_{\text{L}}) } \Big(
        & \theta_{\text{LL}} (\psi_{\text{U}} - \psi)(\phi_{\text{U}} - \phi)
        + \theta_{\text{UL}} (\psi - \psi_{\text{L}}) (\phi_{\text{U}} - \phi) \\
        &+ \theta_{\text{LU}} (\psi_{\text{U}} - \psi) (\phi - \phi_{\text{L}})
        + \theta_{\text{UU}} (\psi - \psi_{\text{L}}) (\phi - \phi_{\text{L}}) \Big)
        \end{aligned}

- Linear interpolation:

    .. math::
        \theta = \frac{ \theta_{\text{L}} (\beta_{\text{U}} - \beta) }{ \beta_{\text{U}} - \beta_{\text{L}} } + \frac{ \theta_{\text{U}} (\beta - \beta_{\text{L}}) }{ \beta_{\text{U}} - \beta_{\text{L}} }

where :math:`\beta = \psi` or :math:`\phi` depending on the bounded gimbal angle.


Algorithm Assumptions and Limitations
-------------------------------------
- All bounding gimbal angles are required to fall within the lookup table bounds. Requests which fall along an edge of
  the interpolation table are invalid.
- For invalid gimbal angle requests, the previously output valid motor angles are returned as the algorithm default
  output. On the first pass, an invalid request returns the motor angles corresponding to the gimbal home (0,0)
  position.

Test Description
----------------

The module is verified through regression tests that compare the algorithm results against an independent reference
implementation. Setup tests are used for the ``GimbalAnglesToMotorAnglesConfig`` validators and round-trip tests are
used to check the set configuration variables are correctly returned. Fuzz tests are added for the regression and
property tests, where the configuration and inputs are randomized over reasonable ranges. The fuzz tests generate
independent lookup tables and associated table layout data. A second test file
``test_gimbalAnglesToMotorAnglesImportTables`` is added which imports the lookup table data and associated table
layout data from CSV files. Hand-pulled data from the CSV files is used to perform the reference interpolation and the
results are compared with the algorithm results.
