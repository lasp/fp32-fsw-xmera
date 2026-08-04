.. raw:: latex

    {\LARGE \textbf{thrustAxisToMotorAngles}}

Executive Summary
-----------------

Given an incoming body thrust reference direction :math:`{}^\mathcal{B}\hat{t}`, this module computes the
corresponding gimbal angles :math:`(\psi, \phi)` and stepper motor angles :math:`(\theta_1, \theta_2)` which align
the SEP gimbal thrust axis with the reference thrust direction. The gimbal angles are first resolved using a simple
DCM conversion. Using the gimbal angles, the motor angles are determined through interpolation, using provided
gimbal-to-motor lookup data tables. The user is required to set the fixed Direction Cosine Matrix
:math:`[\mathcal{MB}]`, which describes the attitude of the gimbal mount frame relative to the spacecraft body frame.
The two steps of this module are outlined in more detail in Figure 1 below.

.. _fig_axisToMotorAngles:

.. figure:: _Documentation/_Images/axisToMotorAngles.pdf
    :width: 75%
    :align: center

    Illustration of the algorithm flow.

Module Architecture
-------------------

The module is split into two layers:

- The **adapter** (``thrustAxisToMotorAngles.h``/``.cpp``) is the SysModel-derived class that handles message
  I/O, validates configuration, builds an immutable ``ThrustAxisToMotorAnglesConfig`` from public properties, and
  constructs the algorithm via two-phase initialization.
- The **algorithm** (``thrustAxisToMotorAnglesAlgorithm.h``/``.cpp``) is a pure C++23 class with no framework
  dependencies. It takes the commanded body-frame thrust direction as input, computes the gimbal and stepper motor
  angles, and returns a payload struct as output. It must not throw from ``update()``.

A pure-C shim (``thrustAxisToMotorAnglesAlgorithm_c.h``/``.cpp``) wraps the algorithm class for use by Ada/Adamant
components via ``extern "C"`` bindings.

Adapter Layer
-------------

The adapter consumes the following messages and public configuration properties:

.. list-table:: Module I/O Messages
    :widths: 20 30 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - thrustDirectionInMsg
      - :ref:`BodyHeadingMsgF32Payload`
      - Body thrust direction input (uses ``rHat_XB_B`` as :math:`{}^\mathcal{B}\hat{t}`)
    * - motor1AngleOutMsg
      - :ref:`HingedRigidBodyMsgF32Payload`
      - Output for motor 1 angles
    * - motor2AngleOutMsg
      - :ref:`HingedRigidBodyMsgF32Payload`
      - Output for motor 2 angles
    * - twoAxisGimbalOutMsg
      - :ref:`TwoAxisGimbalMsgF32Payload`
      - Output for gimbal tip and tilt angles

.. list-table:: Module Configuration Properties
    :widths: 20 15 10 10 30
    :header-rows: 1

    * - Parameter Name
      - Type
      - Units
      - Default
      - Description
    * - dcm_MB
      - Eigen::Matrix3f
      - [-]
      - Identity
      - Gimbal mount frame attitude DCM relative to the spacecraft body frame

Algorithm Layer
---------------

Mathematical Formulation
^^^^^^^^^^^^^^^^^^^^^^^^

Step 1: Map Incoming Thrust Direction to Required Gimbal Angles
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

The first step of the algorithm is to determine the gimbal angles :math:`(\psi, \phi)`  which correspond to the
incoming body thrust direction :math:`{}^\mathcal{B}\hat{t}`. Using the user-provided DCM :math:`[\mathcal{MB}]`,
the reference body thrust direction is first mapped to the gimbal mount frame:

.. math::

   {}^{\mathcal{M}} \hat{\boldsymbol{t}} = [\mathcal{MB}] {}^{\mathcal{B}} \hat{\boldsymbol{t}}

.. note::

    There are two geometric assumptions in the algorithm logic which are required for the EMA mission.

    (1) The instantaneous gimbal attitude relative to the mount frame :math:`[\mathcal{GM}]` is defined using a compound (1-2) Euler angle sequence.
    (2) The gimbal thrust axis is defined as the third mount frame axis.

Using a :math:`(1-2) = (\psi, \phi)` Euler angle sequence, the gimbal body frame attitude relative to the mount frame is:

.. math::

    [\mathcal{GM}] = [C_2(\phi)][C_1(\psi)] = \begin{bmatrix}
        \cos\phi & 0 & -\sin\phi \\
        0 & 1 & 0 \\
        \sin\phi & 0 & \cos\phi
    \end{bmatrix} \begin{bmatrix}
        1 & 0 & 0 \\
        0 & \cos\psi & \sin\psi \\
        0 & -\sin\psi & \cos\psi
    \end{bmatrix} = \begin{bmatrix}
        \cos \phi & \sin\phi \sin\psi & -\sin\phi \cos\psi \\
        0 & \cos\psi & \sin\psi \\
        \sin\phi & -\cos\phi \sin\psi & \cos\phi \cos\psi
    \end{bmatrix}

Summarizing the gimbal first performs a rotation by :math:`\psi` degrees about the mount frame first axis
:math:`\hat{\boldsymbol{m}}_1`, followed by a second rotation by :math:`\phi` degrees about the intermediate frame
(also equivalent to the final gimbal frame) second axis :math:`\hat{\boldsymbol{g}}_2`.

The known gimbal thrust axis :math:`{}^{\mathcal{M}} \hat{\boldsymbol{t}}` can be equated with the third row of the
:math:`[\mathcal{GM}]` DCM:

.. math::

    {}^{\mathcal{M}} \hat{\boldsymbol{g}}_3 = \begin{bmatrix}
        {}^{\mathcal{M}} \hat{\boldsymbol{g}}_{3_1} \\
        {}^{\mathcal{M}} \hat{\boldsymbol{g}}_{3_2} \\
        {}^{\mathcal{M}} \hat{\boldsymbol{g}}_{3_3}
    \end{bmatrix} = \begin{bmatrix}
        \sin\phi \\
        -\cos\phi \sin\psi \\
        \cos\phi \cos\psi
    \end{bmatrix}

The required gimbal tip and tilt angles to achieve the requested thrust direction are therefore:

.. math::

    \psi = \tan^{-1} \left( \frac{- \hat{\boldsymbol{g}}_{3_2}}{\hat{\boldsymbol{g}}_{3_3}} \right) \qquad \phi = \sin^{-1} \left( \hat{\boldsymbol{g}}_{3_1} \right)

Step 2: Map Gimbal Angles to Motor Angles
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

Next, the stepper motor angels corresponding to the obtained gimbal angles are determined using two gimbal-to-motor
angle lookup tables. One lookup table is provided for each stepper motor. The structure of these tables is
outlined in Figure 2.

.. _fig_lookupTableFormat:

.. figure:: _Documentation/_Images/lookupTableFormat.pdf
    :width: 75%
    :align: center

    Gimbal-to-motor angle lookup table data format.

Implicitly, each table varies the gimbal tip and tilt angles in ascending order along the columns and rows,
respectively. Both tables discretize the gimbal angles by the fixed value given by :math:`\delta = 0.5` degrees.
The motor angles corresponding to the discreet combination of gimbal angles are explicitly provided in each table.
Accordingly, the motor angles corresponding to the minimum gimbal angles are located in the upper-left entry of
each table and the angles corresponding to the maximum gimbal angles are located in the bottom-right entry of each
table.

**Determine Interpolation Type**

    Given the provided tables and the desired gimbal tip and tilt angles, the algorithm first determines whether
    no interpolation, linear interpolation, or bilinear interpolation is required to obtain the corresponding motor
    angles.

    - No interpolation: If both gimbal angles are exactly divisible by the discretization value :math:`\delta`, the corresponding motor angles can be directly extracted from the tables and no interpolation is required.

    - Linear interpolation: If only one of the gimbal angles is exactly divisible by :math:`\delta`, linear interpolation is required to obtain each motor angle.

    - Bilinear interpolation: If neither gimbal angle is divisible by :math:`\delta`, bilinear interpolation is required.

**Determine Bounding Angles for Interpolation**

    Next, upper and lower bounds for the gimbal angles are determined if interpolation is required.

    - Bilinear interpolation: The upper and lower bounds for both gimbal angles are determined as the nearest multiples of :math:`\delta`:

        .. math::

            \psi_{\text{L}} < \psi < \psi_{\text{U}} \quad \text{where} \quad \psi_{\text{L}} = \delta \left\lfloor \frac{\psi}{\delta} \right\rfloor, \quad \psi_{\text{U}} = \delta \left\lceil \frac{\psi}{\delta} \right\rceil \\
            \phi_{\text{L}} < \phi < \phi_{\text{U}} \quad\text{where} \quad \phi_{\text{L}} = \delta \left\lfloor \frac{\phi}{\delta} \right\rfloor, \quad \phi_{\text{U}} = \delta \left\lceil \frac{\phi}{\delta} \right\rceil \\

    - Linear interpolation: If linear interpolation is required, only the bounding gimbal angles must be determined for the angle which is not divisible by :math:`\delta` .

**Pull Motor Angles From Tables**

    Using the bounding gimbal angles, the corresponding bounding motor angles are extracted from the provided tables:

    - Bilinear interpolation:

        .. math::

            (\theta_{1, \text{LL}}, \theta_{2, \text{LL}}) = \texttt{pullAngles}(\psi_{\text{L}}, \phi_{\text{L}}) \\
            (\theta_{1, \text{LU}}, \theta_{2, \text{LU}}) = \texttt{pullAngles}(\psi_{\text{L}}, \phi_{\text{U}}) \\
            (\theta_{1, \text{UL}}, \theta_{2, \text{UL}}) = \texttt{pullAngles}(\psi_{\text{U}}, \phi_{\text{L}}) \\
            (\theta_{1, \text{UU}}, \theta_{2, \text{UU}}) = \texttt{pullAngles}(\psi_{\text{U}}, \phi_{\text{U}})

    - Linear interpolation:

        - For linear interpolation where the first gimbal angle :math:`\psi` is bounded:

        .. math::

            (\theta_{1, \text{L}}, \theta_{2, \text{L}}) = \texttt{pullAngles}(\psi_{\text{L}}, \phi) \\
            (\theta_{1, \text{U}}, \theta_{2, \text{U}}) = \texttt{pullAngles}(\psi_{\text{U}}, \phi)

        - For linear interpolation where the second gimbal angle :math:`\phi` is bounded:

        .. math::

            (\theta_{1, \text{L}}, \theta_{2, \text{L}}) = \texttt{pullAngles}(\psi, \phi_{\text{L}}) \\
            (\theta_{1, \text{U}}, \theta_{2, \text{U}}) = \texttt{pullAngles}(\psi, \phi_{\text{U}})


    Using the incoming gimbal angles, the :math:`\texttt{pullAngles}` method determines the integer indices required to
    pull the correct motor angles from the gimbal-to-motor angle lookup tables. The required table row and column
    indices :math:`i` and :math:`j` are determined as:

    .. math::
        (i, j) = \left( \frac{\phi + 55 \delta}{\delta}, \frac{\psi + 38 \delta}{\delta} \right)

    Note from the equation above that the incoming gimbal angles must be shifted to determine the correct row and
    column lookup table indices. The motor angles corresponding to zero gimbal angles are located at the center of the
    lookup tables (For :math:`(\psi, \phi) = (0,0)`, the required table row and column indices are
    :math:`(i,j) = (55, 38)`). Therefore, all incoming gimbal angles must be shifted in order to correctly determine
    the table indices.

**Perform Interpolation**

Using (1) the required gimbal angles :math:`(\psi, \phi)`, (2) bounding gimbal angles
:math:`(\psi_{\text{L}}, \psi_{\text{U}}, \phi_{\text{L}}, \phi_{\text{U}})`, and (3) the
bounding motor angles, interpolation is performed using either the linear or bilinear interpolation formula.

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

Test Description
----------------
