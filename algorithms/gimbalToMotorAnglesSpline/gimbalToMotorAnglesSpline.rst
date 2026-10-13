.. raw:: latex

    {\LARGE \textbf{gimbalToMotorAnglesSpline}}

Executive Summary
-----------------

This module calculates the stepper motor angles :math:`(\theta_1, \theta_2)` for a set of gimbal angles
:math:`(\psi, \phi)`. The module calculates the value of one bicubic B-spline surface. An offline tool calculates
the surface from the measured gimbal-to-motor tables. The module does not keep the tables in memory. The surface
uses 729 coefficients (2.9 KB). The tables of :ref:`gimbalAnglesToMotorAngles` use 8172 values (32 KB).

The module accepts only gimbal angles in an addressable rhombus. For other gimbal angles, the module keeps the last
motor angles that it calculated.

Module Architecture
-------------------

The module has two layers:

- The **adapter** (``gimbalToMotorAnglesSpline.h`` and ``.cpp``) is a ``SysModel``. It reads and writes the
  messages. It makes a ``GimbalToMotorAnglesSplineConfig`` from the public properties. This configuration cannot
  change after the adapter makes it. The adapter makes the algorithm with two-phase initialization.
- The **algorithm** (``gimbalToMotorAnglesSplineAlgorithm.h`` and ``.cpp``) is a C++23 class that has no
  framework dependencies. It receives the gimbal angles and it gives the motor angles. ``update()`` does not throw.

A C shim (``gimbalToMotorAnglesSplineAlgorithm_c.h`` and ``.cpp``) gives access to the algorithm. Ada and
Adamant components use the shim through ``extern "C"`` bindings.

The offline fit tool is ``_tools/fitGimbalToMotorSpline.py``. It is not flight code.

Adapter Layer
-------------

.. list-table:: Module I/O Messages
    :widths: 24 40 36
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - twoAxisGimbalInMsg
      - :ref:`TwoAxisGimbalMsgF32Payload`
      - Input for the two gimbal angles
    * - motor1AngleOutMsg
      - :ref:`MotorAngleRefMsgF32Payload`
      - Output for the motor 1 angle
    * - motor2AngleOutMsg
      - :ref:`MotorAngleRefMsgF32Payload`
      - Output for the motor 2 angle

.. list-table:: Module Configuration Properties
    :widths: 24 20 8 8 20 20
    :header-rows: 1

    * - Parameter Name
      - Type
      - Units
      - Default
      - Description
      - Bounds
    * - centerTipAngle
      - float
      - [rad]
      - 0
      - Gimbal tip angle at the center of the addressable rhombus, :math:`\psi_0`
      - Must be a finite number. :math:`|\psi_0| + a \le \pi/2`
    * - centerTiltAngle
      - float
      - [rad]
      - 0
      - Gimbal tilt angle at the center of the addressable rhombus, :math:`\phi_0`. The two motors are mirror
        images about this tilt angle.
      - Must be a finite number. :math:`|\phi_0| + b \le \pi/2`
    * - halfDiagonalTip
      - float
      - [rad]
      - 0
      - Half of the rhombus diagonal along the gimbal tip axis, :math:`a`
      - Must be a finite number more than zero
    * - halfDiagonalTilt
      - float
      - [rad]
      - 0
      - Half of the rhombus diagonal along the gimbal tilt axis, :math:`b`
      - Must be a finite number more than zero
    * - splineCoefficients
      - std::array<float, 729>
      - [rad]
      - 0
      - Control coefficients :math:`c_{ij}` of the motor angle surface. Coefficient :math:`c_{ij}` is at index
        :math:`27 i + j`.
      - Each value must be a finite number

The fit tool calculates all the configuration values. The adapter examines the values at ``reset()``.

Algorithm Layer
---------------

Mathematical Formulation
^^^^^^^^^^^^^^^^^^^^^^^^

**Rhombus coordinates**

    The algorithm changes the gimbal angles to the normalized coordinates :math:`(u, v)`:

    .. math::

        u = \frac{\psi - \psi_0}{a} + \frac{\phi - \phi_0}{b}, \qquad
        v = \frac{\psi - \psi_0}{a} - \frac{\phi - \phi_0}{b}

    The addressable rhombus is the square :math:`|u| \le 1, |v| \le 1`. The algorithm accepts a point on the edge.
    It also accepts a small fp32 rounding error past the edge, :math:`10^{-5}` in :math:`u` and :math:`v`. This
    error is approximately :math:`3 \times 10^{-6}` rad of gimbal angle.

**Surface**

    The surface :math:`S(u, v)` is a uniform cubic B-spline. It has :math:`N = 24` intervals on :math:`[-1, 1]` in
    each coordinate. The knot spacing is :math:`h = 2/N`. The knot vector has three more knots past each end. Thus
    each coordinate has :math:`N + 3 = 27` coefficients. For a coordinate :math:`x`, the interval index :math:`k`
    and the local coordinate :math:`t` are:

    .. math::

        k = \min\left(\left\lfloor \frac{x + 1}{h} \right\rfloor, N - 1\right), \qquad t = \frac{x + 1}{h} - k

    The four basis weights are:

    .. math::

        B_0 = \frac{(1 - t)^3}{6}, \quad B_1 = \frac{3t^3 - 6t^2 + 4}{6}, \quad
        B_2 = \frac{-3t^3 + 3t^2 + 3t + 1}{6}, \quad B_3 = \frac{t^3}{6}

    The surface value is:

    .. math::

        S(u, v) = \sum_{p=0}^{3} \sum_{q=0}^{3} B_p(t_u) \, B_q(t_v) \, c_{k_u + p, \, k_v + q}

    Each weight is zero or more, and the sum of the weights is one. Thus each surface value is between the smallest
    coefficient and the largest coefficient.

**Motor angles**

    The measured tables show that motor 2 is the mirror image of motor 1 about the tilt center:
    :math:`\theta_2(\psi, \phi) = \theta_1(\psi, 2\phi_0 - \phi)`. The difference in the tables is less than
    0.01 deg. A reflection about :math:`\phi_0` interchanges :math:`u` and :math:`v`. Thus one surface gives the two
    motor angles:

    .. math::

        \theta_1 = S(u, v), \qquad \theta_2 = S(v, u)

**Requests that the algorithm does not accept**

    If a gimbal angle is not a finite number, the algorithm keeps the last motor angles that it calculated. If the
    point is outside the rhombus, the algorithm also keeps these motor angles. Until the algorithm calculates the
    first motor angles, the motor angles are 103.2242 deg.

Offline Fit
^^^^^^^^^^^

The tool ``_tools/fitGimbalToMotorSpline.py`` does these steps:

1. It reads the two measured tables (``_tests/gimbal_to_motor1_angle.csv`` and ``gimbal_to_motor2_angle.csv``).
   The tables use degrees on a 0.5 deg grid.
2. It finds the largest rhombus that has its center at zero tilt and that is in the measured data.
3. It removes a band from the two +tip edges of the rhombus. Near these edges, the motor angle changes very
   quickly with the gimbal angle. In the last grid cell, the change is more than 20 deg for each deg. A cubic
   surface cannot show this change accurately. The band width is 1.0 deg of tip angle at the tilt center.
4. It calculates the coefficients with linear least squares. The motor 1 samples are at :math:`(u, v)` and the
   motor 2 samples are at :math:`(v, u)`.
5. It writes the configuration and the test data files to ``_tests``.

.. _fig_gimbalDomain:

.. figure:: _Documentation/_Images/gimbalDomain.pdf
    :width: 60%
    :align: center

    Measured motor 1 angles, the largest inscribed rhombus, and the addressable rhombus.

.. _fig_fitError:

.. figure:: _Documentation/_Images/fitError.pdf
    :width: 60%
    :align: center

    Motor 1 fit error at the measured points in the addressable rhombus.

The table that follows shows the fit results for the values in this module.

.. list-table:: Fit Results
    :widths: 60 40
    :header-rows: 1

    * - Item
      - Value
    * - Addressable tip range
      - -17.60 deg to 17.25 deg
    * - Addressable tilt range
      - -26.10 deg to 26.10 deg
    * - Part of the inscribed rhombus area that is addressable
      - 94.5 %
    * - Maximum fit error at the measured points
      - 0.043 deg
    * - Maximum error at points that the fit did not use (10 random samples of 10 %)
      - 0.074 deg
    * - Maximum bilinear interpolation error in the same rhombus (estimate)
      - 0.100 deg

To make new coefficients, do the steps that follow:

1. Start the tool with these commands:

   .. code-block:: bash

       python fitGimbalToMotorSpline.py --sweep
       python fitGimbalToMotorSpline.py --band 1.0 --intervals 24 --emit ../_tests --figures ../_Documentation/_Images

2. Write the printed values into the configuration.
3. If you change the number of intervals, change ``NUM_SPLINE_INTERVALS``, ``NUM_SPLINE_COEFFS_PER_AXIS``, and
   ``NUM_SPLINE_COEFFS`` in ``gimbalToMotorAnglesSplineTypes.h``.

Algorithm Assumptions and Limitations
-------------------------------------

- The addressable rhombus is smaller than the measured data. The tool removes a 1.0 deg band at the +tip edges.
  The rhombus shape also removes approximately 0.9 deg on the -tip side, because the measured tilt vertices are at
  +0.5 deg tip.
- The two motors must be mirror images about the tilt center.
- The module does not set a travel range for the motor angles. Each motor angle is between the smallest
  coefficient and the largest coefficient.
- The fit accuracy is applicable only to the measured tables that the tool used. If the hardware calibration
  changes, make new coefficients.

Test Description
----------------

The unit tests compare the fp32 surface with a float64 scipy calculation of the same coefficients. The tests
calculate the tolerance from the size of the coefficients and from the maximum slope of the surface. The tests
also compare the surface with all measured points in the rhombus. Other unit tests examine the configuration
validation, the hold behavior, the reconfiguration, a linear surface, the convex-hull bound, the motor symmetry,
and the continuity at the interval edges. Fuzz tests examine random inputs and random configurations. Some of the
random inputs are not finite numbers. A Python test operates the adapter in a simulation.
