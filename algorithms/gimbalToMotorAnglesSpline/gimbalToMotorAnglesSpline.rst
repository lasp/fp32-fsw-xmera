.. raw:: latex

    {\LARGE \textbf{gimbalToMotorAnglesSpline}}

Executive Summary
-----------------

This module calculates the stepper motor angles :math:`(\theta_1, \theta_2)` for a set of gimbal angles
:math:`(\psi, \phi)`. Each motor has one bicubic B-spline surface that gives the motor steps. The module multiplies
the steps by the step angle to get the motor angles. An offline tool calculates the two surfaces from the measured
gimbal-to-motor-steps lookup table. The module does not keep the table in memory. The two surfaces use
:math:`2 \times 225 = 450` coefficients (1.8 KB), much less than the full grid of the lookup table. See
:ref:`gimbalToMotorAnglesSpline_memoryComparison`.

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
    * - stepAngle
      - float
      - [rad/step]
      - 0
      - Angle per motor step
      - Must be in :math:`[2\pi / 100000, 2\pi]`
    * - defaultMotor1Angle
      - float
      - [rad]
      - 0
      - Motor 1 angle output until the algorithm calculates the first motor angles, and after ``reInitialize()``
      - Must be in :math:`[-\pi, \pi]`
    * - defaultMotor2Angle
      - float
      - [rad]
      - 0
      - Motor 2 angle output until the algorithm calculates the first motor angles, and after ``reInitialize()``
      - Must be in :math:`[-\pi, \pi]`
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
      - Gimbal tilt angle at the center of the addressable rhombus, :math:`\phi_0`
      - Must be a finite number. :math:`|\phi_0| + b \le \pi/2`
    * - halfDiagonalTip
      - float
      - [rad]
      - 0
      - Half of the rhombus diagonal along the gimbal tip axis, :math:`a`. This is the maximum gimbal tip angle
        from the center.
      - Must be a finite number more than zero
    * - halfDiagonalTilt
      - float
      - [rad]
      - 0
      - Half of the rhombus diagonal along the gimbal tilt axis, :math:`b`. This is the maximum gimbal tilt angle
        from the center.
      - Must be a finite number more than zero
    * - motor1SplineCoefficients
      - std::array<float, 225>
      - [steps]
      - 0
      - Control coefficients :math:`c^{(1)}_{ij}` of the motor 1 step surface. Coefficient :math:`c^{(1)}_{ij}` is
        at index :math:`15 i + j`.
      - Each value must be in :math:`[-100000, 100000]`
    * - motor2SplineCoefficients
      - std::array<float, 225>
      - [steps]
      - 0
      - Control coefficients :math:`c^{(2)}_{ij}` of the motor 2 step surface. Coefficient :math:`c^{(2)}_{ij}` is
        at index :math:`15 i + j`.
      - Each value must be in :math:`[-100000, 100000]`

The fit tool calculates the rhombus and the coefficients. The step angle comes from the stepper motor. The adapter
examines the values at ``reset()``.

Algorithm Layer
---------------

Mathematical Formulation
^^^^^^^^^^^^^^^^^^^^^^^^

Rhombus Coordinates
~~~~~~~~~~~~~~~~~~~

The addressable rhombus has its center at the gimbal angles :math:`(\psi_0, \phi_0)`. The algorithm changes the
gimbal angles to the normalized coordinates :math:`(u, v)`:

.. math::

    u = \frac{\psi - \psi_0}{a} + \frac{\phi - \phi_0}{b}, \qquad
    v = \frac{\psi - \psi_0}{a} - \frac{\phi - \phi_0}{b}

:ref:`The rhombus mapping figure <fig_rhombusMapping>` shows the mapping in two steps. First, the algorithm moves
the rhombus center to the origin and divides each angle by its half-diagonal: :math:`x = (\psi - \psi_0)/a` and
:math:`y = (\phi - \phi_0)/b`. This gives the unit rhombus :math:`|x| + |y| \le 1`. Then the algorithm turns the
unit rhombus by 45 deg and multiplies it by :math:`\sqrt{2}`: :math:`u = x + y` and :math:`v = x - y`. The figure
shows :math:`v` on the horizontal axis and :math:`u` on the vertical axis, so that the second step is a rotation
without a reflection. Each edge of the rhombus goes to one edge of the square.

.. _fig_rhombusMapping:

.. figure:: _Documentation/_Images/rhombusMapping.*
    :width: 100%
    :align: center

    Mapping from the gimbal angles to the normalized coordinates, for an example rhombus that is not the rhombus of
    this module. The vertices :math:`A` to :math:`D`, the colored edges, the dashed lines of constant :math:`u` and
    :math:`v`, and the point :math:`P` are the same in the three panels.

The addressable rhombus is the square :math:`|u| \le 1, |v| \le 1`. The algorithm accepts a point on the edge. It
also accepts a small fp32 rounding error past the edge, :math:`10^{-5}` in :math:`u` and :math:`v`. This error is
approximately :math:`3 \times 10^{-6}` rad of gimbal angle.

The inverse mapping changes the normalized coordinates back to the gimbal angles. The sum and the difference of
:math:`u` and :math:`v` give :math:`x = (u + v)/2` and :math:`y = (u - v)/2`, thus:

.. math::

    \psi = \psi_0 + a \, \frac{u + v}{2}, \qquad
    \phi = \phi_0 + b \, \frac{u - v}{2}

The algorithm does not use the inverse mapping. The fit tool uses it to make the reference points and the figures,
and the tests use it to make gimbal angles at known points of the square, for example on its edges.

Surface
~~~~~~~

Each motor has its own surface: :math:`S_1(u, v)` for motor 1 and :math:`S_2(u, v)` for motor 2. The two surfaces
use the same knots and the same equations, and only their coefficients are different. This section writes
:math:`S(u, v)` and :math:`c_{ij}` for either surface.

The surface :math:`S(u, v)` is a uniform bicubic B-spline. The degree along each coordinate is :math:`d = 3`. Each
coordinate has :math:`N` intervals of the same length on :math:`[-1, 1]`. The interval length is :math:`h = 2/N`. The
surface value is:

.. math::

    S(u, v) = \sum_{p=0}^{d} \sum_{q=0}^{d} B_p(t_u) \, B_q(t_v) \, c_{k_u + p, \, k_v + q}

The terms are:

- :math:`k_u` and :math:`k_v` are the indices of the intervals that contain :math:`u` and :math:`v`.
- :math:`t_u` and :math:`t_v` are the local coordinates of :math:`u` and :math:`v` in these intervals.
- :math:`B_p` and :math:`B_q` are the basis weights along :math:`u` and along :math:`v`.
- :math:`c_{ij}` are the coefficients of the surface.

Thus the surface value is a weighted sum of a block of :math:`(d + 1) \times (d + 1) = 4 \times 4` coefficients. The weight of each
coefficient is the product of one weight along :math:`u` and one weight along :math:`v`. The algorithm uses the same
equations along :math:`u` and along :math:`v`. The subsections that follow explain each term.

Interval Index and Local Coordinate
"""""""""""""""""""""""""""""""""""

For a coordinate :math:`\xi` (:math:`u` or :math:`v`), the interval index :math:`k` and the local coordinate
:math:`t` are:

.. math::

    k = \min\left(\max\left(\left\lfloor \frac{\xi + 1}{h} \right\rfloor, 0\right), N - 1\right), \qquad
    t = \frac{\xi + 1}{h} - k

The floor :math:`\lfloor z \rfloor` is the largest integer that is not more than :math:`z`. For :math:`z \ge 0`, it
removes the fractional part, for example :math:`\lfloor 3.7 \rfloor = 3`.

The index :math:`k` tells which interval contains the sample. Interval :math:`k` starts at :math:`-1 + k h` and ends
at :math:`-1 + (k + 1) h`, with :math:`k` from 0 to :math:`N - 1`. The local coordinate :math:`t` tells how far the
sample is along that interval: :math:`t = 0` at the start of the interval and :math:`t = 1` at the end. The minimum
puts the sample :math:`\xi = 1` in the last interval with :math:`t = 1`, not in an interval :math:`N` that does not
exist. The maximum puts a sample that is slightly less than :math:`-1`, in the rounding error that the algorithm
accepts, in the first interval.

Basis Weights
"""""""""""""

The four basis weights are:

.. math::

    B_0 = \frac{(1 - t)^3}{6}, \quad B_1 = \frac{3t^3 - 6t^2 + 4}{6}, \quad
    B_2 = \frac{-3t^3 + 3t^2 + 3t + 1}{6}, \quad B_3 = \frac{t^3}{6}

On interval :math:`k`, the spline is a blend of four control points, the coefficients :math:`c_k` to
:math:`c_{k+3}`. The weight :math:`B_p(t)` is the part of the control point :math:`c_{k+p}` in the blend. Each
weight is the part of one basis function that is on this interval. :math:`B_3` is the first part of the basis
function that starts at this interval, and :math:`B_0` is the last part of the basis function that started three
intervals before. :ref:`The basis function figure <fig_basisFunctions>` shows these parts.

.. _fig_basisFunctions:

.. figure:: _Documentation/_Images/basisFunctions.*
    :width: 100%
    :align: center

    Left: the four basis functions that are not zero on interval :math:`k`. The thick parts are on interval
    :math:`k`. Right: the same parts as the weights :math:`B_0(t)` to :math:`B_3(t)`. The sum of the weights is one
    for each :math:`t`.

The four cubic polynomials come from these conditions:

- The sum of the four weights is one for each :math:`t`. If all the coefficients have the same value, the surface
  has this value everywhere. If a constant is added to all the coefficients, the same constant is added to the
  surface. Thus the spline does not add a bias to the motor steps.
- Each basis function is zero at its ends, and its slope and its curvature are also zero at its ends. Thus each
  basis function is zero outside its four intervals and joins zero smoothly. A coefficient changes the surface only
  on these four intervals. Thus only 16 coefficients are necessary for each point, and the fit is local: a measured
  point has an effect only on the coefficients near it.
- At a knot, the value, the slope, and the curvature of each basis function are the same on the two sides. Thus the
  spline and its first two derivatives are continuous at the knots. When the gimbal angles move across a knot, the
  surface does not jump, and its rate and its acceleration also do not jump.

A basis function has four cubic parts, thus 16 polynomial coefficients. The conditions at the two ends give 6
equations, and the conditions at the three knots inside give 9 equations. These 15 equations give the shape of the
basis function. The condition that the sum is one gives its scale. Thus the conditions give exactly one set of four
weights.

The table that follows gives the value, the first derivative, and the second derivative of each weight at the start
(:math:`t = 0`) and at the end (:math:`t = 1`) of an interval. The derivatives are with respect to :math:`t`.

.. list-table:: Weights and Their Derivatives at the Ends of an Interval
    :widths: 16 14 14 14 14 14 14
    :header-rows: 1

    * - Weight
      - :math:`B_p(0)`
      - :math:`B_p(1)`
      - :math:`B_p'(0)`
      - :math:`B_p'(1)`
      - :math:`B_p''(0)`
      - :math:`B_p''(1)`
    * - :math:`B_0`
      - :math:`1/6`
      - :math:`0`
      - :math:`-1/2`
      - :math:`0`
      - :math:`1`
      - :math:`0`
    * - :math:`B_1`
      - :math:`4/6`
      - :math:`1/6`
      - :math:`0`
      - :math:`-1/2`
      - :math:`-2`
      - :math:`1`
    * - :math:`B_2`
      - :math:`1/6`
      - :math:`4/6`
      - :math:`1/2`
      - :math:`0`
      - :math:`1`
      - :math:`-2`
    * - :math:`B_3`
      - :math:`0`
      - :math:`1/6`
      - :math:`0`
      - :math:`1/2`
      - :math:`0`
      - :math:`1`

Let :math:`f_k(t) = \sum_{p=0}^{d} B_p(t) \, c_{k+p}` be the spline along one coordinate on interval :math:`k`.
The table gives these values at the start of the interval:

.. math::

    f_k(0) = \frac{c_k + 4 c_{k+1} + c_{k+2}}{6}, \qquad
    f_k'(0) = \frac{c_{k+2} - c_k}{2}, \qquad
    f_k''(0) = c_k - 2 c_{k+1} + c_{k+2}

and these values at the end of the interval:

.. math::

    f_k(1) = \frac{c_{k+1} + 4 c_{k+2} + c_{k+3}}{6}, \qquad
    f_k'(1) = \frac{c_{k+3} - c_{k+1}}{2}, \qquad
    f_k''(1) = c_{k+1} - 2 c_{k+2} + c_{k+3}

The values at the end of interval :math:`k` are equal to the values at the start of interval :math:`k + 1`. This
shows that the spline and its first two derivatives are continuous at the knots. Because :math:`t = (\xi + 1)/h - k`,
the derivatives with respect to the coordinate :math:`\xi` are :math:`f_k'(t)/h` and :math:`f_k''(t)/h^2`.

Each weight is zero or more, and the sum of the weights is one. Thus each surface value is between the smallest
coefficient and the largest coefficient.

Coefficients
""""""""""""

The coefficients :math:`c_{ij}` are the control points of the surface. The index :math:`i` is along :math:`u` and
the index :math:`j` is along :math:`v`. Coefficient :math:`c_{ij}` is at index :math:`(N + d) i + j` in
``motor1SplineCoefficients`` and ``motor2SplineCoefficients``. The coefficients are in motor steps.

The :math:`N` intervals give :math:`N + 1` knots on :math:`[-1, 1]`. The knot vector has three more knots past each
end, thus :math:`N + 7` knots in total. Each cubic basis function is not zero on four intervals. Thus four basis
functions are not zero on each interval. On the first interval and on the last interval, three of these four
functions extend past the end of :math:`[-1, 1]`. The three extra knots give these functions. Because the knots are
uniform, all the intervals use the same four weight equations, and the sum of the weights is one everywhere in
:math:`[-1, 1]`. A clamped knot vector, with four equal knots at each end, needs different weight equations near the
ends.

Each basis function has one coefficient. A spline of degree :math:`d` on :math:`N + 2d + 1` knots has
:math:`(N + 2d + 1) - d - 1 = N + d` basis functions. This module uses :math:`N = 12` and :math:`d = 3`, thus each
coordinate has :math:`N + d = 15` coefficients, and each surface has :math:`15^2 = 225` coefficients.

The table that follows gives the sizes of the spline for :math:`N` intervals and the degree :math:`d`.

.. list-table:: Spline Sizes
    :widths: 46 30 24
    :header-rows: 1

    * - Quantity
      - Formula
      - Value for :math:`N = 12`, :math:`d = 3`
    * - Interval length :math:`h`
      - :math:`2 / N`
      - :math:`1/6`
    * - Knots on :math:`[-1, 1]`
      - :math:`N + 1`
      - 13
    * - Extra knots past each end
      - :math:`d`
      - 3
    * - Knots in total
      - :math:`N + 2d + 1`
      - 19
    * - Basis functions that are not zero on each interval
      - :math:`d + 1`
      - 4
    * - Coefficients for each coordinate
      - :math:`N + d`
      - 15
    * - Coefficients for each surface
      - :math:`(N + d)^2`
      - 225
    * - Coefficients for the two surfaces
      - :math:`2 (N + d)^2`
      - 450
    * - Coefficients for each surface value
      - :math:`(d + 1)^2`
      - 16

The spline does not go through its control points: the value at a knot, :math:`f_k(0)`, is a blend of three
coefficients, not one coefficient. For this reason, the coefficients come from a fit and are not the measured motor
steps. The section :ref:`gimbalToMotorAnglesSpline_fitMethod` explains the fit.

Motor Angles
~~~~~~~~~~~~

The value of each surface is the motor steps. The algorithm does not round the motor steps to whole steps. It
multiplies them by the step angle :math:`s` (``stepAngle``):

.. math::

    n_1 = S_1(u, v), \quad n_2 = S_2(u, v), \qquad \theta_1 = s \, n_1, \quad \theta_2 = s \, n_2

The two surfaces use the same point :math:`(u, v)`. Each coefficient is in :math:`[-100000, 100000]`, thus each
surface value is also in this range.

Requests That the Algorithm Does Not Accept
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If a gimbal angle is not a finite number, the algorithm keeps the last motor angles that it calculated. If the point
is outside the rhombus, the algorithm also keeps these motor angles. Until the algorithm calculates the first motor
angles, the motor angles are the default motor angles ``defaultMotor1Angle`` and ``defaultMotor2Angle``. The
algorithm sets the motor angles to the default motor angles when it is created, and ``reInitialize()`` sets them back
to the default motor angles of the current configuration. ``setConfig()`` keeps the last motor angles, thus new
default motor angles apply only after the next ``reInitialize()``. The adapter creates a new algorithm at each
``reset()``.

For the motors at zero gimbal angles, set each one to the
motor steps at :math:`\psi = \phi = 0` from the motor-steps-to-gimbal lookup table, multiplied by :math:`s`.

.. _gimbalToMotorAnglesSpline_fitMethod:

Fit Method
^^^^^^^^^^

The tool ``_tools/fitGimbalToMotorSpline.py`` does these steps:

1. It reads the gimbal-to-motor-steps lookup table (``_tests/table_gimbal_angles_to_motor_steps.xlsx``). The table
   gives the motor 1 and motor 2 steps on a uniform grid of gimbal tip angles :math:`\alpha = \psi` (columns) and
   gimbal tilt angles :math:`\beta = \phi` (rows). The populated cells form a rhombus.
2. It finds the addressable rhombus: the rhombus of largest area that is inscribed in the populated cells of the
   table. The center :math:`(\psi_0, \phi_0)` and the half-diagonals :math:`a` and :math:`b` are all free. The
   subsection :ref:`gimbalToMotorAnglesSpline_rhombus` explains the search.
3. It calculates the coefficients of the two surfaces with linear least squares. The subsection
   :ref:`gimbalToMotorAnglesSpline_leastSquares` explains this step.
4. It calculates the errors of the fit as gimbal angles with the motor-steps-to-gimbal lookup table
   (``_tests/table_motor_steps_to_gimbal_angles.xlsx``). The fit does not use this table. The subsection
   :ref:`gimbalToMotorAnglesSpline_numIntervals` explains the errors.
5. It writes the configuration and the test data files to ``_tests``. The section
   :ref:`gimbalToMotorAnglesSpline_runOfflineFit` explains how to run the tool.

.. _gimbalToMotorAnglesSpline_leastSquares:

Least-Squares Fit
~~~~~~~~~~~~~~~~~

The surface value is linear in the coefficients. Let :math:`\beta_i(\xi)` be the basis function of coefficient
:math:`i` along one coordinate, with :math:`i` from 0 to 14. On interval :math:`k`, :math:`\beta_{k+p}(\xi) =
B_p(t)` for :math:`p` from 0 to 3, and the other basis functions are zero. Thus each surface is:

.. math::

    S(u, v) = \sum_{i=0}^{14} \sum_{j=0}^{14} \beta_i(u) \, \beta_j(v) \, c_{ij}

The tool uses the :math:`M` table cells in the addressable rhombus. For cell :math:`m`, it changes the gimbal angles
:math:`(\psi_m, \phi_m)` to :math:`(u_m, v_m)`. The table steps are :math:`n_{1,m}` and :math:`n_{2,m}`. Each cell
gives one equation for each surface:

.. math::

    S_1(u_m, v_m) = n_{1,m}, \qquad S_2(u_m, v_m) = n_{2,m}

The tool writes the equations of each surface as one linear system :math:`A \mathbf{c} = \mathbf{n}`:

- :math:`\mathbf{c}` has the 225 coefficients of the surface. Coefficient :math:`c_{ij}` is at index
  :math:`15 i + j`, the same order as ``motor1SplineCoefficients`` and ``motor2SplineCoefficients``.
- Each row of :math:`A` is one cell. The row of cell :math:`m` has :math:`\beta_i(u_m) \beta_j(v_m)` in column
  :math:`15 i + j`.
- :math:`\mathbf{n}` has the table steps of one motor in the same order as the rows.

The two motors use the same cells, thus the two systems have the same matrix :math:`A`. The tool solves them together,
with one column of steps for each motor. Each row has only 16 values that are not zero, because only 16 basis
functions are not zero at a point. Thus :math:`A` has :math:`M` rows and 225 columns.

There are more equations than coefficients, and the table steps have errors. Thus no set of coefficients satisfies
all the equations. For each motor, the tool finds the coefficients that make the sum of the squared errors as small
as possible:

.. math::

    \min_{\mathbf{c}} \left\| A \mathbf{c} - \mathbf{n} \right\|^2
    = \min_{\mathbf{c}} \sum_{m=1}^{M} \left( S(u_m, v_m) - n_m \right)^2

The solution satisfies the normal equations :math:`A^T A \mathbf{c} = A^T \mathbf{n}`. The tool does not calculate
:math:`A^T A`. It uses ``numpy.linalg.lstsq``, which solves the problem with a singular value decomposition of
:math:`A`. This is more accurate, because the condition number of :math:`A^T A` is the square of the condition number
of :math:`A`. The tool prints the rank and the condition number of :math:`A`. If the rank is 225, the table cells
determine each coefficient.

The tool calculates the fit in motor steps and writes the coefficients in motor steps. The range of the coefficients
of each motor is wider than the range of its table steps in the rhombus. The basis
functions of the outer coefficients are mostly outside the rhombus. The table cells determine only the part of these
functions that is inside the rhombus, thus these coefficients can be outside the range of the table steps. Thus the
bound "between the smallest coefficient and the largest coefficient" is wider than the range of the table steps.

.. _gimbalToMotorAnglesSpline_rhombus:

Addressable Rhombus
~~~~~~~~~~~~~~~~~~~

The addressable rhombus is the rhombus of largest area :math:`2 a b` that is inscribed in the populated cells of the
gimbal-to-motor-steps table. A rhombus is inscribed if, at each tilt angle from :math:`\phi_0 - b` to
:math:`\phi_0 + b`, its tip range is inside the populated cells of the two table rows above and below that tilt.
The tool samples the rhombus at 2001 tilt angles for this check.

The tool writes the half-diagonals as :math:`a = s \cos\theta` and :math:`b = s \sin\theta`, so that
:math:`\theta` sets the shape and :math:`s` the size. For a center :math:`(\psi_0, \phi_0)` and a shape
:math:`\theta`, it finds the largest inscribed size :math:`s` by bisection. The area is then
:math:`2 a b = s^2 \sin 2\theta`. The edge of the populated cells is a staircase, thus the area has many local
maxima as a function of :math:`(\psi_0, \phi_0, \theta)`. The tool uses a global search
(``scipy.optimize.differential_evolution``, with a fixed seed) to find the center and the shape that give the largest
area. The center is searched within four table steps of the middle of the populated cells. The search takes
approximately 7 s, and different seeds give the same rhombus to 1e-6 deg.

The center of the rhombus is not exactly at zero gimbal angles, and the two half-diagonals are different. The tool
prints the center, the half-diagonals, and the radius of the largest circle about zero gimbal angles that the rhombus
contains.

The figures also show the required circle of radius 13.5 deg about zero gimbal angles. This radius is fixed in the
tool (``REQUIRED_RADIUS``), and it is not an option. Compare the printed radius with it to make sure that the
addressable rhombus contains all of this circle.

The rhombus reaches the edges of the populated table cells, where the table steps change quickly. The two lookup
tables also do not agree well near these edges, thus the largest gimbal angle errors of the tables are near the edges
of the rhombus.

.. _gimbalToMotorAnglesSpline_numIntervals:

Number of Intervals
~~~~~~~~~~~~~~~~~~~

The number of intervals :math:`N` sets the size of the surfaces. Each surface has :math:`(N + d)^2` coefficients, with
:math:`d = 3`. Each coefficient is one ``float`` of 4 bytes. The time to calculate one motor angle does not change
with :math:`N`, because the algorithm always uses :math:`(d + 1)^2 = 16` coefficients.

The command ``python fitGimbalToMotorSpline.py --sweep`` compares different values of :math:`N`. All the fits use
the same addressable rhombus and the same :math:`M` table cells. The errors are for the two motors together. The
lookup table data is not in the repository, thus this document does not give the values. The table that follows
gives the size of the surfaces, which does not depend on the data.

.. list-table:: Size of the Surfaces for Different Numbers of Intervals
    :widths: 20 40 40
    :header-rows: 1

    * - :math:`N`
      - Coefficients for each surface
      - Memory for the two surfaces [bytes]
    * - 4
      - 49
      - 392
    * - 6
      - 81
      - 648
    * - 8
      - 121
      - 968
    * - 10
      - 169
      - 1352
    * - **12**
      - **225**
      - **1800**
    * - 14
      - 289
      - 2312
    * - 16
      - 361
      - 2888
    * - 18
      - 441
      - 3528
    * - 20
      - 529
      - 4232

The sweep prints these metrics for each :math:`N`:

- **Equations for each coefficient** is the number of rows of :math:`A` divided by the number of coefficients of one
  surface, :math:`M / (N + d)^2`.
- **Condition number of** :math:`A`.
- **Coefficient range** is the smallest and the largest coefficient of the two surfaces. Each surface value is
  between the smallest and the largest coefficient of its surface.
- **Fitted table** errors are the errors at the gimbal-to-motor-steps table cells that the fit used.
- **Gimbal angles** errors give the fit errors as gimbal angles. For each fitted table cell, the tool calculates the
  surfaces at the gimbal angles of the cell. Then it finds the
  gimbal angles of these motor steps with the motor-steps-to-gimbal lookup table, which the fit does not use. The
  tool interpolates this table linearly between its grid points. The error is the distance between these gimbal
  angles and the gimbal angles of the cell, :math:`\sqrt{\Delta\psi^2 + \Delta\phi^2}`. Most of the error is
  along the tilt axis.
- **Not used cells** are the gimbal angle errors at cells that the fit did not use. The tool removes a random 10 % of
  the cells, fits the remaining cells, and calculates the gimbal angle error at the removed cells. It does this 10
  times with different random samples. The table gives the maximum and the RMS of all these errors. These errors
  show how accurate the surfaces are between the table cells. If the surfaces give motor steps outside the
  motor-steps-to-gimbal lookup table at a removed cell, the maximum and the RMS do not include this cell.
- **Table floor** is the error of the two lookup tables. The two tables do not agree exactly. The tool does the same
  calculation as for the gimbal angle errors, with the motor steps of the table in place of the surfaces. This error
  does not change with :math:`N`. The largest of these errors are near the edges of the rhombus (see
  :ref:`gimbalToMotorAnglesSpline_rhombus`). The fit does not follow each table cell exactly, thus its maximum gimbal
  angle error can be less than the maximum error of the tables.

When :math:`N` increases, the surfaces have more coefficients, thus the errors at the fitted cells decrease. The
gimbal angle errors use the fitted cells, thus they do not show the accuracy between the cells. The errors at the not
used cells show this accuracy, and after some value of :math:`N` more intervals do not make them smaller. For a large
:math:`N`, each interval contains only a small number of table cells, and some coefficients near the vertices of the
rhombus have almost no data. Then the condition number of :math:`A` and the coefficient range increase. The sweep
stops at :math:`N = 20`, because larger values give coefficients that increase quickly, up to the bound of 100000
steps.

This module uses :math:`N = 12`. Of the values in the sweep, it gives the smallest maximum gimbal angle error, and
the values of :math:`N` that give the same accuracy at the not used cells use more memory.

Fit Results
~~~~~~~~~~~

``--report`` prints the fit metrics for the values of this module: :math:`N = 12` intervals and the largest inscribed
rhombus. ``--figures`` also writes figures of the fit (see :ref:`gimbalToMotorAnglesSpline_runOfflineFit`). The
metrics and these figures come from the lookup table data, thus they are not in the repository.

.. _gimbalToMotorAnglesSpline_memoryComparison:

Memory Comparison
~~~~~~~~~~~~~~~~~

All values are ``float`` of 4 bytes. The spline keeps :math:`2 \times 225 = 450` coefficients and the center and the
2 half-diagonals of the addressable rhombus, thus 454 values (1816 bytes). This does not depend on the size of the
lookup table.

A gimbal-to-motor-steps lookup table of :math:`R` rows and :math:`C` columns keeps the full grid of each motor, thus
the algorithm can find a cell from its index. The gimbal angles of the grid change along one axis only, thus the
table keeps one vector of :math:`R` values and one vector of :math:`C` values for the gimbal angles. Thus the table
keeps :math:`2 R C + R + C` values. A table that keeps only the populated cells, or only the cells in the addressable
rhombus, is smaller, but it needs more memory to find a cell. The spline uses less memory than the full grid when
:math:`2 R C + R + C > 454`, for example for a grid of more than 15 by 15 cells.

The motor-steps-to-gimbal-angles table is not in this comparison, because the spline does not replace it.

Algorithm Assumptions and Limitations
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- The addressable rhombus is the largest rhombus inscribed in the populated cells of the table. The gimbal angles in
  the populated cells that are outside the rhombus are not addressable. The rhombus contains all the gimbal angles
  within the radius that the tool prints, but not all the gimbal angles within a larger radius.
- The module does not set a travel range for the motors. The steps of each motor are between the smallest and the
  largest coefficient of its surface.
- The motor steps are not whole steps. The component that moves the motors must round them, if necessary.
- The algorithm does not check the default motor angles against the surfaces. If the lookup table changes, calculate
  ``defaultMotor1Angle`` and ``defaultMotor2Angle`` again.
- The fit accuracy is applicable only to the lookup tables that the tool used. If the hardware calibration changes,
  make new coefficients.

.. _gimbalToMotorAnglesSpline_runOfflineFit:

Python Offline Fit Tool
-----------------------

The tool ``_tools/fitGimbalToMotorSpline.py`` is a Python script. It needs ``numpy``, ``scipy``, and ``openpyxl``.
The figures also need ``matplotlib``. The default paths of the lookup tables are relative to the tool, thus the tool
can start from any directory. The paths of ``--emit`` and ``--figures`` are relative to the current directory.

Each time the tool starts, it reads the two lookup tables, finds the largest inscribed rhombus (approximately 7 s),
and prints its center, its half-diagonals, and the radius of the largest circle about zero gimbal angles that it
contains. Then it does the actions that the options select. If no action is selected, the tool stops after this.

Command-Line Options
^^^^^^^^^^^^^^^^^^^^

.. list-table:: Lookup Table Options
    :widths: 26 30 44
    :header-rows: 1

    * - Option
      - Default
      - Description
    * - ``--steps-table PATH``
      - ``_tests/table_gimbal_angles_to_motor_steps.xlsx``
      - Gimbal-to-motor-steps lookup table that the tool fits. The sheets ``Alpha Grid`` and ``Beta Grid`` give the
        gimbal tip and tilt angles [deg]. The sheets ``M1`` and ``M2`` give the motor steps. An empty cell is not
        measured. The two motor sheets must have the same empty cells.
    * - ``--gimbal-table PATH``
      - ``_tests/table_motor_steps_to_gimbal_angles.xlsx``
      - Motor-steps-to-gimbal lookup table that the tool uses to find the gimbal angle errors. The sheets
        ``M1 Grid`` and ``M2 Grid`` give the motor steps. The sheets ``Alpha`` and ``Beta`` give the gimbal tip and
        tilt angles [deg].

.. list-table:: Fit Options
    :widths: 26 22 52
    :header-rows: 1

    * - Option
      - Default
      - Description
    * - ``--intervals N``
      - 12
      - Number of intervals :math:`N` on each coordinate

.. list-table:: Action Options
    :widths: 26 22 52
    :header-rows: 1

    * - Option
      - Default
      - Description
    * - ``--sweep``
      - Off
      - Print the fit metrics for :math:`N` = 4, 6, 8, ..., 20. This action ignores ``--intervals``.
    * - ``--report``
      - Off
      - Print the fit metrics for ``--intervals`` only
    * - ``--emit DIR``
      - None
      - Write the configuration and the test data files to ``DIR``
    * - ``--figures DIR``
      - None
      - Write the documentation figures to ``DIR``
    * - ``--switches PATH``
      - ``_tests/referenceSwitches.toml``
      - Reference switch positions [steps] that the domain figure shows, if the file exists. Each table
        ``[switches.<NAME>_<MOTOR>_<KIND>]`` gives the steps of motor 1 or 2 at the points of one switch, keyed by
        the point name. The tool uses only the tables with ``<KIND>`` ``N``, and pairs the motor 1 and motor 2 steps
        of each point.
    * - ``--show``
      - Off
      - Open the figures in interactive windows. This option can be used with ``--figures`` or without it.
    * - ``--num-reference COUNT``
      - 200
      - Number of random reference points that ``--emit`` writes
    * - ``--seed SEED``
      - 2026
      - Seed of the random reference points that ``--emit`` writes

``--sweep`` and ``--report`` print one row of a markdown table for each fit. The columns are :math:`N`, the number of coefficients of the two surfaces and their memory, the number of equations for each
coefficient, the rank and the condition number of :math:`A`, the coefficient range, the maximum tip and tilt
errors and the maximum, RMS, and 99th percentile of the gimbal angle error [deg], the maximum and RMS gimbal angle
error of the lookup tables [deg], the maximum and RMS of the error at the fitted table [steps], and the maximum and
RMS of the gimbal angle error at the not used cells [deg] with the number of these cells whose motor steps are
outside the motor-steps-to-gimbal lookup table. The subsection :ref:`gimbalToMotorAnglesSpline_numIntervals`
explains these metrics.

Output Files
^^^^^^^^^^^^

.. list-table:: Files That ``--emit`` Writes
    :widths: 30 70
    :header-rows: 1

    * - File
      - Contents
    * - ``splineCoefficientsMotor1.csv``
      - One row with the :math:`(N + d)^2` coefficients of the motor 1 surface [steps]. Coefficient :math:`c_{ij}` is
        at index :math:`(N + d) i + j`. These are the values of ``motor1SplineCoefficients``.
    * - ``splineCoefficientsMotor2.csv``
      - The same for the motor 2 surface. These are the values of ``motor2SplineCoefficients``.
    * - ``splineDomain.csv``
      - One row with :math:`\psi_0`, :math:`\phi_0`, :math:`a`, and :math:`b` [rad]. These are the values of
        ``centerTipAngle``, ``centerTiltAngle``, ``halfDiagonalTip``, and ``halfDiagonalTilt``.
    * - ``splineReference.csv``
      - Tip and tilt [rad], and :math:`S_1` and :math:`S_2` [steps], at ``--num-reference`` random points in the
        rhombus and at its four vertices. The surface values are float64 values from ``scipy``, not rounded.
    * - ``tablePointsInDomain.csv``
      - Tip and tilt [rad], and the motor 1 and motor 2 steps, of the gimbal-to-motor-steps table cells in the
        addressable rhombus

The tool also prints :math:`N`, the number of coefficients of each surface, and the maximum fit error.

The tool writes each figure as a PDF file and as an SVG file. The figures in this document use the extension
``.*``, thus Sphinx uses the SVG files for HTML and the PDF files for LaTeX.

.. list-table:: Files That ``--figures`` Writes
    :widths: 30 70
    :header-rows: 1

    * - File
      - Contents
    * - ``rhombusMapping.pdf`` and ``rhombusMapping.svg``
      - :ref:`The rhombus mapping figure <fig_rhombusMapping>`
    * - ``basisFunctions.pdf`` and ``basisFunctions.svg``
      - :ref:`The basis function figure <fig_basisFunctions>`
    * - ``gimbalDomain.pdf`` and ``gimbalDomain.svg``
      - Table steps of motor 1, the addressable rhombus, the required circle of radius 13.5 deg about zero gimbal
        angles, and the reference switch positions (N) from ``--switches``. The tool changes the motor steps of each
        switch position to gimbal angles with the motor-steps-to-gimbal lookup table.
    * - ``fitError.pdf`` and ``fitError.svg``
      - Gimbal angle error at the table cells in the addressable rhombus [deg], with the required circle
    * - ``fitSurface3d.pdf`` and ``fitSurface3d.svg``
      - Motor 1 spline surface in the addressable rhombus, colored by the gimbal angle error, with the table cells

``rhombusMapping`` and ``basisFunctions`` do not use the lookup table data, and this document shows them. The other
figures come from the lookup table data, thus they are not in the repository.

Make New Coefficients
^^^^^^^^^^^^^^^^^^^^^

To make new coefficients, do the steps that follow:

1. Go to the ``_tools`` directory. Examine the fit metrics for the numbers of intervals:

   .. code-block:: bash

       python fitGimbalToMotorSpline.py --sweep

2. Write the configuration, the test data files, and the figures for the number of intervals that you select:

   .. code-block:: bash

       python fitGimbalToMotorSpline.py --intervals 12 --emit ../_tests --figures ../_Documentation/_Images

   Add ``--show`` to also open the figures in interactive windows.

3. Set the configuration from ``_tests/splineDomain.csv``, ``_tests/splineCoefficientsMotor1.csv``, and
   ``_tests/splineCoefficientsMotor2.csv``. Set ``stepAngle`` from the stepper motor, and ``defaultMotor1Angle``
   and ``defaultMotor2Angle`` from the motor steps at zero gimbal angles.
4. If you change the number of intervals, change ``NUM_SPLINE_INTERVALS`` in ``gimbalToMotorAnglesSplineTypes.h``.
   ``NUM_SPLINE_COEFFS_PER_AXIS`` and ``NUM_SPLINE_COEFFS`` derive from it.
5. Do not commit the lookup tables, the files that ``--emit`` writes, the figures of the fit, or numbers from them
   (see :ref:`gimbalToMotorAnglesSpline_testData`).

Test Description
----------------

The tests are in ``_tests``. The C++ unit tests (``test_gimbalToMotorAnglesSpline.cpp``) and the fuzz tests
(``test_gimbalToMotorAnglesSpline_fuzz.cpp``) test the algorithm. The Python tests
(``test_gimbalToMotorAnglesSpline.py``) operate the adapter in a simulation. The fuzz tests are built only when
``XMERA_ENABLE_FUZZTESTS`` is on.

.. _gimbalToMotorAnglesSpline_testData:

Test Data and Tolerances
^^^^^^^^^^^^^^^^^^^^^^^^

The tests use the five files that ``--emit`` writes (see :ref:`gimbalToMotorAnglesSpline_runOfflineFit`). These files
come from the lookup table data, thus they are not in the repository: write them to ``_tests`` with the tool before
you run the tests. The fitted configuration of the tests is ``splineDomain.csv``, ``splineCoefficientsMotor1.csv``,
and ``splineCoefficientsMotor2.csv``, with the test step angle :math:`s = 0.01` rad for each step
(``kTestStepAngle``). Every test configuration has the default motor angles -0.8 rad for motor 1 and 1.2 rad for
motor 2 (``kTestDefaultMotor1Angle`` and ``kTestDefaultMotor2Angle``). They are different, thus the tests find a swap.

The tests compare the motor angles in motor steps. The function ``motorSteps`` divides each motor angle by :math:`s`.
The multiplication by :math:`s` in the algorithm and this division each add a relative fp32 rounding error of not
more than :math:`\epsilon/2`, thus the steps differ from the surface value by not more than :math:`\epsilon` times
the value (``kAngleConversionRelativeError``).

Some tests use a linear test configuration that does not depend on the files. Its rhombus has
:math:`\psi_0 = 0.01` rad, :math:`\phi_0 = 0`, :math:`a = 0.3` rad, and :math:`b = 0.45` rad. Its coefficients are the
Greville abscissae multiplied by 1000: :math:`c^{(1)}_{ij} = 1000 \, (-1 + (j - 1) h)` for motor 1 and
:math:`c^{(2)}_{ij} = 1000 \, (-1 + (i - 1) h)` for motor 2. With these coefficients, the surfaces are exactly
:math:`S_1(u, v) = 1000 \, v` and :math:`S_2(u, v) = 1000 \, u`, thus :math:`n_1 = 1000 \, v` and
:math:`n_2 = 1000 \, u` in the rhombus. The two motors have different surfaces, thus the tests
find a swap of the two coefficient sets. The motor steps are in :math:`[-1000, 1000]`.

Slope Bound
~~~~~~~~~~~

The function ``surfaceSlopeBound`` calculates a bound :math:`L` of the slope of one surface:

.. math::

    L = \frac{1}{h} \max_{i, j} \left\{ |c_{i+1, j} - c_{ij}|, \; |c_{i, j+1} - c_{ij}| \right\}

The slope is the derivative of the surface with respect to the normalized coordinates :math:`u` and :math:`v`, not
with respect to the gimbal angles. Its unit is motor steps for each unit of :math:`u` or :math:`v`.

The derivative of a cubic B-spline is a quadratic B-spline. Its coefficients are the differences of adjacent
coefficients. On interval :math:`k`, the derivatives of the weights in :math:`f_k(t) = \sum_{p=0}^{d} B_p(t) \,
c_{k+p}` give:

.. math::

    f_k'(t) = \frac{(1 - t)^2}{2} \, (c_{k+1} - c_k) + \frac{-2t^2 + 2t + 1}{2} \, (c_{k+2} - c_{k+1})
    + \frac{t^2}{2} \, (c_{k+3} - c_{k+2})

The three quadratic weights are zero or more, and their sum is one for each :math:`t`. Thus :math:`|f_k'(t)|` is not
more than the largest of the three differences. The derivative with respect to the coordinate is
:math:`f_k'(t)/h`. On the surface, the weights along the other coordinate are also zero or more, and their sum is
also one. Thus:

.. math::

    \left| \frac{\partial S}{\partial u} \right| \le \frac{1}{h} \max_{i, j} |c_{i+1, j} - c_{ij}|, \qquad
    \left| \frac{\partial S}{\partial v} \right| \le \frac{1}{h} \max_{i, j} |c_{i, j+1} - c_{ij}|

:math:`L` is the larger of these two bounds. The tests calculate :math:`L` for each surface and use it in two ways:

- For two points in the square, :math:`|S(u_2, v_2) - S(u_1, v_1)| \le L \, (|u_2 - u_1| + |v_2 - v_1|)`. The
  continuity test and the Lipschitz fuzz test use this bound for the motor steps. Their tolerance is

  .. math::

      1.1 \, L \, (|u_2 - u_1| + |v_2 - v_1|) + 2 \, T_s

  The factor 1.1 covers the rounding of the offsets when the tests map them to gimbal angles. :math:`T_s` is the
  step tolerance (see the next subsection), one for each of the two points.
- A rounding error in :math:`u` or :math:`v` changes the surface value by not more than :math:`L` multiplied by the
  rounding error. The fp32 calculation tolerance uses this bound.

The bound is conservative: for the fitted surfaces, the largest slopes on a dense grid of points are much less than
:math:`L`. The largest differences are between the outer coefficients, but the basis functions of these coefficients
are mostly outside the square. Also, the one bound :math:`L` is used for the two coordinates, but each surface
changes much less along one of them.

The slopes with respect to the gimbal angles come from the chain rule. For each motor:

.. math::

    \frac{\partial n}{\partial \psi} = \frac{1}{a} \left( \frac{\partial S}{\partial u}
    + \frac{\partial S}{\partial v} \right), \qquad
    \frac{\partial n}{\partial \phi} = \frac{1}{b} \left( \frac{\partial S}{\partial u}
    - \frac{\partial S}{\partial v} \right)

Tolerances
~~~~~~~~~~

The tests use these tolerances:

- **fp32 calculation tolerance.** The function ``evaluationTolerance`` calculates this tolerance for one surface from
  its coefficients:

  .. math::

      T = 3 \left( 20.5 \, \epsilon \max |c_{ij}| + 2 \cdot 9 \, \epsilon \, L \right)

  The first term is the rounding error of the coefficients, of the weights, and of the weighted sum. The second term
  is the rounding error of :math:`u` and of :math:`v`, approximately :math:`9 \epsilon` each, multiplied by the
  slope bound. The factor 3 is a margin. :math:`\epsilon` is the fp32 machine epsilon. The tests calculate :math:`T`
  from the coefficients, thus :math:`T` changes when the coefficients change.
- **Step tolerance.** The function ``stepTolerance`` gives :math:`T_s = T + \epsilon \max |c_{ij}|`: the fp32
  calculation tolerance plus the error of the change from motor steps to a motor angle and back. The tests use this
  tolerance to compare the motor steps with a float64 value of the same surface.
- **Linear surface tolerance.** :math:`3 \, (1 + h) \cdot 1000 \, (2 \cdot 20.5 \, \epsilon + \epsilon)`,
  approximately 0.018 steps. The coefficients of the linear surfaces are at most :math:`1000 \, (1 + h)` steps, and
  their slope is 1000 steps for each unit, thus the rounding of :math:`u` and :math:`v` adds about as much as the
  value error. The factor 3 is a margin, as for :math:`T`.
- **Fit tolerance.** The maximum fit error plus the larger step tolerance of the two motors. The test calculates the
  maximum fit error from the files: the largest difference between the float64 surfaces and the table cells in the
  rhombus. This is the maximum fit error that the tool prints.
- **Coefficient change.** The least-squares tests change coefficients by 10 to 50 steps. They calculate the sum of the
  squared errors with the float64 surface, not with the algorithm, because the fp32 calculation errors of the
  algorithm change the sum by more than a small coefficient change does. The tests read the coefficients and the table cells as ``float``,
  which moves the minimum of the sum a little. This adds a term to the change of the sum that is linear in the
  coefficient change. For a change of 10 steps, the quadratic increase of the sum is much larger than this term, also
  for the corner coefficients, which the table constrains the least.

Regression Tests
^^^^^^^^^^^^^^^^

- ``MatchesScipyReferenceEvaluation``
    - Compares the motor steps with ``splineReference.csv`` at 200 random points and at the four vertices of the
      rhombus. The reference values are float64 ``scipy`` values of the same coefficients. The tolerance is the step
      tolerance of each motor.

- ``MatchesMeasuredTableInsideDomain``
    - Compares the motor steps with all the table cells in the addressable rhombus (``tablePointsInDomain.csv``).
      The tolerance is the fit tolerance.

Hold Tests
^^^^^^^^^^

The hold tests, except the first one, use the linear test configuration. Each test first sends a point in the
rhombus and checks that :math:`n_1 = 1000 \, v` and :math:`n_2 = 1000 \, u`. Thus the test knows that the algorithm
calculated new motor angles. Then the test sends a request that the algorithm does not accept and checks that the
motor angles are exactly equal to the last motor angles.

- ``FirstCallOutsideDomainReturnsDefault``
    - Sends a point outside the rhombus first. Checks that the motor angles are exactly the default motor angles of
      the configuration.

- ``OutsideDomainHoldsPreviousValidOutput``
    - Sends points outside the rhombus at :math:`(u, v) = (1.5, 0)`, :math:`(0, -1.5)`, and :math:`(3, 3)`.

- ``NonFiniteInputHoldsPreviousValidOutput``
    - Sends gimbal angles that are NaN, :math:`+\infty`, or :math:`-\infty`, in one angle or in the two angles.

- ``OverflowingFiniteInputHoldsPreviousValidOutput``
    - Sends the largest fp32 values with opposite signs. These values are finite, but :math:`u` and :math:`v`
      become :math:`+\infty` and :math:`-\infty`, and their sum is NaN.

- ``RhombusEdgeIsAddressableAndJustOutsideIsNot``
    - Sends a point on each of the four edges (:math:`|u| = 1` or :math:`|v| = 1`) and checks the motor steps of
      the linear surfaces. Then sends the same point with the edge coordinate multiplied by :math:`1 + 10^{-3}`.
      Checks that the algorithm keeps the motor angles of the point on the edge.

- ``ReInitializeRestoresInitialOutput``
    - Sends a point outside the rhombus to a new algorithm to get its initial motor angles. Then sends a point in
      the rhombus to a second algorithm, calls ``reInitialize()``, and sends the point outside the rhombus. Checks
      that the motor angles are equal to the initial motor angles of the new algorithm.

- ``ReInitializeUsesReconfiguredDefaults``
    - Sets a configuration with different default motor angles before any point in the rhombus. Checks that a point
      outside the rhombus keeps the default motor angles of the first configuration. Then calls ``reInitialize()``
      and checks that a point outside the rhombus gives the default motor angles of the new configuration.

Configuration Tests
^^^^^^^^^^^^^^^^^^^

- ``SetConfigReplacesSurfaceAndKeepsPreviousOutput``
    - Calculates motor angles with the fitted configuration. Then sets a configuration with all the motor 1
      coefficients equal to 1250 steps and all the motor 2 coefficients equal to -2500 steps. Checks that a point
      outside the rhombus keeps the last motor angles of the fitted surfaces, and that a point in the rhombus gives
      exactly 1250 steps for motor 1 and -2500 steps for motor 2.

- ``ConfigRoundTrip``
    - Makes a configuration from the test step angle, the test default motor angles, the fitted rhombus, and the
      fitted coefficients of the two motors. Checks that the configuration gives the same values back.

- ``ConfigRejectsInvalidStepAngle``
    - Checks that the configuration accepts the step angles :math:`2\pi / 100000`, 0.01, and :math:`2\pi`. Checks
      that it does not accept a step angle one fp32 step past each limit, zero, a negative value, NaN, or
      :math:`\pm\infty`.

- ``ConfigRejectsInvalidDefaultMotorAngles``
    - For each motor, with the default motor angle of the other motor valid, checks that the configuration accepts
      zero, :math:`\pm\pi`, and the test value. Checks that it does not accept a value one fp32 step past
      :math:`\pm\pi`, NaN, or :math:`\pm\infty`.

- ``ConfigRejectsInvalidDomain``
    - Checks that a valid rhombus is accepted, and that a rhombus at the center with half-diagonals of
      :math:`\pi/2` and of the smallest positive fp32 value is accepted in tip and in tilt. Checks that a rhombus
      with a center of :math:`\pm 0.5` rad and a vertex exactly at :math:`\pm\pi/2` is accepted in tip and in tilt,
      and that the same rhombus with the half-diagonal one fp32 step larger is not. Checks that the configuration
      does not accept a rhombus with a value that is NaN or infinite, with a half-diagonal that is zero or less than
      zero, or that goes past :math:`\pm\pi/2` in tip or in tilt.

- ``ConfigRejectsInvalidCoefficients``
    - For each motor, with the coefficients of the other motor valid, sets the first, the center, or the last
      coefficient to a test value. Checks that the configuration accepts :math:`\pm 100000` steps. Checks that it does
      not accept a value one fp32 step past :math:`\pm 100000` steps, NaN, or :math:`\pm\infty`.

Property Tests
^^^^^^^^^^^^^^

- ``PropertyReproducesLinearSurface``
    - Uses the linear test configuration. Checks that :math:`n_1 = 1000 \, v` and :math:`n_2 = 1000 \, u` on a grid of
      points, within the linear surface tolerance. This test checks the basis weights, the order of the
      coefficients, and which coefficient set each motor uses.

- ``PropertyMotorStepsAreSurfacesAtUv``
    - Uses the fitted configuration. Evaluates :math:`S_1` and :math:`S_2` from their definition in float64, with
      the Cox-de Boor recursion on the extended knot vector. Checks on a grid of points that :math:`n_1 = S_1(u, v)`
      and :math:`n_2 = S_2(u, v)` within the step tolerance. Also checks
      that :math:`S_1(u, v)` and :math:`S_1(v, u)`, :math:`S_2(u, v)` and :math:`S_2(v, u)`, and :math:`S_1` and
      :math:`S_2` differ by more than 1000 times the tolerance somewhere on the grid, so that the test finds a swap
      of :math:`u` and :math:`v` in either motor and a swap of the two coefficient sets.

- ``PropertyOutputWithinCoefficientBounds``
    - Calls ``propertyOutputWithinCoefficientBounds`` with the fitted configuration on a grid of points. Checks
      that the motor steps of each motor are between the smallest and the largest coefficient of its surface, with
      the fp32 calculation and angle change errors.

- ``PropertyOutputIsFiniteForAnyInput``
    - Calls ``propertyOutputIsFiniteForAnyInput`` for each pair of NaN, :math:`\pm\infty`, the largest fp32
      value with each sign, the smallest subnormal value, zero, and two angles in the rhombus.

- ``PropertyOutsideDomainHoldsPreviousOutput``
    - Calls ``propertyOutsideDomainHoldsPreviousOutput`` with two points in the rhombus, one of them a vertex,
      and four points outside it, from 1 % past the edge to :math:`|u| = |v| = 50`.

- ``PropertyFittedCoefficientsMinimizeTableError``
    - Calculates the sum of the squared errors at the table cells for the fitted coefficients of the two motors,
      with the float64 surfaces. Changes all the coefficients by 10 steps with alternating signs, for motor 1, for
      motor 2, and for the two motors together (``propertyPerturbedCoefficientsIncreaseTableError``). Checks that
      each change makes the sum of the squared errors larger. This test checks that the coefficients are the least-squares solution.

- ``PropertySurfaceIsContinuousAcrossIntervalEdges``
    - At each knot inside :math:`[-1, 1]`, calls ``propertySurfaceIsLipschitz`` with the points :math:`10^{-4}`
      before and after the knot, along :math:`u` and along :math:`v`. Checks that the change of the motor steps of
      each motor is not more than the slope bound of its surface multiplied by the distance, plus the step
      tolerance of the two points.

Fuzz Tests
^^^^^^^^^^

Each fuzz test calls the same property function as a unit test, from ``gimbalToMotorAnglesSplineTestHelpers.hpp``,
with inputs that the fuzzer generates.

- ``propertyOutputIsFiniteForAnyInput``
    - Sends any two fp32 values, including NaN, infinite values, and very large values. Checks that the motor
      angles are finite. Checks that they are equal to the initial motor angles of a new algorithm, or that their
      motor steps are between the smallest and the largest coefficient of each motor.

- ``propertyOutsideDomainHoldsPreviousOutput``
    - Sends a random point in the rhombus and then a random point outside it, with :math:`|u|` or :math:`|v|` up
      to 50. Checks that the algorithm keeps the motor angles of the first point.

- ``propertySurfaceIsLipschitz``
    - Sends two random points in the rhombus that are not more than 0.01 apart in :math:`u` and in :math:`v`.
      Checks that the change of the motor steps of each motor is not more than the slope bound of its surface
      multiplied by the distance, plus the step tolerance of the two points.

- ``propertyOutputWithinCoefficientBounds``
    - Makes a random configuration: a center from -0.5 rad to 0.5 rad, half-diagonals from 0.01 rad to 1 rad, and
      coefficients of each motor from -100000 steps to 100000 steps, the full range that the configuration accepts.
      Sends a random point in its rhombus. Checks that the steps of each motor are between the smallest and the largest coefficient of its
      surface.

- ``propertyPerturbedCoefficientsIncreaseTableError``
    - Changes a random set of coefficients of each motor by random values from 10 steps to 50 steps, with random
      signs. Checks that the sum of the squared errors at the table cells becomes larger.

Python Adapter Tests
^^^^^^^^^^^^^^^^^^^^

The GTest suite tests the algorithm. The Python test only checks the adapter interface.

- ``test_gimbal_to_motor_angles_spline``
    - Sets the adapter properties from the fitted configuration, with the step angle 0.01 rad for each step, and
      writes one gimbal command at :math:`t = 0` to ``twoAxisGimbalInMsg``. Checks that ``motor1AngleOutMsg`` and
      ``motor2AngleOutMsg`` give motor steps within 0.5 steps of the scipy values of the first row of
      ``splineReference.csv``.
