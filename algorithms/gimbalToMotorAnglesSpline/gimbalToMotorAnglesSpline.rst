.. raw:: latex

    {\LARGE \textbf{gimbalToMotorAnglesSpline}}

Executive Summary
-----------------

This module calculates the stepper motor angles :math:`(\theta_1, \theta_2)` for a set of gimbal angles
:math:`(\psi, \phi)`. Each motor has one bicubic B-spline surface that gives the motor steps. The module multiplies
the steps by the step angle to get the motor angles. The module accepts only gimbal angles in an addressable rhombus.
For other gimbal angles, the module keeps the last motor angles that it calculated.

An offline tool calculates the two surfaces from the measured gimbal-to-motor-steps lookup table. The module does not
keep the table in memory. The two surfaces use :math:`2 \times 81 = 162` coefficients (648 bytes), much less than the
full grid of the lookup table (see :ref:`gimbalToMotorAnglesSpline_memoryComparison`).

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
    * - stepsPerRevolution
      - uint32_t
      - [steps]
      - 0
      - Number of motor steps per :math:`2\pi` rad
      - Must be in :math:`[1, 100000]`
    * - halfDiagonalTip
      - float
      - [rad]
      - 0
      - Half of the rhombus diagonal along the gimbal tip axis, :math:`a`. This is the maximum gimbal tip angle.
      - Must be in :math:`(0, \pi/2]`
    * - halfDiagonalTilt
      - float
      - [rad]
      - 0
      - Half of the rhombus diagonal along the gimbal tilt axis, :math:`b`. This is the maximum gimbal tilt angle.
      - Must be in :math:`(0, \pi/2]`
    * - thetaMax
      - float
      - [rad]
      - 0
      - Maximum requested thrust cone angle, :math:`r`: the radius of the required circle of gimbal angles about
        zero
      - Must be more than zero and not more than :math:`a b / \sqrt{a^2 + b^2}`, the radius of the largest circle
        in the rhombus, calculated in fp32
    * - motor1SplineCoefficients
      - std::array<float, 81>
      - [steps]
      - 0
      - Control coefficients :math:`c^{(1)}_{ij}` of the motor 1 step surface. Coefficient :math:`c^{(1)}_{ij}` is
        at index :math:`9 i + j`.
      - Each value must be in :math:`[-100000, 100000]`
    * - motor2SplineCoefficients
      - std::array<float, 81>
      - [steps]
      - 0
      - Control coefficients :math:`c^{(2)}_{ij}` of the motor 2 step surface. Coefficient :math:`c^{(2)}_{ij}` is
        at index :math:`9 i + j`.
      - Each value must be in :math:`[-100000, 100000]`

The fit tool calculates the rhombus and the coefficients. The steps per revolution come from the stepper motor. The
adapter examines the values at ``reset()``.

Algorithm Layer
---------------

Mathematical Formulation
^^^^^^^^^^^^^^^^^^^^^^^^

Rhombus Coordinates
~~~~~~~~~~~~~~~~~~~

The gimbal angles are the tip angle :math:`\psi` and the tilt angle :math:`\phi`, in radians. :math:`\psi` is
``theta1`` of ``twoAxisGimbalInMsg``, the rotation about the first gimbal axis, and :math:`\phi` is ``theta2``, the
rotation about the second gimbal axis. The addressable rhombus has its center at zero gimbal angles, and its diagonals
are along the tip and tilt axes: :math:`a` is half of the diagonal along the tip axis (``halfDiagonalTip``), and
:math:`b` is half of the diagonal along the tilt axis (``halfDiagonalTilt``). The algorithm changes the gimbal angles
to the normalized coordinates :math:`(u, v)`:

.. math::

    u = \frac{\psi}{a} + \frac{\phi}{b}, \qquad
    v = \frac{\psi}{a} - \frac{\phi}{b}

:ref:`The rhombus mapping figure <fig_rhombusMapping>` shows the mapping in two steps. First, the algorithm divides
each angle by its half-diagonal: :math:`x = \psi/a` and :math:`y = \phi/b`. This gives the unit rhombus
:math:`|x| + |y| \le 1`. Then the algorithm turns the unit rhombus by 45 deg and multiplies it by :math:`\sqrt{2}`:
:math:`u = x + y` and :math:`v = x - y`. The figure shows :math:`v` on the horizontal axis and :math:`u` on the vertical
axis, so that the second step is a rotation without a reflection. Each edge of the rhombus goes to one edge of the
square.

.. _fig_rhombusMapping:

.. figure:: _Documentation/_Images/rhombusMapping.*
    :width: 100%
    :align: center

    Mapping from the gimbal angles to the normalized coordinates, for an example rhombus that is not the rhombus of
    this module. The vertices :math:`A` to :math:`D`, the colored edges, the dashed lines of constant :math:`u` and
    :math:`v`, and the point :math:`P` are the same in the three panels.

The addressable rhombus is the square :math:`|u| \le 1, |v| \le 1`. The algorithm accepts a point on the edge, and
rejects a point past the edge. The fp32 calculation of :math:`u` and :math:`v` has a rounding error, thus a point
near the edge can be accepted or rejected. The subsection :ref:`gimbalToMotorAnglesSpline_rhombus` explains how the
size of the rhombus keeps this rounding away from the requests that the module assumes.

The inverse mapping changes the normalized coordinates back to the gimbal angles. The sum and the difference of
:math:`u` and :math:`v` give :math:`x = (u + v)/2` and :math:`y = (u - v)/2`, thus:

.. math::

    \psi = a \, \frac{u + v}{2}, \qquad
    \phi = b \, \frac{u - v}{2}

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

Thus the surface value is a weighted sum of a block of :math:`(d + 1) \times (d + 1) = 4 \times 4` coefficients. The
weight of each coefficient is the product of one weight along :math:`u` and one weight along :math:`v`. The algorithm
uses the same equations along :math:`u` and along :math:`v`. The subsections that follow explain each term.

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
exist.

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

An important property arises from the fact that each weight is non-negative, and the sum of the weights is one: the
surface value at any point must be between the smallest coefficient and the largest coefficient. This property is used
in the testing suite as a property of this module.

Coefficients
""""""""""""

The coefficients :math:`c_{ij}` are the control points of the surface. The index :math:`i` is along :math:`u` and
the index :math:`j` is along :math:`v`. Coefficient :math:`c_{ij}` is at index :math:`(N + d) i + j` in
``motor1SplineCoefficients`` and ``motor2SplineCoefficients``. Because the surface maps gimbal angles to motor steps,
the coefficients are in motor steps.

The knots are the ends of the intervals, at :math:`\xi = -1 + k h`. The :math:`N` intervals give :math:`N + 1` knots
on :math:`[-1, 1]`. Each cubic basis function is not zero on four intervals in a row, thus the basis functions of the
intervals near :math:`-1` and :math:`1` start or end up to three intervals past :math:`[-1, 1]`. The knot vector
therefore has :math:`d = 3` more knots past each end, with the same spacing :math:`h`, and :math:`N + 2d + 1` knots in
total. Because all the knots have the same spacing, every interval uses the same four weight equations.

Each basis function has one coefficient. A spline of degree :math:`d` on :math:`N + 2d + 1` knots has
:math:`(N + 2d + 1) - d - 1 = N + d` basis functions. This module uses :math:`N = 6` and :math:`d = 3`, thus each
coordinate has :math:`N + d = 9` coefficients, and each surface has :math:`9^2 = 81` coefficients.

The table that follows gives the sizes of the spline for :math:`N` intervals and the degree :math:`d`.

.. list-table:: Spline Sizes
    :widths: 46 30 24
    :header-rows: 1

    * - Quantity
      - Formula
      - Value for :math:`N = 6`, :math:`d = 3`
    * - Interval length :math:`h`
      - :math:`2 / N`
      - :math:`1/3`
    * - Knots on :math:`[-1, 1]`
      - :math:`N + 1`
      - 7
    * - Extra knots past each end
      - :math:`d`
      - 3
    * - Knots in total
      - :math:`N + 2d + 1`
      - 13
    * - Basis functions that are not zero on each interval
      - :math:`d + 1`
      - 4
    * - Coefficients for each coordinate
      - :math:`N + d`
      - 9
    * - Coefficients for each surface
      - :math:`(N + d)^2`
      - 81
    * - Coefficients for the two surfaces
      - :math:`2 (N + d)^2`
      - 162
    * - Coefficients for each surface value
      - :math:`(d + 1)^2`
      - 16

The spline does not go through its control points: at a knot, :math:`t = 0` and the weights are
:math:`(1/6, 4/6, 1/6, 0)`, thus the value is a blend of three coefficients, not one coefficient. For this reason, the
coefficients come from a fit and are not the measured motor steps. The section
:ref:`gimbalToMotorAnglesSpline_offlineAnalysis` explains the fit.

Motor Angles
~~~~~~~~~~~~

The value of each surface is the motor steps. The algorithm does not round the motor steps to whole steps. It
multiplies them by the step angle :math:`s = 2\pi / N_s`, where :math:`N_s` is the number of motor steps per revolution
(``stepsPerRevolution``):

.. math::

    s = \frac{2\pi}{N_s}, \qquad n_1 = S_1(u, v), \quad n_2 = S_2(u, v), \qquad \theta_1 = s \, n_1, \quad
    \theta_2 = s \, n_2

The algorithm calculates :math:`s` in fp32. :math:`N_s` is not more than 100000, thus its conversion to fp32 is exact.

The two surfaces use the same point :math:`(u, v)`. Each coefficient is in :math:`[-100000, 100000]`, thus each
surface value is also in this range.

.. _gimbalToMotorAnglesSpline_algorithmFlow:

Algorithm Flow
^^^^^^^^^^^^^^

At each ``update()`` call, the algorithm does these steps:

1. It changes the gimbal angles to the normalized coordinates :math:`(u, v)`.
2. If the point is not in the square :math:`|u| \le 1, |v| \le 1`, the algorithm does not accept the request. It
   gives the last motor angles that it calculated, and it does not change them. A gimbal angle that is not a finite
   number gives a :math:`u` or a :math:`v` that is not a finite number, thus the algorithm also does not accept it.
3. It calculates the motor angles from the two surfaces and the step angle :math:`s`.
4. It keeps these motor angles as the last motor angles, and it gives them.

The algorithm keeps the last motor angles in a ``std::optional``, which is empty when the algorithm is created and
after ``reInitialize()``. While it is empty, step 2 gives the motor angles of the current configuration at zero gimbal
angles, :math:`u = v = 0`. The algorithm calculates these motor angles from the two surfaces when it gets a
configuration, at creation and at each ``setConfig()``. It does not keep them as the last motor angles. ``setConfig()``
keeps the last motor angles. If the algorithm has not calculated motor angles yet, the motor angles of the new
configuration at zero gimbal angles apply at once. The adapter creates a new algorithm at each ``reset()``.

.. _gimbalToMotorAnglesSpline_assumptions:

Algorithm Assumptions and Limitations
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- The module assumes that all the requested gimbal angles are in the circle of radius :math:`r` (``thetaMax``) about
  zero gimbal angles. The algorithm does not compare each request with this circle. It accepts every request in the
  addressable rhombus, thus the rhombus must contain the circle. This is why the configuration checks that
  ``thetaMax`` is more than zero and not more than the radius of the largest circle in the rhombus: then every request
  that the module assumes is addressable.
- The fit tool makes the rhombus larger than the circle by a margin :math:`m` (see
  :ref:`gimbalToMotorAnglesSpline_rhombus`). The configuration does not check this margin, thus a configuration that
  is not from the fit tool can have a rhombus that only touches the circle.
- The algorithm rejects a request past the edge of the rhombus with no tolerance. With a rhombus from the fit tool, a
  request near the edge is about :math:`m` outside the circle, thus it is not a request that the module assumes.
- The gimbal angles in the populated cells of the lookup table that are outside the rhombus are not addressable.
- The module does not set a travel range for the motors. The steps of each motor are between the smallest and the
  largest coefficient of its surface.
- The motor steps are not whole steps. The component that moves the motors must round them, if necessary.
- The fit accuracy is applicable only to the lookup tables that the tool used. If the hardware calibration changes,
  make new coefficients.

.. _gimbalToMotorAnglesSpline_offlineAnalysis:

Offline Analysis
----------------

The offline tool ``_tools/fitGimbalToMotorSpline.py`` calculates the configuration of the algorithm from the
lookup tables. The algorithm does not do these calculations. This section gives the method of the fit, and then the
results of the fit for the lookup tables in ``_tests``. The subsection :ref:`gimbalToMotorAnglesSpline_fitMethod`
lists the steps of the tool, and the section :ref:`gimbalToMotorAnglesSpline_runOfflineFit` explains how to run it.

.. _gimbalToMotorAnglesSpline_leastSquares:

Least-Squares Fit
^^^^^^^^^^^^^^^^^

The surface value is linear in the coefficients. Let :math:`\beta_i(\xi)` be the basis function of coefficient
:math:`i` along one coordinate, with :math:`i` from 0 to :math:`N + d - 1`. On interval :math:`k`,
:math:`\beta_{k+p}(\xi) = B_p(t)` for :math:`p` from 0 to :math:`d`, and the other basis functions are zero. Thus each
surface is:

.. math::

    S(u, v) = \sum_{i=0}^{N+d-1} \sum_{j=0}^{N+d-1} \beta_i(u) \, \beta_j(v) \, c_{ij}

The tool uses the :math:`M` table cells in the addressable rhombus. For cell :math:`m`, it changes the gimbal angles
:math:`(\psi_m, \phi_m)` to :math:`(u_m, v_m)`. The table steps are :math:`n_{1,m}` and :math:`n_{2,m}`. Each cell
gives one equation for each surface:

.. math::

    S_1(u_m, v_m) = n_{1,m}, \qquad S_2(u_m, v_m) = n_{2,m}

The tool writes the equations of each surface as one linear system :math:`A \mathbf{c} = \mathbf{n}`:

- :math:`\mathbf{c}` has the :math:`(N + d)^2` coefficients of the surface. Coefficient :math:`c_{ij}` is at index
  :math:`(N + d) i + j`, the same order as ``motor1SplineCoefficients`` and ``motor2SplineCoefficients``.
- Each row of :math:`A` is one cell. The row of cell :math:`m` has :math:`\beta_i(u_m) \beta_j(v_m)` in column
  :math:`(N + d) i + j`.
- :math:`\mathbf{n}` has the table steps of one motor in the same order as the rows.

The two motors use the same cells, thus the two systems have the same matrix :math:`A`. The tool solves them together,
with one column of steps for each motor. Each row has only :math:`(d + 1)^2` values that are not zero, because
only :math:`(d + 1)^2` basis functions are not zero at a point. Thus :math:`A` has :math:`M` rows and :math:`(N + d)^2`
columns.

There are more equations than coefficients, and the table steps have errors. Thus no set of coefficients satisfies
all the equations. For each motor, the tool finds the coefficients that make the sum of the squared errors as small
as possible:

.. math::

    \min_{\mathbf{c}} \left\| A \mathbf{c} - \mathbf{n} \right\|^2
    = \min_{\mathbf{c}} \sum_{m=1}^{M} \left( S(u_m, v_m) - n_m \right)^2

The solution satisfies the normal equations :math:`A^T A \mathbf{c} = A^T \mathbf{n}`. The tool does not calculate
:math:`A^T A`. It uses ``numpy.linalg.lstsq``, which solves the problem with a singular value decomposition of
:math:`A`. This is more accurate, because the condition number of :math:`A^T A` is the square of the condition number of
:math:`A`. If the rank of :math:`A` is :math:`(N + d)^2`, the table cells determine each coefficient.

The tool calculates the fit in motor steps and writes the coefficients in motor steps. The range of the coefficients of
each motor is wider than the range of its table steps in the rhombus. The basis functions of the outer coefficients are
mostly outside the rhombus. The table cells determine only the part of these functions that is inside the rhombus, thus
these coefficients can be outside the range of the table steps. Thus the bound "between the smallest coefficient and the
largest coefficient" is wider than the range of the table steps.

.. _gimbalToMotorAnglesSpline_rhombus:

Addressable Rhombus
^^^^^^^^^^^^^^^^^^^

The addressable rhombus has its center at zero gimbal angles. It contains the required circle of gimbal angles of radius
:math:`r` about zero, and it has the aspect ratio of the populated cells of the gimbal-to-motor-steps table:
:math:`k = b / a` is the range of the tilt angles over the range of the tip angles of these cells.

The radius of the largest circle in the rhombus is :math:`a b / \sqrt{a^2 + b^2} = a k / \sqrt{1 + k^2}`. The tool
sets this radius to :math:`r' = r + m`, the required radius with the margin :math:`m` of the next paragraphs, thus:

.. math::

    a = r' \, \frac{\sqrt{1 + k^2}}{k}, \qquad b = r' \sqrt{1 + k^2}

The rhombus must be inside the populated cells of the table. It is inside if, at each tilt angle from :math:`-b` to
:math:`b`, its tip range is inside the populated cells of the two table rows above and below that tilt. The tool
samples the rhombus at 2001 tilt angles for this check, and stops with an error if the rhombus is not inside.

The module assumes that every request is in the circle of radius :math:`r` (``thetaMax``), see
:ref:`gimbalToMotorAnglesSpline_assumptions`. The rhombus contains the circle of radius :math:`r + m`, with the margin
:math:`m` (``GIMBAL_MARGIN``), the same for every radius. A request less than :math:`m` past the required
circle is still in the rhombus, thus the algorithm accepts a request that is a little larger than :math:`r`, for
example because of a rounding before the algorithm. The configuration keeps :math:`r` as ``thetaMax``, not
:math:`r + m`.

The margin also covers the fp32 rounding of :math:`u` and :math:`v`. Each point of the required circle has
:math:`|u| \le r / (r + m)` and :math:`|v| \le r / (r + m)`. The algorithm calculates :math:`u` and :math:`v` in fp32
from the fp32 values of :math:`a` and :math:`b`. The roundings of :math:`a`, of the division, and of the sum each have
a relative error of at most :math:`2^{-24}`, thus the calculated :math:`|u|` and :math:`|v|` are at most
:math:`(1 + 3 \cdot 2^{-24})` times the exact values, approximately :math:`1.8 \times 10^{-7}` more. This is much less
than :math:`m / (r + m)`, thus the rounding cannot put a point of the required circle past the edge. The tool also
calculates :math:`u` and :math:`v` in fp32 at 100001 points of the circle, and stops with an error if the largest
value is more than 1.

The margin :math:`m` makes the largest fp32 value of :math:`|u|` and :math:`|v|` on the required circle smaller than 1
by approximately :math:`m / (r + m)`.

.. _gimbalToMotorAnglesSpline_fitResults:

.. _gimbalToMotorAnglesSpline_fitMetrics:

Fit Results
^^^^^^^^^^^

The results in this section are for the lookup tables in ``_tests``.

Fit Metrics
~~~~~~~~~~~

The fit is measured with the metrics that follow. The errors are for the two motors together.

- **Equations for each coefficient** is the number of rows of :math:`A` divided by the number of coefficients of one
  surface, :math:`M / (N + d)^2`.
- **Condition number of** :math:`A`.
- **Coefficient range** is the smallest and the largest coefficient of the two surfaces. Each surface value is
  between the smallest and the largest coefficient of its surface.
- **Fitted table** errors are the errors at the gimbal-to-motor-steps table cells that the fit used.
- **Gimbal angles** errors give the fit errors as gimbal angles. For each fitted table cell, the tool calculates the
  surfaces at the gimbal angles of the cell. Then it finds the gimbal angles of these motor steps with the
  motor-steps-to-gimbal lookup table, which the fit does not use. The tool interpolates this table linearly between its
  grid points. The error is the distance between these gimbal angles and the gimbal angles of the cell,
  :math:`\sqrt{\Delta\psi^2 + \Delta\phi^2}`. Most of the error is along the tilt axis.
- **Not used cells** are the gimbal angle errors at cells that the fit did not use. The tool removes a random 10 % of
  the cells, fits the remaining cells, and calculates the gimbal angle error at the removed cells. It does this 10
  times with different random samples. The table gives the maximum and the RMS of all these errors. These errors
  show how accurate the surfaces are between the table cells. If the surfaces give motor steps outside the
  motor-steps-to-gimbal lookup table at a removed cell, the maximum and the RMS do not include this cell.
- **Table floor** is the error of the two lookup tables. The two tables do not agree exactly. The tool does the same
  calculation as for the gimbal angle errors, with the motor steps of the table in place of the surfaces. This error
  does not change with :math:`N`. The surfaces do not follow each table cell exactly, thus the gimbal angle errors of
  the fit are larger than this floor.

Selected Fit
~~~~~~~~~~~~

The module uses :math:`N = 6` intervals, the required radius :math:`r = 13.5` deg, and the margin :math:`m = 0.1` deg.
Thus the largest fp32 value of :math:`|u|` and :math:`|v|` on the required circle is smaller than 1 by approximately
:math:`m / (r + m) = 0.0074`.

With the lookup tables in ``_tests``, the rhombus has :math:`a = 16.27` deg and :math:`b = 24.78` deg, and it is
3.80 deg inside the populated cells. :ref:`The gimbal domain figure <fig_gimbalDomain>` shows the rhombus, the required
circle, and the motor 1 steps of the table.

.. _fig_gimbalDomain:

.. figure:: _Documentation/_Images/gimbalDomain.*
    :width: 80%
    :align: center

    Motor 1 steps of the populated cells of the gimbal-to-motor-steps table in ``_tests``, the addressable rhombus, and
    the required circle of radius 13.5 deg about zero gimbal angles.

The fit has these results:

- :math:`M = 2159` table cells are in the rhombus, thus there are 26.7 equations for each coefficient. :math:`A` has
  full rank, 81, and its condition number is 1.67e3.
- The coefficients are from -22701 to 22529 steps.
- At the fitted table cells, the error is 11.9 steps maximum and 1.97 steps RMS.
- The gimbal angle error is 0.0429 deg maximum, 0.0072 deg RMS, and 0.0214 deg at the 99th percentile. The maximum
  tip error is 0.0113 deg and the maximum tilt error is 0.0429 deg.
- At the not used cells, the gimbal angle error is 0.0439 deg maximum and 0.0077 deg RMS. The motor steps of all
  these cells are inside the motor-steps-to-gimbal lookup table.
- The table floor is 0.0062 deg maximum and 0.0022 deg RMS.

.. _fig_fitError:

.. figure:: _Documentation/_Images/fitError.*
    :width: 80%
    :align: center

    Gimbal angle error of the fit at the table cells in the addressable rhombus, with the required circle.

.. _fig_fitSurface3d:

.. figure:: _Documentation/_Images/fitSurface3d.*
    :width: 100%
    :align: center

    Motor 1 spline surface in the addressable rhombus, colored by the gimbal angle error, with the table cells.

.. _gimbalToMotorAnglesSpline_numIntervals:

Number of Intervals
~~~~~~~~~~~~~~~~~~~

The number of intervals :math:`N` sets the size of the surfaces. Each surface has :math:`(N + d)^2` coefficients, with
:math:`d = 3`. Each coefficient is one ``float`` of 4 bytes. The time to calculate one motor angle does not change
with :math:`N`, because the algorithm always uses :math:`(d + 1)^2 = 16` coefficients.

The table that follows compares the fits for :math:`N` = 4, 6, 8, ..., 20. All the fits use the same addressable
rhombus and the same :math:`M` table cells.

.. list-table:: Fit Errors for Different Numbers of Intervals, Lookup Tables in ``_tests``
    :widths: 8 14 22 14 14 14 14
    :header-rows: 1

    * - :math:`N`
      - Condition number of :math:`A`
      - Coefficient range [steps]
      - Fitted table max / RMS [steps]
      - Gimbal angles max / RMS [deg]
      - Not used cells max [deg]
      - Not used cells RMS [deg]
    * - 4
      - 1.59e3
      - [-23835, 24109]
      - 14.6 / 2.33
      - 0.0340 / 0.0086
      - 0.0318
      - 0.0089
    * - **6**
      - **1.67e3**
      - **[-22701, 22529]**
      - **11.9 / 1.97**
      - **0.0429 / 0.0072**
      - **0.0439**
      - **0.0077**
    * - 8
      - 1.95e3
      - [-22607, 21975]
      - 9.1 / 1.83
      - 0.0260 / 0.0066
      - 0.0277
      - 0.0071
    * - 10
      - 2.35e3
      - [-22557, 21720]
      - 9.4 / 1.74
      - 0.0311 / 0.0061
      - 0.0323
      - 0.0069
    * - 12
      - 3.16e3
      - [-21929, 21695]
      - 8.3 / 1.68
      - 0.0231 / 0.0059
      - 0.0254
      - 0.0069
    * - 14
      - 4.15e3
      - [-21825, 21207]
      - 8.7 / 1.60
      - 0.0203 / 0.0055
      - 0.0383
      - 0.0070
    * - 16
      - 5.03e3
      - [-22082, 21491]
      - 9.1 / 1.57
      - 0.0215 / 0.0054
      - 0.0845
      - 0.0081
    * - 18
      - 5.88e3
      - [-22545, 22935]
      - 8.0 / 1.52
      - 0.0165 / 0.0052
      - 0.2152
      - 0.0119
    * - 20
      - 7.96e3
      - [-25409, 29508]
      - 7.6 / 1.48
      - 0.0168 / 0.0051
      - 0.5488
      - 0.0238

When :math:`N` increases, the surfaces have more coefficients, thus the errors at the fitted cells decrease. The
gimbal angle errors use the fitted cells, thus they do not show the accuracy between the cells. The errors at the not
used cells show this accuracy, and after some value of :math:`N` more intervals do not make them smaller. For a large
:math:`N`, each interval contains only a small number of table cells, and some coefficients near the vertices of the
rhombus have almost no data. Then the condition number of :math:`A` increases, and the errors at the not used cells
increase quickly. The comparison stops at :math:`N = 20`, because larger values give coefficients that increase quickly:
with the lookup tables in ``_tests``, the coefficients go past the bound of 100000 steps at :math:`N = 24`.

This module uses :math:`N = 6`, thus the two surfaces have 162 coefficients (648 bytes). Its RMS errors are near the
RMS errors of larger values of :math:`N`: 0.0072 deg at the fitted cells and 0.0077 deg at the not used cells, and
0.0059 deg and 0.0069 deg for :math:`N = 12`. Its maximum errors are larger: 0.0429 deg at the fitted cells and
0.0439 deg at the not used cells, and 0.0231 deg and 0.0254 deg for :math:`N = 12`. Of the values in the comparison,
:math:`N = 12` gives the smallest errors at the not used cells, with 225 coefficients for each surface.

.. _gimbalToMotorAnglesSpline_memoryComparison:

Memory Comparison
~~~~~~~~~~~~~~~~~

All values are ``float`` of 4 bytes. The spline keeps :math:`2 \times 81 = 162` coefficients (648 bytes). This does
not depend on the size of the lookup table.

A gimbal-to-motor-steps lookup table of :math:`R` rows and :math:`C` columns keeps the full grid of each motor, thus
the algorithm can find a cell from its index. The gimbal angles of the grid change along one axis only, thus the
table keeps one vector of :math:`R` values and one vector of :math:`C` values for the gimbal angles. Thus the table
keeps :math:`2 R C + R + C` values. A table that keeps only the populated cells, or only the cells in the addressable
rhombus, is smaller, but it needs more memory to find a cell. The spline uses less memory than the full grid when
:math:`2 R C + R + C > 162`, for example for a grid of more than 8 by 8 cells.

The table that follows gives the memory of the full grid of the gimbal-to-motor-steps lookup table in ``_tests``
and of the spline.

.. list-table:: Memory of the Gimbal-to-Motor-Steps Lookup Table in ``_tests`` and of the Spline
    :widths: 40 20 20 20
    :header-rows: 1

    * - Data
      - Grid [rows x columns]
      - Values
      - Memory [bytes]
    * - Gimbal-to-motor-steps table
      - 108 x 74
      - 16166
      - 64664
    * - Spline coefficients
      -
      - 162
      - 648

The spline uses about 1/100 of the memory of the table.

Test Description
----------------

The tests are in ``_tests``. The C++ unit tests (``test_gimbalToMotorAnglesSpline.cpp``) and the fuzz tests
(``test_gimbalToMotorAnglesSpline_fuzz.cpp``) test the algorithm. The Python tests
(``test_gimbalToMotorAnglesSpline.py``) test the fit: they operate the adapter in a simulation with the committed
configuration and compare it with the lookup tables and scipy. The fuzz tests are built only when
``XMERA_ENABLE_FUZZTESTS`` is on.

The algorithm declares its spline functions in the namespace ``gimbal_to_motor_angles_spline_detail``:
``cubicBSplineWeights``, ``splineInterval``, and ``evaluateSurface``. The tests of the spline itself call these
functions directly, at exact values of :math:`u` and :math:`v` anywhere in :math:`[-1, 1] \times [-1, 1]`, edges and
vertices included, without the change from gimbal angles and without the step angle. The other tests send requests to
``update()``, thus they also check the change from gimbal angles to :math:`u` and :math:`v`, the accepted and the
rejected requests, the step angle, and the configuration.

.. _gimbalToMotorAnglesSpline_testData:

Test Data and Tolerances
^^^^^^^^^^^^^^^^^^^^^^^^

The C++ tests do not read files. They use two made-up test configurations, the linear and the rough test
configuration, as the ``oeStateEphem`` C++ tests use made-up Chebyshev coefficients. Only the Python tests use the
fitted configuration: ``splineDomain.csv``, ``splineCoefficientsMotor1.csv``, and ``splineCoefficientsMotor2.csv``,
which ``--emit`` writes (see :ref:`gimbalToMotorAnglesSpline_runOfflineFit`) from the two lookup tables in ``_tests``.
These tables are changed from the calibration of the hardware, thus the tests, the fit results, and the figures in this
document do not give the motor steps of the hardware. If the tables in ``_tests`` change, write the configuration and
the figures again with the tool; ``test_configuration_matches_fit_tool`` fails until then. All the tests use
:math:`N_s = 628` test steps per revolution, a step angle :math:`s` of about 0.01 rad (``kTestStepsPerRevolution``).

The tests compare the motor angles in motor steps. The function ``motorSteps`` calculates :math:`s = 2\pi / N_s` in fp32
as the algorithm does, and divides each motor angle by :math:`s`. The multiplication by :math:`s` in the algorithm and
this division each add a relative fp32 rounding error of not more than :math:`\epsilon/2`, thus the steps differ from
the surface value by not more than :math:`\epsilon` times the value (``kAngleConversionRelativeError``).

The linear test configuration (``makeLinearConfig``) has a rhombus with :math:`a = 0.3` rad and
:math:`b = 0.45` rad, and :math:`r = 0.24` rad, about 0.01 rad inside the radius of the largest circle in the rhombus,
0.2496 rad. Its coefficients are the Greville abscissae multiplied by 1000:
:math:`c^{(1)}_{ij} = 1000 \, (-1 + (j - 1) h)` for motor 1 and :math:`c^{(2)}_{ij} = 1000 \, (-1 + (i - 1) h)` for
motor 2. With these coefficients, the surfaces are exactly :math:`S_1(u, v) = 1000 \, v` and
:math:`S_2(u, v) = 1000 \, u`, thus :math:`n_1 = 1000 \, v` and :math:`n_2 = 1000 \, u` in the rhombus. The two motors
have different surfaces, thus the tests find a swap of the two coefficient sets. The motor steps are in
:math:`[-1000, 1000]`.

A linear surface is the same polynomial on every interval, thus it cannot show an error where one interval meets the
next, or an error in the weights that a linear surface does not see. The rough test configuration
(``makeRoughConfig``) has coefficients that change irregularly from one coefficient to the next, thus its surfaces are
a different cubic on every interval. Each coefficient is :math:`50000 \, w` steps, with :math:`w` in :math:`[-1, 1)`
from an integer hash of the coefficient index and a seed, one seed for each motor (``roughSurfaceCoefficients``). The
values are multiples of :math:`2^{-23}` before the scaling, thus they are the same on every platform. Its rhombus has
:math:`a = 0.24` rad and :math:`b = 0.2` rad, wider along tip than the linear test rhombus, and :math:`r = 0.15` rad,
about 0.0036 rad inside the radius of the largest circle in the rhombus, 0.1536 rad.

The tests that need the algorithm to accept a request send requests in the circle of radius :math:`r`, the requests
that the module assumes. In the two test rhombi, :math:`r` is well inside the largest circle in the rhombus, much more
than the fp32 rounding of :math:`u` and :math:`v`, thus the algorithm always accepts these requests. The function
``gimbalAnglesInCircle`` gives the gimbal angles at a fraction of :math:`r` from zero, from 0 to 1, and in a direction
from 0 to :math:`2\pi`. Only ``RhombusEdgeIsTheAcceptanceBoundary`` sends accepted requests outside the circle, at
:math:`10^{-3}` inside the rhombus edges. The tests calculate the expected values from the float64 :math:`u` and
:math:`v` of the gimbal angles (``referenceUv``).

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
  continuity test and the Lipschitz fuzz test use this bound for the surface values, which they calculate directly
  with ``evaluateSurface``. Their tolerance is

  .. math::

      L \, (|u_2 - u_1| + |v_2 - v_1|) + 2 \, T_v

  :math:`T_v` is the surface value tolerance (see the next subsection), one for each of the two points.
- A rounding error in :math:`u` or :math:`v` changes the surface value by not more than :math:`L` multiplied by the
  rounding error. The fp32 calculation tolerance uses this bound.

The bound is conservative: for the fitted surfaces of the tool, the largest slopes on a dense grid of points are much
less than :math:`L`. The largest differences are between the outer coefficients, but the basis functions of these
coefficients are mostly outside the square. Also, the one bound :math:`L` is used for the two coordinates, but each
surface changes much less along one of them.

The slopes with respect to the gimbal angles come from the chain rule. For each motor:

.. math::

    \frac{\partial n}{\partial \psi} = \frac{1}{a} \left( \frac{\partial S}{\partial u}
    + \frac{\partial S}{\partial v} \right), \qquad
    \frac{\partial n}{\partial \phi} = \frac{1}{b} \left( \frac{\partial S}{\partial u}
    - \frac{\partial S}{\partial v} \right)

Tolerances
~~~~~~~~~~

The tests use these tolerances:

- **Surface value tolerance.** The function ``surfaceValueTolerance`` calculates the tolerance of a surface value
  that ``evaluateSurface`` calculates at exact :math:`u` and :math:`v`:

  .. math::

      T_v = 3 \cdot 20.5 \, \epsilon \max |c_{ij}|

  This is the rounding error of the coefficients, of the weights, and of the weighted sum. The factor 3 is a margin.
- **fp32 calculation tolerance.** The function ``evaluationTolerance`` calculates this tolerance for one surface from
  its coefficients, for a request to ``update()``:

  .. math::

      T = T_v + 3 \cdot 2 \cdot 9 \, \epsilon \, L

  The second term is the rounding error of :math:`u` and of :math:`v` that the algorithm calculates from the gimbal
  angles, approximately :math:`9 \epsilon` each, multiplied by the slope bound. The factor 3 is a margin.
  :math:`\epsilon` is the fp32 machine epsilon. The tests calculate :math:`T` from the coefficients, thus :math:`T`
  changes when the coefficients change.
- **Step tolerance.** The function ``stepTolerance`` gives :math:`T_s = T + \epsilon \max |c_{ij}|`: the fp32
  calculation tolerance plus the error of the change from motor steps to a motor angle and back. The tests use this
  tolerance to compare the motor steps with a float64 value of the same surface.
- **Linear surface tolerance.** :math:`3 \, (1 + h) \cdot 1000 \, (2 \cdot 20.5 \, \epsilon + \epsilon)`,
  approximately 0.020 steps, for the motor steps of a request to ``update()`` with the linear test configuration. The
  coefficients of the linear surfaces are at most :math:`1000 \, (1 + h)` steps, and their slope is 1000 steps for
  each unit, thus the rounding of :math:`u` and :math:`v` adds about as much as the value error. The factor 3 is a
  margin, as for :math:`T`.
- **Linear surface evaluation tolerance.** :math:`3 \, (1 + h) \cdot 1000 \cdot 20.5 \, \epsilon`, approximately
  0.0098 steps, for the linear surfaces that ``evaluateSurface`` calculates at exact :math:`u` and :math:`v`: the value
  error only.

The tests of the fitted configuration, against scipy at the table cells, against the fit tool, and for the
least-squares solution, are the Python fit tests (see :ref:`gimbalToMotorAnglesSpline_pythonFitTests`).

Acceptance and Hold Tests
^^^^^^^^^^^^^^^^^^^^^^^^^

These tests check which requests the algorithm accepts, and what it gives for the requests that it does not accept.
They use the linear test configuration, except where the description names the rough test configuration. The tests
of a rejected request first send a point in the rhombus and check its motor steps, thus the test knows that the
algorithm calculated new motor angles. Then the test sends a request that the algorithm does not accept and checks
that the motor angles are exactly equal to the last motor angles.

- ``FirstCallOutsideDomainReturnsZeroGimbalMotorAngles``
    - With the rough test configuration, sends a point outside the rhombus first. Checks that the motor angles are
      exactly the motor angles that a second algorithm gives for zero gimbal angles, and that they are the float64
      surfaces at :math:`u = v = 0`, within the step tolerance.

- ``OutsideDomainHoldsPreviousValidOutput``
    - Sends points outside the rhombus at :math:`(u, v) = (1.5, 0)`, :math:`(0, -1.5)`, and :math:`(3, 3)`.

- ``NonFiniteInputHoldsPreviousValidOutput``
    - Sends gimbal angles that are NaN, :math:`+\infty`, or :math:`-\infty`, in one angle or in the two angles.

- ``OverflowingFiniteInputHoldsPreviousValidOutput``
    - Sends the largest fp32 values with opposite signs. These values are finite, but :math:`u` and :math:`v`
      become :math:`+\infty` and :math:`-\infty`, and their sum is NaN.

- ``CircleIsAddressable``
    - For the rough and for the linear test configuration, sends requests at 720 directions on the circle of radius
      :math:`r`, after ``reInitialize()``. Checks that each request gives the motor steps of the float64 reference
      surfaces (``referenceSurface``) at its :math:`u` and :math:`v`, within the step tolerance. Thus every request
      that the module assumes is addressable. With the rough coefficients, this test also checks the algorithm against
      the reference on every interval that the circle crosses.

- ``RhombusEdgeIsTheAcceptanceBoundary``
    - For the rough and for the linear test configuration, at each of the four edges, sends a request with
      :math:`|u|` or :math:`|v|` equal to :math:`1 - 10^{-3}`, then the request with the same coordinate equal to
      :math:`1 + 10^{-3}`. Checks that the first request is outside the circle of radius :math:`r`, and that it is
      accepted and gives the motor steps of the float64 reference surfaces within the step tolerance. Checks that the
      second request is not accepted and keeps the motor angles of the first. Thus the rhombus edge, not the circle,
      decides which requests are accepted. The offsets of :math:`10^{-3}` are much larger than the fp32 rounding of
      :math:`u` and :math:`v`, thus the test does not depend on the rounding of points on the edge.

- ``ReInitializeRestoresInitialOutput``
    - Sends a point outside the rhombus to a new algorithm to get its initial motor angles. Then sends a point in
      the rhombus to a second algorithm, calls ``reInitialize()``, and sends the point outside the rhombus. Checks
      that the motor angles are equal to the initial motor angles of the new algorithm.

- ``FallbackUsesZeroGimbalMotorAnglesOfCurrentConfig``
    - Sends a point outside the rhombus to the linear test configuration and checks that it gives zero steps for the
      two motors. Then sets a configuration whose surfaces are the linear surfaces plus 250 steps for motor 1 and
      -400 steps for motor 2. Checks that a point outside the rhombus gives 250 and -400 steps at once, thus the motor
      angles at zero gimbal angles of the first configuration were not kept as the last motor angles. Then sends a
      point in the rhombus and checks that a point outside the rhombus keeps those motor angles, not the motor angles
      at zero gimbal angles. Then calls ``reInitialize()`` and checks that a point outside the rhombus gives the motor
      angles at zero gimbal angles of the new configuration again.

Configuration Tests
^^^^^^^^^^^^^^^^^^^

- ``SetConfigReplacesSurfaceAndKeepsPreviousOutput``
    - Calculates motor angles with the rough test configuration. Then sets a configuration with all the motor 1
      coefficients equal to 1250 steps and all the motor 2 coefficients equal to -2500 steps. Checks that a point
      outside the rhombus keeps the last motor angles of the rough surfaces, and that a point in the rhombus gives
      exactly 1250 steps for motor 1 and -2500 steps for motor 2.

- ``ConfigRoundTrip``
    - Makes a configuration from the test steps per revolution, the rough test rhombus, and the rough coefficients of
      the two motors. Checks that the configuration gives the same values back.

- ``ConfigRejectsInvalidStepsPerRevolution``
    - Checks that the configuration accepts 1, 628, and 100000 steps per revolution. Checks that it does not accept
      zero or 100001 steps per revolution.

- ``ConfigRejectsInvalidDomain``
    - Checks that the linear test rhombus is accepted. Checks that rhombi with :math:`r` exactly equal to the fp32
      radius of their largest circle are accepted, with half-diagonals of 0.3 rad and 0.45 rad, and with one or two
      half-diagonals of :math:`\pi/2`. Checks that :math:`r` equal to the smallest positive fp32 value is accepted.
      Checks that the configuration does not accept a value that is NaN or infinite, a value that is zero or less
      than zero, a half-diagonal one fp32 step past :math:`\pi/2`, or an :math:`r` one fp32 step past the radius of
      the largest circle.

- ``ConfigRejectsInvalidCoefficients``
    - For each motor, with the coefficients of the other motor valid, sets the first, the center, or the last
      coefficient to a test value. Checks that the configuration accepts :math:`\pm 100000` steps. Checks that it does
      not accept a value one fp32 step past :math:`\pm 100000` steps, NaN, or :math:`\pm\infty`.

Property Tests
^^^^^^^^^^^^^^

- ``CubicWeightsArePartitionOfUnity``
    - Calls ``cubicBSplineWeights`` at 65 values of :math:`t` from 0 to 1. Checks that the four weights are not
      negative and that their sum is one within :math:`8 \epsilon`. Checks that the weights are
      :math:`(1/6, 4/6, 1/6, 0)` at :math:`t = 0` and :math:`(0, 1/6, 4/6, 1/6)` at :math:`t = 1`, thus the weights at
      the end of one interval are the weights at the start of the next one, moved by one coefficient.

- ``SplineIntervalFindsLocalCoordinate``
    - Calls ``splineInterval`` at 601 values of :math:`x` from -1 to 1. Checks that the interval is less than
      :math:`N`, that :math:`t` is in :math:`[0, 1]`, and that :math:`-1 + h \, (k + t)` is :math:`x` within
      :math:`8 \epsilon`. Checks that :math:`x = -1` is the start of the first interval and that :math:`x = 1` is the
      end of the last interval.

- ``PropertyReproducesLinearSurface``
    - Calls ``propertyReproducesLinearSurface`` on a grid of :math:`u` and :math:`v` from -1 to 1 in steps of
      0.0625, edges included. The property evaluates ``evaluateSurface`` with the coefficients of the linear surfaces.
      Checks that the surfaces are :math:`1000 \, u` and :math:`1000 \, v` within the linear surface evaluation
      tolerance. This test checks the basis weights, the interval search, and the order of the coefficients.
      The tests with the linear configuration, such as ``CircleIsAddressable``, check which coefficient set
      each motor uses and that :math:`u` and :math:`v` are not swapped.

- ``PropertySurfaceWithinCoefficientBounds``
    - Calls ``propertySurfaceWithinCoefficientBounds`` with the rough coefficients of each motor on a grid of 81 by 81
      points of :math:`u` and :math:`v` from -1 to 1. Checks that each surface value is between the smallest and the
      largest coefficient of its surface, with the surface value tolerance.

- ``PropertyOutputIsFiniteForAnyInput``
    - Calls ``propertyOutputIsFiniteForAnyInput`` for each pair of NaN, :math:`\pm\infty`, the largest fp32
      value with each sign, the smallest subnormal value, zero, and two angles in the circle.

- ``PropertyOutsideDomainHoldsPreviousOutput``
    - Calls ``propertyOutsideDomainHoldsPreviousOutput`` with two points in the circle, one of them on it, and four
      points outside the rhombus, from 1 % past the edge to :math:`|u| = |v| = 50`.

- ``PropertySurfaceIsContinuousAcrossIntervalEdges``
    - At each knot inside :math:`[-1, 1]`, calls ``propertySurfaceIsLipschitz`` with the points :math:`10^{-4}` before
      and after the knot, along :math:`u` and along :math:`v`, at six values of the other coordinate from -1 to 1.
      Checks that the change of each rough test surface is not more than the slope bound of the surface multiplied by
      the distance, plus the surface value tolerance of the two points.

Fuzz Tests
^^^^^^^^^^

Each fuzz test calls the same property function as a unit test, from ``gimbalToMotorAnglesSplineTestHelpers.hpp``,
with inputs that the fuzzer generates.

- ``propertyOutputIsFiniteForAnyInput``
    - Sends any two fp32 values, including NaN, infinite values, and very large values. Checks that the motor
      angles are finite. Checks that they are equal to the initial motor angles of a new algorithm, or that their
      motor steps are between the smallest and the largest coefficient of each motor, with the surface value
      tolerance and the angle conversion error.

- ``propertyOutsideDomainHoldsPreviousOutput``
    - Sends a random point in the circle and then a random point outside the rhombus, with :math:`|u|` or
      :math:`|v|` up to 50. Checks that the algorithm keeps the motor angles of the first point.

- ``propertyReproducesLinearSurface``
    - Evaluates the linear surfaces at a random point of :math:`[-1, 1] \times [-1, 1]`, edges included. Checks that
      the surfaces are :math:`1000 \, u` and :math:`1000 \, v` within the linear surface evaluation tolerance.

- ``propertySurfaceIsLipschitz``
    - Evaluates the rough test surfaces at two random points of :math:`[-1, 1] \times [-1, 1]` that are not more than
      0.01 apart in :math:`u` and in :math:`v`. Checks that the change of each surface is not more than the slope bound
      of the surface multiplied by the distance, plus the surface value tolerance of the two points.

- ``propertySurfaceWithinCoefficientBounds``
    - Evaluates a surface with random coefficients from -100000 steps to 100000 steps, the full range that the
      configuration accepts, at a random point of :math:`[-1, 1] \times [-1, 1]`. Checks that the value is between the
      smallest and the largest coefficient, with the surface value tolerance.

.. _gimbalToMotorAnglesSpline_pythonFitTests:

Python Fit Tests
^^^^^^^^^^^^^^^^

The GTest suite tests the algorithm. The Python tests check the committed configuration (``splineDomain.csv``,
``splineCoefficientsMotor1.csv``, and ``splineCoefficientsMotor2.csv``) against the gimbal-to-motor-steps lookup table
in ``_tests``, as the ``oeStateEphem`` test checks its Chebyshev fit against SPICE. The tests calculate the reference
values from the table and with scipy when they run, thus they do not need reference data files. They import the
functions of the fit tool from ``_tools/fitGimbalToMotorSpline.py``.

``test_matches_scipy_evaluation`` and ``test_round_trip_through_steps_to_gimbal_table`` use the outputs of the adapter
for every table cell in the circle of radius :math:`r`, which the tests make once. The adapter properties are set from
the committed configuration, with 628 steps per revolution. Each request is written to ``twoAxisGimbalInMsg`` with a new
write time and the simulation runs one task step, and the tests change the motor angles of ``motor1AngleOutMsg`` and
``motor2AngleOutMsg`` to motor steps with the fp32 step angle. The other tests check the configuration files directly.
The step tolerance is calculated as ``stepTolerance`` in the GTest suite.

- ``test_matches_scipy_evaluation``
    - Sends every table cell in the circle of radius :math:`r`. Checks that the motor steps are the scipy (float64)
      values of the committed surfaces at the same fp32 gimbal angles, within the step tolerance. This test checks that
      the algorithm and scipy use the same basis functions and the same order of the coefficients. The rhombus is
      rotated from the table grid, thus the cells cover every interval along :math:`u` and :math:`v` at many positions
      in it, and the outer cells are close to the circle. The test compares with scipy and not with the measured
      steps: a tolerance from the fit error would pass for any fit, because the fit error sets it.

- ``test_round_trip_through_steps_to_gimbal_table``
    - Maps the motor steps of the adapter back to gimbal angles with the motor-steps-to-gimbal lookup table
      (``stepsToGimbalTip.csv`` and ``stepsToGimbalTilt.csv``, bilinear interpolation, ``GimbalTable`` of the fit
      tool). Checks
      that the motor steps are inside this table, and that the distance between the requested and the mapped gimbal
      angles, :math:`\sqrt{\Delta\psi^2 + \Delta\phi^2}`, is not more than ``MAX_ROUND_TRIP_ERROR`` = 0.05 deg at
      every table cell in the circle of radius :math:`r`. The fit does not use this table, thus this test is an
      independent check of the accuracy of the module, as the gimbal error metrics of the fit tool (see
      :ref:`gimbalToMotorAnglesSpline_fitMetrics`). The committed configuration gives about 0.043 deg at most. The two
      tables alone give about 0.006 deg at most: the table steps mapped back with the other table.

- ``test_configuration_matches_fit_tool``
    - Makes the rhombus and the fit again with the fit tool, from the table and the committed :math:`r` and number of
      intervals. Checks that the committed half-diagonals and coefficients are the same, to the ten significant
      digits of the files. Thus the test fails if the table or the tool changes and the configuration is not written
      again. The tool makes the rhombus with the margin :math:`m` past the circle, and before it writes the
      configuration it checks with ``fp32_circle_max_uv`` that fp32 rounding keeps the circle in the rhombus and with
      ``inside_margin`` that the rhombus is inside the populated table cells. Thus the committed rhombus also has
      these properties.

- ``test_coefficients_are_least_squares``
    - Calculates the gradient :math:`A^T (A c - n)` of half the sum of the squared errors at the table cells in the
      rhombus, for each motor. Checks that each component is zero to within the rounding of the coefficients in the
      files: they keep ten significant digits, thus the bound is :math:`A^T A \, (10^{-9} |c|)`, twice the rounding.

.. _gimbalToMotorAnglesSpline_runOfflineFit:

Python Offline Fit Tool
-----------------------

The tool ``_tools/fitGimbalToMotorSpline.py`` is a Python script. It needs ``numpy`` and ``scipy``.
The figures also need ``matplotlib``. The default directory of the lookup tables is relative to the tool, thus the tool
can start from any directory. The paths of ``--emit`` and ``--figures`` are relative to the current directory.

Each time the tool starts, it reads the two lookup tables, finds the addressable rhombus about the circle of
``--radius``, and prints its half-diagonals, its distance to the edge of the populated cells, and the largest fp32 value
of :math:`|u|` and :math:`|v|` on the circle. Then it does the actions that the options select. If no action is
selected, the tool stops after this.

.. _gimbalToMotorAnglesSpline_fitMethod:

Fit Method
^^^^^^^^^^

The tool ``_tools/fitGimbalToMotorSpline.py`` does these steps:

1. It reads the gimbal-to-motor-steps lookup table (``_tests/gimbalToStepsMotor1.csv`` and
   ``_tests/gimbalToStepsMotor2.csv``). The table gives the motor 1 and motor 2 steps on a uniform grid of gimbal tip angles :math:`\alpha = \psi` (columns) and
   gimbal tilt angles :math:`\beta = \phi` (rows). The populated cells form a rhombus.
2. It finds the addressable rhombus: the rhombus about zero gimbal angles, with the aspect ratio of the populated
   cells of the table, whose edges touch the required circle of gimbal angles plus a small margin. The subsection
   :ref:`gimbalToMotorAnglesSpline_rhombus` explains this step.
3. It calculates the coefficients of the two surfaces with linear least squares. The subsection
   :ref:`gimbalToMotorAnglesSpline_leastSquares` explains this step.
4. It calculates the errors of the fit as gimbal angles with the motor-steps-to-gimbal lookup table
   (``_tests/stepsToGimbalTip.csv`` and ``_tests/stepsToGimbalTilt.csv``). The fit does not use this table. The subsection
   :ref:`gimbalToMotorAnglesSpline_fitMetrics` explains the errors.
5. It writes the configuration to ``DIR`` (``--emit DIR``). The subsection
   :ref:`gimbalToMotorAnglesSpline_makeNewCoefficients` explains how to run the tool, with ``DIR`` set to ``_tests``.

Command-Line Options
^^^^^^^^^^^^^^^^^^^^

.. list-table:: Lookup Table Options
    :widths: 26 30 44
    :header-rows: 1

    * - Option
      - Default
      - Description
    * - ``--table-dir DIR``
      - ``_tests``
      - Directory of the lookup tables. Each table is a CSV file: the first row gives the column axis, the first
        column gives the row axis, and the rest gives the table values. The gimbal-to-motor-steps lookup table that
        the tool fits is ``gimbalToStepsMotor1.csv`` and ``gimbalToStepsMotor2.csv``: columns along the gimbal tip
        angle [deg], rows along the gimbal tilt angle [deg], and the motor steps. A ``nan`` value is not measured. The
        two files must have the same axes and the same ``nan`` values. The motor-steps-to-gimbal lookup table that the
        tool uses to find the gimbal angle errors is ``stepsToGimbalTip.csv`` and ``stepsToGimbalTilt.csv``: columns
        along the motor 1 steps, rows along the motor 2 steps, and the gimbal tip and tilt angles [deg].

.. list-table:: Fit Options
    :widths: 26 22 52
    :header-rows: 1

    * - Option
      - Default
      - Description
    * - ``--radius DEG``
      - 13.5
      - Radius [deg] of the required circle of gimbal angles about zero that the addressable rhombus must contain
    * - ``--intervals N``
      - 6
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
      - Write the configuration to ``DIR``
    * - ``--figures DIR``
      - None
      - Write the documentation figures to ``DIR``
    * - ``--show``
      - Off
      - Open the figures in interactive windows. This option can be used with ``--figures`` or without it.

``--sweep`` and ``--report`` print one row of a markdown table for each fit. The columns are :math:`N`, the number of
coefficients of the two surfaces and their memory, the number of equations for each coefficient, the rank and the
condition number of :math:`A`, the coefficient range, the maximum tip and tilt errors and the maximum, RMS, and 99th
percentile of the gimbal angle error [deg], the maximum and RMS gimbal angle error of the lookup tables [deg], the
maximum and RMS of the error at the fitted table [steps], and the maximum and RMS of the gimbal angle error at the not
used cells [deg] with the number of these cells whose motor steps are outside the motor-steps-to-gimbal lookup table.
The subsection :ref:`gimbalToMotorAnglesSpline_fitMetrics` explains these metrics.

.. _gimbalToMotorAnglesSpline_outputFiles:

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
      - One row with :math:`a`, :math:`b`, and the required radius :math:`r` [rad]. These are the values of
        ``halfDiagonalTip``, ``halfDiagonalTilt``, and ``thetaMax``.

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
      - Table steps of motor 1, the addressable rhombus, and the required circle of radius ``--radius`` about zero
        gimbal angles
    * - ``fitError.pdf`` and ``fitError.svg``
      - Gimbal angle error at the table cells in the addressable rhombus [deg], with the required circle
    * - ``fitSurface3d.pdf`` and ``fitSurface3d.svg``
      - Motor 1 spline surface in the addressable rhombus, colored by the gimbal angle error, with the table cells

``rhombusMapping`` and ``basisFunctions`` do not use the lookup table data. The other figures come from the lookup
tables in ``_tests``. This document shows all the figures.

.. _gimbalToMotorAnglesSpline_makeNewCoefficients:

Make New Coefficients
^^^^^^^^^^^^^^^^^^^^^

To make new coefficients, do the steps that follow:

1. Go to the ``_tools`` directory. Examine the fit metrics for the numbers of intervals:

   .. code-block:: bash

       python fitGimbalToMotorSpline.py --sweep

2. Write the configuration and the figures for the number of intervals that you select:

   .. code-block:: bash

       python fitGimbalToMotorSpline.py --intervals 6 --emit ../_tests --figures ../_Documentation/_Images

   Add ``--show`` to also open the figures in interactive windows.

3. Set the configuration from ``_tests/splineDomain.csv``, ``_tests/splineCoefficientsMotor1.csv``, and
   ``_tests/splineCoefficientsMotor2.csv``. Set ``stepsPerRevolution`` from the stepper motor.
4. If you change the number of intervals, change ``NUM_SPLINE_INTERVALS`` in ``gimbalToMotorAnglesSplineTypes.h``.
   ``NUM_SPLINE_COEFFS_PER_AXIS`` and ``NUM_SPLINE_COEFFS`` derive from it.
5. The lookup tables in ``_tests`` are not the calibration of the hardware (see
   :ref:`gimbalToMotorAnglesSpline_testData`). To make the coefficients of the hardware, write its lookup tables as
   the CSV files of ``--table-dir`` in a directory outside the repository, give this directory with ``--table-dir``,
   and write the files and the figures to directories outside the
   repository. Do not commit the hardware lookup tables, or the files, figures, and numbers that the tool makes from
   them.
