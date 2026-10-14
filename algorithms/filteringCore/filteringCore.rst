.. _filtering-architecture:

Filtering Architecture (filteringCore)
=======================================

.. note::

   This page describes the filter core infrastructure
   under ``algorithms/filteringCore``. It explains how the pieces relate, how to use
   a filter, and how to add a new one. Although it currently only contains a SRuKF, it is built to be able to extend to
   EKFs, uKFs, SRIFs, Batch, Sequential batch, and more.

Background
---------------

This filtering core exists to create and interface in order to compose filters with ease
and be able to rely on the previously tested math and business logic required by filter families.

For example, the kalmanFilter header file contains the applySequential and applySequentialRobust functions which orchestrate the
calls of time and measurement updates on a queue of measurements. These functions can call the srukf (other header)
definitions of those functions to perform those specific updates.

With that, the implementation of a filter which has specific dynamics, state vector, and measurements
can be done at the module/algorithm level all the while relying on these core capabilities.

Composable structure
--------------------

::

   +-------------------------------------------------------------+
   | xmera host adapter   (algorithms/sunlineFilter/              |
   |                       sunlineFilter.{h,cpp})                  |
   |   - SunlineFilter : SysModel, ReadFunctor<>, Message<>       |
   |   - marshals MsgPayloads <-> plain-data I/O types           |
   +----------------------------+--------------------------------+
                                | owns (pimpl)
                                v
   +-------------------------------------------------------------+
   | algorithm library   (algorithms/sunlineFilter/              |
   |                       sunlineFilterAlgorithm.{h,cpp})         |
   |   - SunlineFilterAlgorithm  (stateful, plain C++ class)       |
   |   - sunlineFilterSpecs.h    (State, dynamics, I/O types,     |
   |                             variant Measurement)            |
   |   - measurement model(s) (in the .cpp)                       |
   +----------------------------+--------------------------------+
                                | built on
                                v
   +-------------------------------------------------------------+
   | filteringCore   (header-only INTERFACE library)            |
   |   - StateVector<Components...> + component tags             |
   |   - concepts: FilterState, Measurement, Dynamics, ...       |
   |   - DynamicsModel / rk4 / propagate                         |
   |   - measurement_queue                                       |
   |   - SRUKF functional core + SRuKF<Spec> facade     |
   +-------------------------------------------------------------+
                  depends only on Eigen + the C++ stdlib

The host adapter and the algorithm library live in the same directory
(``algorithms/sunlineFilter``) and compile into one SWIG module, but they stay
seperate: ``sunlineFilter.{h,cpp}`` is the xmera translation
unit, and ``sunlineFilterAlgorithm.{h,cpp}`` / ``sunlineFilterSpecs.h`` depend
only on ``filteringCore``.

filteringCore
~~~~~~~~~~~~~~~

Header-only interface library (``algorithms/filteringCore``). The headers
sit at the top of that directory and are included as ``<filteringCore/...>``,
which the parent ``algorithms/`` include path resolves.

All filter math (states, covariances, sigma points, measurement models,
dynamics, and the propagation step ``dt``) is single precision ``float``.
Absolute times (measurement time tags, ``callTime``, the queue's
time-of-last-measurement) stay ``double`` so they do not lose resolution over
long runs; only their differences are narrowed to ``float`` before
``timeUpdate(dt)``.

``state.hpp``
   ``StateVector<Components...>`` is a type-composed state vector laid
   out contiguously in a single fixed-size ``Eigen::Vector``. Components are
   addressed by their tag type at compile time:

   .. code-block:: cpp

      using SunlineState = filtering::StateVector<filtering::Position<3>,
                                                  filtering::Velocity<3>,
                                                  filtering::Bias<1>>;  // size 7
      SunlineState s;
      s.set<filtering::Position<3>>(Eigen::Vector3f{0, 0, 1});  // sun heading
      Eigen::Vector3f omega = s.get<filtering::Velocity<3>>();  // body rate

   Component tags (``Position``, ``Velocity``, ``Acceleration``, ``Bias``,
   ``MrpAttitude``, ``AngularRate``, ``Consider``) each carry a compile-time
   ``size``. ``StateVector`` satisfies ``LinearlyCombinable`` via ``add()`` and
   ``scale()`` and exposes ``raw()`` for the underlying Eigen vector.

``concepts.hpp``
   The C++20 concepts that define the interfaces, so filters compose by
   satisfying an interface rather than by inheritance:

   - ``LinearlyCombinable<T>`` — has ``scale(k)``, ``add(other)`` and ``subtract(other)``.
   - ``FilterState<S>`` — ``LinearlyCombinable`` plus ``S::size`` and ``raw()``.
   - ``Measurement<M, State>`` — ``M::size``, ``observation()``,
     ``model(state)``, ``noise()``, ``subtract(a, b)``.
   - ``Dynamics<D, State>`` — invocable ``(float, State) -> State``.

``dynamicsModel.hpp``
   ``DynamicsModel<State>`` (a ``std::function`` alias) plus ``rk4Increment()``,
   ``rk4()`` and ``propagate()`` — RK4 integration usable on any
   ``LinearlyCombinable`` state. ``propagate()`` accumulates the sub-steps
   with Kahan compensation (see "Accumulating the RK4 sub-steps").
   ``propagateDeviation()`` propagates a small deviation from a reference
   state (see "Propagating the sigma points as deviations").

``measurementQueue.h``
   ``measurement_queue<Measurement, CAPACITY>`` — a bounded, time-ordered
   container (``enqueue`` / ``popEarliest`` / ``clear`` / ``isFull`` /
   ``isEmpty``) plus a persistent ``getTimeOfLastMeasurement()`` /
   ``setTimeOfLastMeasurement()`` pair: the time of the last measurement
   the scheduler folded into the filter. Scheduling itself is a free
   template (see ``kalmanFilter.hpp``).

``kalmanFilter.hpp``
   The ``SequentialFilter<F, M>`` concept plus the scheduling free functions.
   The concept is deliberately non-prescriptive: it only requires that a
   filter has a ``timeUpdate(dt)`` and a ``measurementUpdate(m)``. It does not
   constrain their return types. What those methods must
   actually return is fixed instead by the **scheduler you choose**, so the
   scheduler — not the concept — is where the final contract on the filter's
   methods lives:

   - ``applySequential(queue, filter, callTime)`` — the basic scheduler.
     It interleaves ``timeUpdate`` / ``measurementUpdate`` relative to the
     queue's last-measurement time, so the
     updates should return ``void``. It trusts every update and always advances the
     anchor.
   - ``applySequentialRobust(queue, filter, callTime)`` — the robust scheduler.
     It reads each update's outcome, so ``timeUpdate`` and
     ``measurementUpdate`` must return a bool validity (``true`` /
     has-value on success), and the filter must expose a ``clear()`` that rewinds
     it to its last good state. On a failed update it calls ``clear()`` and
     leaves the anchor in place, so a bad measurement can neither corrupt the
     estimate nor advance the timeline.

   The upshot: the concept keeps the core open — any two update methods satisfy
   it — while the chosen scheduler drives the design of those methods. Pick
   ``applySequentialRobust`` and you are obliged to make your updates report
   validity and to implement ``clear()``; pick ``applySequential`` and plain
   ``void`` updates suffice. Other scheduling styles (batch, iterated, ...) drop
   in as further ``apply_*`` free templates, each imposing its own method
   contract, without touching the queue.

``srukf.hpp``
   The square-root uKF (van der Merwe & Wan, ICASSP 2001) in two layers:

   - a functional core — ``SrukfStorage<State>`` (plain data, all
     fixed-size) and free functions ``srukf::reset``,
     ``srukf::timeUpdate<State, D>``, ``srukf::measurementUpdate<State, M>``;
   - a state container — ``SRuKF<State, Dyn>``, which holds a
     ``SrukfStorage`` and a settable ``dynamics`` member and exposes
     ``configure()`` / ``reInitialize()`` / ``timeUpdate(dt)`` /
     ``measurementUpdate<M>(m)`` plus setters/getters and a
     ``getStateAtLastMeasurement()`` / ``setStateLastMeasurement()`` pair for
     the rolling last-measurement state.

   ``configure()`` re-derives the sigma-point spread (lambda, eta), the sigma
   weights, and the process-noise Cholesky from alpha, beta, and the process
   noise, leaving the estimate untouched; ``reInitialize()`` calls ``configure()``
   and additionally seeds the state and covariance from the initial conditions.

   ``timeUpdate(dt)`` always propagates **from the saved last-measurement
   state**, which it leaves untouched, so it is idempotent in ``dt`` — calling
   it twice with the same elapsed time produces the same posterior. Only
   ``measurementUpdate`` advances that saved state.

Statistics about the central sigma point
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The SRuKF uses :math:`2N + 1` sigma points :math:`\boldsymbol{X}_0, \ldots, \boldsymbol{X}_{2N}`, where
:math:`\boldsymbol{X}_0` is the central point and :math:`N` is the state size. With
:math:`\lambda = N(\alpha^2 - 1)`, the weights are

.. math::

   w^m_0 = \frac{\lambda}{N + \lambda}, \qquad
   w^m_i = w^c_i = \frac{1}{2(N + \lambda)} \ (i > 0), \qquad
   w^c_0 = w^m_0 + 1 - \alpha^2 + \beta

For a small :math:`\alpha`, the central weight is large and negative: :math:`w^m_0 = 1 - 1/\alpha^2`, which is
about :math:`-2499` for :math:`\alpha = 0.02`. The textbook sums :math:`\sum_i w^m_i \boldsymbol{X}_i` and
:math:`\sum_i w^c_i (\boldsymbol{X}_i - \bar{\boldsymbol{x}})(\cdot)^T` then subtract large terms that are almost
equal. The textbook square-root form also includes the central point with a rank-1 downdate by
:math:`w^c_0 \approx -2496`. In single precision, this loses most of the significant digits of the mean and the
covariance.

The SRuKF therefore writes all the statistics in terms of the deviations from the central point and the mean
offset:

.. math::

   \boldsymbol{d}_i = \boldsymbol{X}_i - \boldsymbol{X}_0, \qquad
   \boldsymbol{m} = \bar{\boldsymbol{x}} - \boldsymbol{X}_0 = \sum_{i=1}^{2N} w^m_i \boldsymbol{d}_i

Because :math:`\sum_i w^m_i = 1` and :math:`\boldsymbol{d}_0 = \boldsymbol{0}`, the covariance about the mean is
equal to

.. math::

   \sum_{i=0}^{2N} w^c_i (\boldsymbol{X}_i - \bar{\boldsymbol{x}})(\boldsymbol{X}_i - \bar{\boldsymbol{x}})^T
   = \sum_{i=1}^{2N} w^m_i \, \boldsymbol{d}_i \boldsymbol{d}_i^T + (\beta - \alpha^2) \, \boldsymbol{m} \boldsymbol{m}^T

The large negative weight does not appear. Only the small coefficient :math:`\beta - \alpha^2`
(``wMeanOffset``) remains. In the square-root form, the QR decomposition uses the weighted deviations and the
process-noise factor, and a rank-1 update adds the mean-offset term.

The measurement update uses the same expansion with :math:`\boldsymbol{u}_i = \boldsymbol{Y}_i \ominus
\boldsymbol{Y}_0` and :math:`\bar{\boldsymbol{u}} = \bar{\boldsymbol{y}} \ominus \boldsymbol{Y}_0`, where
:math:`\ominus` is the ``subtract()`` of the measurement model:

.. math::

   \bar{\boldsymbol{y}} = \boldsymbol{Y}_0 + \sum_{i=1}^{2N} w^m_i (\boldsymbol{Y}_i - \boldsymbol{Y}_0), \qquad
   P_{yy} = \sum_{i=1}^{2N} w^m_i \, \boldsymbol{u}_i \boldsymbol{u}_i^T
          + (\beta - \alpha^2) \, \bar{\boldsymbol{u}} \bar{\boldsymbol{u}}^T + R, \qquad
   P_{xy} = \sum_{i=1}^{2N} w^m_i \, \boldsymbol{d}_i \boldsymbol{u}_i^T
          + (\beta - \alpha^2) \, \boldsymbol{m} \bar{\boldsymbol{u}}^T

The identity is exact when :math:`\ominus` is linear, such as a plain vector difference. When :math:`\ominus` is
nonlinear, the textbook form linearizes it at the mean :math:`\bar{\boldsymbol{y}}` and the expanded form linearizes
it at the central point :math:`\boldsymbol{Y}_0`. For example, an attitude filter that measures MRPs uses
``subMrp``, which is nonlinear. The two forms are equal when :math:`\bar{\boldsymbol{y}} = \boldsymbol{Y}_0` (for
example after a ``timeUpdate(0)``), and their difference increases with :math:`|\bar{\boldsymbol{y}} -
\boldsymbol{Y}_0|`. For an MRP attitude filter, the relative difference in the state correction is less than
:math:`10^{-4}` for an attitude standard deviation of :math:`10^{-2}` (MRP) or less.

The expansion must be about a sigma point, so that the deviation of that point, and with it the :math:`w^c_0` term,
is zero. Centering on :math:`\bar{\boldsymbol{y}}` gives the deviation :math:`\boldsymbol{Y}_0 \ominus
\bar{\boldsymbol{y}} \approx -\bar{\boldsymbol{u}}`, which is not zero. The :math:`w^c_0 \approx -2496` term (for :math:`\alpha = 0.02`) then
returns, and the other weights add a term of about :math:`+2500 \, \bar{\boldsymbol{u}} \bar{\boldsymbol{u}}^T`
that almost cancels it. So the choice is:

- Linear :math:`\ominus`: both centerings give the same result in exact arithmetic, and only
  :math:`\boldsymbol{Y}_0` is numerically safe in fp32.
- Nonlinear :math:`\ominus` (``subMrp``): the two centerings differ slightly, because :math:`\ominus` is
  linearized at a different point. This is the cost of avoiding the cancellation, and it is small for two reasons:

  - :math:`\boldsymbol{Y}_0` is the propagated prior mean, so it is close to :math:`\bar{\boldsymbol{y}}`. The gap
    :math:`|\bar{\boldsymbol{y}} - \boldsymbol{Y}_0|` comes only from curvature in the dynamics and measurement
    model, and it is zero after ``timeUpdate(0)``.
  - :math:`\bar{\boldsymbol{y}}` is computed as :math:`\boldsymbol{Y}_0 + \sum_i w^m_i (\boldsymbol{Y}_i -
    \boldsymbol{Y}_0)` in any case. For MRPs, that weighted average is itself an approximation, so the "textbook"
    linearization at :math:`\bar{\boldsymbol{y}}` is not more correct in any strict sense. It linearizes at an
    equally approximate point.

Accumulating the RK4 sub-steps
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``propagate()`` integrates an interval in RK4 sub-steps of ``kIntegrationStep`` (at most ``kMaxNumberOfSteps`` of
them). In float, the sub-step increment can be only a few ulps of the state. For example, at a range of
:math:`5 \times 10^5` km one ulp of position is about 0.03 km, and a sub-step moves the position by a few km. The
rounding of a nearly constant increment is also nearly constant, so the error does not average out: it grows
linearly with the number of sub-steps. When the measurements do not observe that direction (for example, the range
of a heading-only filter), they never remove the bias.

``rk4Increment()`` therefore returns the increment of each sub-step separately from the state, and
``propagate()`` adds it with Kahan compensation. For sub-step :math:`k`, :math:`\boldsymbol{x}_k` is the state
before the sub-step, :math:`\Delta\boldsymbol{x}_k` is the RK4 increment, and :math:`\boldsymbol{c}_k` is the
compensation: the part of the earlier increments that the additions rounded away, with the opposite sign. No
increment has been added before the first sub-step, so the compensation starts at zero:

.. math::

   \boldsymbol{c}_0 = \boldsymbol{0}

Each sub-step then corrects its increment with the compensation, adds the corrected increment to the state, and
computes the compensation for the next sub-step:

.. math::

   \boldsymbol{y}_k = \Delta\boldsymbol{x}_k - \boldsymbol{c}_k, \qquad
   \boldsymbol{s}_k = \boldsymbol{x}_k + \boldsymbol{y}_k, \qquad
   \boldsymbol{c}_{k+1} = (\boldsymbol{s}_k - \boldsymbol{x}_k) - \boldsymbol{y}_k, \qquad
   \boldsymbol{x}_{k+1} = \boldsymbol{s}_k

:math:`\boldsymbol{y}_k` is the corrected increment (``correctedIncrement`` in the code) and :math:`\boldsymbol{s}_k`
is the rounded sum (``next``), which becomes the new state. Because :math:`\boldsymbol{s}_k` and
:math:`\boldsymbol{x}_k` are close, :math:`\boldsymbol{s}_k - \boldsymbol{x}_k` is exact in float: it is the
increment that the addition actually applied. :math:`\boldsymbol{c}_{k+1}` is therefore the rounding error of the
addition, and the next sub-step subtracts it from its increment.

In a heading-only planetary flyby, plain addition makes the float filter overconfident (NEES of 17 to 32 instead of
about 3 at closest approach and after it). With the compensation, the filter matches the double-precision filter.

.. warning::

   Kahan compensation requires that the target is not compiled with ``-ffast-math`` (or another flag that allows
   reassociation). With reassociation, the compiler can simplify :math:`\boldsymbol{c}_{k+1}` to zero.

Propagating the sigma points as deviations
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The statistics of the time update use only the deviations :math:`\boldsymbol{d}_i = \boldsymbol{X}_i -
\boldsymbol{X}_0` (see "Statistics about the central sigma point"). If each sigma point is propagated as an absolute
state and :math:`\boldsymbol{d}_i` is formed after the propagation, :math:`\boldsymbol{d}_i` keeps only the bits that
survive next to the large state. Each RK4 sub-step also rounds the sigma point to the ulp of the state, so a change
of the deviation smaller than about one ulp is lost. For example, at a range of :math:`5 \times 10^5` km one ulp of
position is about 0.03 km, while the velocity spread moves a position deviation by about :math:`3 \times 10^{-4}` km
in one sub-step. The spread of the sigma points then stops growing, and the covariance stops growing with it.

The SRuKF therefore propagates the central sigma point :math:`\boldsymbol{X}_0` with ``propagate()``, and each other
sigma point as its deviation from the central point with ``propagateDeviation()``. The deviation obeys

.. math::

   \dot{\boldsymbol{d}} = \boldsymbol{f}(t, \boldsymbol{X}_0 + \boldsymbol{d}) - \boldsymbol{f}(t, \boldsymbol{X}_0)

and is integrated on its own scale. ``propagateDeviation()`` integrates the reference :math:`\boldsymbol{X}_0` and the
deviation together. Each RK4 stage of the deviation uses the matching stage of the reference, so in exact arithmetic
the stage points :math:`\boldsymbol{X}_{0,k} + \boldsymbol{d}_k` are those of an RK4 on the absolute sigma point. Only
the rounding changes.

The SRuKF keeps the propagated deviations (``sigmaDeviations``) and the mean offset :math:`\boldsymbol{m}`
(``meanOffset``). It uses them directly in the QR decomposition, the rank-1 update, :math:`P_{xy}` and the state
update, :math:`\boldsymbol{x}^+ = \boldsymbol{X}_0 + (\boldsymbol{m} + K \boldsymbol{\nu})`, where
:math:`\boldsymbol{\nu}` is the innovation. The absolute sigma points :math:`\boldsymbol{X}_0 + \boldsymbol{d}_i` are
formed only as inputs to the measurement models. A ``timeUpdate(0)`` sets the deviations directly to
:math:`\pm \eta` times the columns of the square-root covariance, with no rounding against the state.

In a heading-only Mars flyby with absolute propagation, the float filter's covariance is 4 to 20 times larger
than that of the double-precision filter, and its estimate is less accurate by about the same factor. With deviation
propagation, the float filter matches the double-precision filter. Without it, the C++ flyby tests in which a precise
heading measurement must shrink the covariance fail.

Measurement update in Joseph form
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The textbook square-root update removes :math:`K P_{yy} K^T` from the prior with one rank-1 downdate of the
Cholesky factor per column of :math:`K S_y`, so that :math:`P^+ = P^- - K P_{yy} K^T`. A precise measurement almost
fully resolves some directions of the state. In those directions the downdate subtracts two nearly equal terms, and
in float the result can become indefinite: the downdate fails, or the covariance grows instead of shrinking.

The SRuKF therefore writes the updated covariance as a sum of squares. With the posterior deviations
:math:`\boldsymbol{z}_i = \boldsymbol{d}_i - K \boldsymbol{u}_i` and :math:`\boldsymbol{m}_z = \boldsymbol{m} - K
\bar{\boldsymbol{u}}` (the notation of "Statistics about the central sigma point"):

.. math::

   P^+ = \sum_{i=1}^{2N} w^m_i \, \boldsymbol{z}_i \boldsymbol{z}_i^T + (\beta - \alpha^2) \, \boldsymbol{m}_z
         \boldsymbol{m}_z^T + K R K^T + Q_{\text{out}}

:math:`Q_{\text{out}}` (``sqrtNoiseOutsideSigmaPoints``, stored as its square root) is the part of the prior that
the sigma points do not represent. After a propagating ``timeUpdate`` it is the process noise :math:`Q`, which
enters the prior factor through the QR decomposition but is not in the deviations. After a ``timeUpdate(0)`` it is
zero, because the sigma points are drawn from the full covariance.

To show that this is the textbook update, expand the first two terms with the expansions of :math:`P_{yy}` and
:math:`P_{xy}`, where :math:`P^-_{\sigma}` is the prior covariance carried by the sigma points:

.. math::

   \sum_{i=1}^{2N} w^m_i \, \boldsymbol{z}_i \boldsymbol{z}_i^T + (\beta - \alpha^2) \, \boldsymbol{m}_z
   \boldsymbol{m}_z^T = P^-_{\sigma} - K P_{xy}^T - P_{xy} K^T + K (P_{yy} - R) K^T

With :math:`P_{xy} = K P_{yy}` and :math:`P^- = P^-_{\sigma} + Q_{\text{out}}`, the sum is :math:`P^- - K P_{yy}
K^T`. The two forms are equal in exact arithmetic. As a sum of squares, the Joseph form stays positive
semi-definite when float rounding perturbs :math:`K`. In the square-root form, ``measurementUpdate`` computes it
with one QR decomposition of

.. math::

   \left[ \sqrt{w^m_1} \, (\boldsymbol{z}_1 \ \cdots \ \boldsymbol{z}_{2N}) \quad K S_R \quad S_{Q_{\text{out}}}
   \right]

where :math:`S_R` and :math:`S_{Q_{\text{out}}}` are the Cholesky factors of :math:`R` and :math:`Q_{\text{out}}`,
and one rank-1 update by :math:`\boldsymbol{m}_z` with the coefficient :math:`\beta - \alpha^2`.

For usual measurement noise the two forms give the same results. In a heading-only flyby with a heading noise of
:math:`3 \times 10^{-6}` or more, the float filter is equally accurate with either form. With a heading noise of
:math:`10^{-6}`, the textbook update rejects 3,000 to 20,000 updates in 50 runs and the filter diverges. The Joseph
form rejects none.

Rank-1 update by rotations
~~~~~~~~~~~~~~~~~~~~~~~~~~

The time update, :math:`S_y` and the Joseph form each finish with a rank-1 update of a lower-triangular factor
:math:`S`, by the mean offset with the coefficient :math:`c = \beta - \alpha^2`: the new factor :math:`L` satisfies

.. math::

   L L^T = S S^T + c \, \boldsymbol{v} \boldsymbol{v}^T

The simple way is to form :math:`P = S S^T + c \, \boldsymbol{v} \boldsymbol{v}^T` and factor it again. The
condition number of :math:`P` is the square of the condition number of :math:`S`, and float keeps only about 7
significant digits. In a covariance with a variance of :math:`10^{-6}` next to one of :math:`10^{4}`, the small
direction is rounded away when :math:`P` is formed, and the factor of :math:`P` is wrong in that direction.

``choleskyUpDownDate()`` therefore never forms :math:`P`. It rotates :math:`\boldsymbol{x} = \sqrt{|c|} \,
\boldsymbol{v}` into the columns of :math:`L`, starting from :math:`L = S`. For each column :math:`k` with
:math:`\ell = L_{kk}` and :math:`x_k \neq 0` (a zero :math:`x_k` needs no rotation):

- **Update** (:math:`c > 0`), a Givens rotation:

  .. math::

     r = \sqrt{\ell^2 + x_k^2}, \qquad \gamma = \ell / r, \qquad \sigma = x_k / r, \qquad L_{kk} \leftarrow r

  and for :math:`i > k`, with :math:`L_{ik}` the value before the rotation:

  .. math::

     L_{ik} \leftarrow \gamma L_{ik} + \sigma x_i, \qquad x_i \leftarrow \gamma x_i - \sigma L_{ik}

- **Downdate** (:math:`c < 0`), a hyperbolic rotation:

  .. math::

     r = \sqrt{(\ell - x_k)(\ell + x_k)}, \qquad \gamma = r / \ell, \qquad \sigma = x_k / \ell, \qquad
     L_{kk} \leftarrow r

  and for :math:`i > k`, with :math:`L_{ik}` the value after its update:

  .. math::

     L_{ik} \leftarrow (L_{ik} - \sigma x_i) / \gamma, \qquad x_i \leftarrow \gamma x_i - \sigma L_{ik}

Each rotation works on entries of the size of the factor, not of the covariance, so the condition number is not
squared. A downdate that would make the result indefinite gives :math:`|x_k| > |\ell|` and a non-finite :math:`r`,
which the finite checks of the time and measurement updates reject. Because :math:`S S^T` is never formed, these
checks also test that the covariance :math:`S S^T` is finite: :math:`S` can stay finite while :math:`S S^T`
overflows float.

The fuzz tests found the case where this matters. A flyby prior with velocity variances of :math:`10^{-6}` next to
:math:`10^{4}`, propagated for 47 s without process noise, gave a velocity variance of :math:`10^{8}` instead of
about :math:`10^{4}` when the factor was formed again. With the rotations, the result is correct and does not depend
on the process noise. For a well-conditioned covariance, the two methods give the same results.


host adapter
~~~~~~~~~~~~

``algorithms/sunlineFilter/sunlineFilter.{h,cpp}`` is the only xmera layer.
``SunlineFilter`` is a ``SysModel`` that owns the message ports and holds the
algorithm behind a ``std::unique_ptr`` (forward-declared, so the SWIG-parsed
header never sees the concept-heavy core). See filter implementation examples
for more details.

How the pieces compose
--------------------------------------------

Every filter built on ``filteringCore`` leans on the core filter capabilities.
The core supplies the estimator math, the numerical integrator, the
measurement scheduler, and the C++20 concepts that define the interfaces; a concrete
filter supplies only what is genuinely filter-specific: what is estimated, how
it evolves, and how it is observed.


Static view: concepts as interfaces
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Each concept is an interface in the core; a concrete type connects to it by
*satisfying* it — no inheritance, checked at compile time:

::

   interface (concept in filteringCore)         implementation (sunlineFilter)
   ------------------------------------------     ---------------------------
   FilterState<State>                        <==  StateVector<Position<3>,
                                                              Velocity<3>,
                                                              Bias<1>>
   Dynamics<D, State>                        <==  SunlineDynamics (s x omega)
   Measurement<M, State>                     <==  CssMeasurementModel /
                                                  RateMeasurementModel
   SequentialFilter<Filter, Measurement>     <==  SunlineFilterAlgorithm

``SunlineFilterAlgorithm`` is the single composition root. It holds the
``SRuKF`` and the retained configuration, installs the dynamics
functor onto the estimator's ``dynamics`` member in ``reset()``, satisfies
``SequentialFilter`` via public ``timeUpdate`` / ``measurementUpdate``, owns
the ``measurement_queue<Measurement, BatchSize>``, exposes the single-call
``update(currentSeconds, CssData, RateData)`` (which enqueues whichever
readings are present and runs
``applySequential(measurements, *this, currentSeconds)``), and packages the
filter's state and covariance into the plain-data ``FilterStateOutput`` /
``CssResidualsOutput`` / ``RateResidualsOutput``. If the filter needs to keep
the estimate physically meaningful, it post-processes after the queue drains —
sunline's ``regularize()`` renormalizes the heading to a unit vector and clamps
the CSS bias to its configured bounds; the SRUKF itself stays agnostic to that.


Time bookkeeping: queue ↔ SRUKF
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The scheduler keeps one anchor — the last accepted measurement — in two places
that stay in lockstep:

- ``queue.getTimeOfLastMeasurement()`` — the *time* of that measurement;
  persisted across calls.
- ``srukf.getStateAtLastMeasurement()`` — the *state* (and the matching
  covariance / sqrtCovar) at that same time; set inside
  ``srukf::measurementUpdate``.

This anchor is the last estimate the filter fully trusts, and it is never
compromised by a prediction. ``timeUpdate(dt)`` propagates a working copy of
the anchor forward by ``dt`` to produce the current output; the anchor itself is
left intact. Only ``measurementUpdate`` advances it — once a new measurement is
folded in, that posterior becomes the new anchor. So ``dt`` is always measured
from the anchor, not from the previous ``timeUpdate`` call.

Walkthrough of ``applySequential(queue, filter, callTime)``::

   Timeline (queue contents):

                       m1 (t=2)        m2 (t=4)             callTime (t=7)
                          ▼               ▼                    │
    ──•───────────────────•───────────────•────────────────────•─────→  t
      │
      tLast := queue.getTimeOfLastMeasurement()   (e.g. t=0)

   applySequential:
     tLast = queue.getTimeOfLastMeasurement()                    // = 0
     for each m in queue (earliest first):
         if m.timeTag < tLast: continue                          // drop late arrivals
         filter.timeUpdate(m.timeTag - tLast)                    // propagate from anchor
         filter.measurementUpdate(m)                             // advances the anchor
         tLast = m.timeTag                                       // local cursor
     if tLast < callTime:
         filter.timeUpdate(callTime - tLast)                     // for OUTPUT only
     queue.setTimeOfLastMeasurement(tLast)                       // persist

After the loop above runs against m1 then m2:

- inside the SRUKF: ``stateLastMeasurement`` (the anchor) = post-update state at t=4;
- on the queue: ``getTimeOfLastMeasurement() == 4``;
- the final ``filter.timeUpdate(callTime - 4) == timeUpdate(3)`` advances a
  working copy to t=7 without moving the anchor. The output messages read
  this propagated value.

On the next call, if a new measurement arrives at t=10, the scheduler issues
``filter.timeUpdate(10 - 4) == timeUpdate(6)`` — the SRUKF propagates 6 s
forward from its preserved t=4 anchor, not 3 s past the previously-output
t=7 value. Because the anchor was never compromised, no drift accumulates from
the output-only propagation.

Why the pattern is generic
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

- **The core knows nothing filter-specific.** ``measurement_queue``,
  ``propagate`` / ``rk4``, and the estimator engines are parametrized only by
  the ``Spec`` / ``State`` and driven entirely through concepts; they compile
  against any filter that satisfies the contracts.
- **A new filter implements four interfaces plus wiring.** Provide a ``State``
  (a ``StateVector<...>``), a ``Dynamics`` closure, one or more ``Measurement``
  models, and a ``Spec``; a single algorithm class wires them together,
  satisfies ``SequentialFilter`` via its ``timeUpdate`` / ``measurementUpdate``,
  owns a ``measurement_queue``, and exposes the host-facing surface. Nothing
  in the core is copied or edited.
- **Composition is by concept, not inheritance.** There are no virtual bases
  and no runtime dispatch; a type that fails a contract fails to compile at the
  call site with a concept diagnostic, and everything stays fixed-size and
  inlinable.
- **The estimator is itself swappable behind its interface.** Any estimator
  that presents the ``SequentialFilter`` pair (``timeUpdate`` /
  ``measurementUpdate``) can sit behind the algorithm, so the same queue and
  scheduling drive it unchanged; today that estimator is ``SRuKF<State>``.
- **Scheduling is a swappable axis too.** ``measurement_queue`` is a generic
  time-ordered container; each scheduling policy is a free template
  (``applySequential`` today, ``apply_batch`` / ``apply_iterated`` /
  etc. as new filter families need them). An algorithm class
  picks the one that fits, and adding a new policy is a pure addition that
  doesn't touch the queue.

Heterogeneous measurements
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A multi-sensor filter consumes more than one kind of measurement on one
timeline. ``sunlineFilter`` is an example — it folds CSS array readings
and gyro rates into the same state on a shared timeline. No core change is
needed for this; the core is agnostic of the measurement type:

- ``measurement_queue``, ``applySequential``, and ``SequentialFilter`` all
  operate over an arbitrary ``Measurement`` type.
- ``srukf::measurementUpdate<M>`` is templated per measurement model, and
  each model carries its own ``M::size``, so different observation
  dimensions already work (the CSS model is size ``MaxCss``; the rate model
  is size 3, both driven through the same path).

``std::variant`` is freestanding on the target toolchain (GCC 14+ / C++26
P2407, ``__cpp_lib_freestanding_variant``). The filter that knows its kinds names the set as a
``std::variant`` and dispatches with ``std::visit``

.. code-block:: cpp

   using Measurement = std::variant<CssMeasurement, RateMeasurement>;
   filtering::measurement_queue<Measurement, BatchSize> measurements;   // one timeline

   void measurementUpdate(Measurement const& m) {                 // SequentialFilter
       std::visit([this](auto const& meas){ this->applyMeasurement(meas); }, m);
   }
   // one private overload per kind: input struct -> model -> srukf.measurementUpdate(model)
   void applyMeasurement(CssMeasurement const&);
   void applyMeasurement(RateMeasurement const&);

Because every kind shares the one queue timeline, ``applySequential``
interleaves them in time order. The variant is over the input structs; each
``applyMeasurement`` overload builds that kind's model (the
``Measurement<M, State>`` object) and calls ``srukf.measurementUpdate`` — the
same input-struct-vs-model split a single-kind filter would use. Each overload also
records its own residuals (``lastCssResiduals`` / ``lastRateResiduals``), so the
bundled ``SunlineFilterOutput`` reports only the kinds that actually fired this
cycle.

Assumptions and Limitations
---------------------------

Heterogeneous rates share one anchor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``applySequential`` keeps a single last-measurement anchor
(``queue.getTimeOfLastMeasurement()``) for all measurement kinds, and it
drops any queued measurement whose ``timeTag`` is earlier than that anchor (the
``if m.timeTag < tLast: continue`` guard). When two kinds are ingested at
different frequencies, the faster kind repeatedly advances the shared anchor; if
the filter time step is small enough that a slower kind's sample then lands *in
the past* relative to the already-advanced anchor, that slower measurement is
silently skipped and never folded into the state.

To keep every kind observed you must either:

- **size the filter time step** so at least one of each of the N measurement kinds
  falls within a single ``update`` **call** — then all kinds are enqueued
  together and applied in time order, so none lands behind the anchor; or
- **reduce the rate of the faster kind** relative to the time step.

In other words, avoid time steps so small that a window can hold several
fast-kind samples but no slow-kind sample.

How to use a filter
-------------------

Configuring, resetting, and stepping ``SunlineFilterAlgorithm`` directly (no
xmera):

.. code-block:: cpp

   using namespace filtering::sunlineFilter;

   SunlineFilterAlgorithm::State x0;
   x0.set<filtering::Position<3>>(sHat0);                       // sun heading (unit)
   x0.set<filtering::Velocity<3>>(omega0);                      // body rate
   x0.set<filtering::Bias<1>>(Eigen::Vector<float, 1>{1.0F});  // bias state

   // Build a validated configuration (throws on invalid input). CSS geometry
   // (boresights, per-sensor scale factors, count) is latched from CSSConfig by
   // the host adapter at reset; here it is supplied directly.
   SunlineFilterConfig cfg = SunlineFilterConfig::create(
       0.02, 2.0, Q, x0, P0,            // alpha, beta, processNoise (7x7), initialState, initialCovariance (7x7)
       0.5, 1.5,                        // biasLowerBound, biasUpperBound
       nHat, scaleFactor, numCss,       // MaxCss x 3 boresights, MaxCss scale factors, count (0 .. MaxCss)
       0.1, sigmaCss, sigmaGyro);       // sensorThreshold, cssMeasurementNoiseStd, gyroMeasurementNoiseStd

   SunlineFilterAlgorithm algo(cfg);     // owns the SRUKF + measurement_queue; construction seeds the filter

   // Queue-driven path: hand the raw readings to one drive call. update() packs
   // whichever readings are present (timeTag > 0), enqueues them, and runs
   // applySequential(measurements, *this, currentSeconds) under the hood, then
   // regularizes the state and returns a single bundled snapshot.
   CssData  css;  css.timeTag  = t;  css.cosValues = cosValues;
   RateData rate; rate.timeTag = t;  rate.rate     = omegaMeas;
   SunlineFilterOutput out = algo.update(currentSeconds, css, rate);

   // Direct-stepping path: call the SequentialFilter pair yourself (this is
   // what applySequential calls under the hood).
   algo.timeUpdate(dt);                            // predict
   algo.measurementUpdate(RateMeasurement{...});   // fold a measurement in directly

   // Runtime resets / reconfiguration:
   algo.reInitializeExceptPersistentStates();  // clear pending measurements + residuals; keep state and covariance
   algo.reInitialize();                         // the above plus re-seed state and covariance from the config
   algo.setConfig(newCfg);     // swap the config and re-derive the SRUKF parameters in place

   FilterStateOutput   state   = out.filterState;   // mean + covariance
   CssResidualsOutput  cssRes  = out.cssResiduals;   // pre/post-fit; valid only if CSS fired
   RateResidualsOutput rateRes = out.rateResiduals;  // pre/post-fit; valid only if gyro fired
   // (getFilterOutput() / getLastCssResiduals() / getLastRateResiduals() expose
   //  the same snapshot outside the update call.)

How to add a new filter
-----------------------

#. **Pick the state.** Compose it from component tags, e.g.
   ``using SunlineState = StateVector<Position<3>, Velocity<3>, Bias<1>>;``.
#. **Write the I/O types** (``<filter>Specs.h``) — plain-data structs for
   the measurement(s) in and the state/residuals out, plus the dynamics
   functor that satisfies ``Dynamics<D, State>``. A multi-kind filter also
   names its ``Measurement`` variant here.
#. **Write the algorithm class** — the single composition root. Holds a
   ``SRuKF<State, Dyn>`` and an immutable validated ``Config`` (built by a
   static ``create()`` factory, supplied at construction and swappable via
   ``setConfig()``), ``reInitializeExceptPersistentStates()`` / ``reInitialize()``, the
   ``SequentialFilter`` pair (``timeUpdate(dt)`` / ``measurementUpdate(m)``), the readouts
   (``getFilterOutput`` / ``getCovariance`` / per-kind residuals), a
   ``measurement_queue<Measurement, CAPACITY>``, plus a single-call
   ``update(...)`` that runs
   ``applySequential(this->measurements, *this, callTime)`` (or a
   different ``apply_*`` if your filter family wants different scheduling).
   The ``update`` signature is filter-specific — sunline uses
   ``update(currentSeconds, CssData, RateData)`` and returns a bundled
   ``SunlineFilterOutput``. Put the measurement model(s) in the ``.cpp`` — they
   only need to *satisfy* ``Measurement<M, State>``, no base class to inherit.
   If your filter needs to keep the estimate physically meaningful, post-process
   after the queue drains (sunline renormalizes the heading and clamps the bias
   in ``regularize()``) — the SRUKF itself stays agnostic to it.
#. **Add the algorithm to its module** under ``algorithms/<filter>`` and an
   ``add_subdirectory`` entry, linking ``filteringCore``.
#. **Write a gtest** (``_tests/``) that drives the algorithm class directly,
   with no xmera — config round-trip, direct ``timeUpdate`` / ``measurementUpdate``
   stepping, and the queue-driven path through ``update(...)``. See
   ``algorithms/sunlineFilter/_tests``.
#. **Write the host adapter** under ``algorithms/<filter>`` that marshals
   messages to and from the plain-data I/O types and links the algorithm library.
