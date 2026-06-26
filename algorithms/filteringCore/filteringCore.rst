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

For example, the kalmanFilter header file contains the applySequential function which orchestrates the
calls of time and measurement updates on a queue of measurements. These functions can call the srukf (other header)
definitions of those functions to perform those specific updates.

With that, the implementation of a filter which has specific dynamics, state vector, and measurements
can be done at the module/algorithm level all the while relying on these core capabilities.

Composable structure
-----------------

::

   +-------------------------------------------------------------+
   | xmera host adapter   (algorithms/sunlineSRuKF/              |
   |                       sunlineSRuKF.{h,cpp})                  |
   |   - SunlineSRuKF : SysModel, ReadFunctor<>, Message<>       |
   |   - marshals MsgPayloads <-> plain-data I/O types           |
   +----------------------------+--------------------------------+
                                | owns (pimpl)
                                v
   +-------------------------------------------------------------+
   | algorithm library   (algorithms/sunlineSRuKF/              |
   |                       sunlineSRuKFAlgorithm.{h,cpp})         |
   |   - SunlineSRuKFAlgorithm  (stateful, plain C++ class)       |
   |   - sunlineSRuKFSpecs.h    (State, dynamics, I/O types,     |
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

The host adapter and the algorithm library live in the **same directory**
(``algorithms/sunlineSRuKF``) and compile into one SWIG module, but they stay
cleanly separable: ``sunlineSRuKF.{h,cpp}`` is the only xmera-aware translation
unit, and ``sunlineSRuKFAlgorithm.{h,cpp}`` / ``sunlineSRuKFSpecs.h`` depend
only on ``filteringCore``.

filteringCore
~~~~~~~~~~~~~~~

Header-only ``INTERFACE`` library (``algorithms/filteringCore``). The headers
sit at the top of that directory and are included as ``<filteringCore/...>``,
which the parent ``algorithms/`` include path resolves.

``state.hpp``
   ``StateVector<Components...>`` is a variadic, type-composed state vector laid
   out contiguously in a single fixed-size ``Eigen::Vector``. Components are
   addressed by their tag type at compile time:

   .. code-block:: cpp

      using SunlineState = filtering::StateVector<filtering::Position<3>,
                                                  filtering::Velocity<3>,
                                                  filtering::Bias<1>>;  // size 7
      SunlineState s;
      s.set<filtering::Position<3>>(Eigen::Vector3d{0, 0, 1});  // sun heading
      Eigen::Vector3d omega = s.get<filtering::Velocity<3>>();  // body rate

   Component tags (``Position``, ``Velocity``, ``Acceleration``, ``Bias``,
   ``MrpAttitude``, ``AngularRate``, ``Consider``) each carry a compile-time
   ``size``. ``StateVector`` satisfies ``LinearlyCombinable`` via ``add()`` and
   ``scale()`` and exposes ``raw()`` for the underlying Eigen vector.

``concepts.hpp``
   The C++20 concepts that define the interfaces, so filters compose by
   *satisfying an interface* rather than by inheritance:

   - ``LinearlyCombinable<T>`` — has ``scale(k)`` and ``add(other)``.
   - ``FilterState<S>`` — ``LinearlyCombinable`` plus ``S::size`` and ``raw()``.
   - ``Measurement<M, State>`` — ``M::size``, ``observation()``,
     ``model(state)``, ``noise()``, ``subtract(a, b)``.
   - ``Dynamics<D, State>`` — invocable ``(double, State) -> State``.

``dynamicsModel.hpp``
   ``DynamicsModel<State>`` (a ``std::function`` alias) plus ``rk4()`` and
   ``propagate()`` — concept-constrained RK4 integration usable on any
   ``LinearlyCombinable`` state.

``measurementQueue.h``
   ``measurement_queue<Measurement, CAPACITY>`` — a bounded, time-ordered
   container (``enqueue`` / ``popEarliest`` / ``clear`` / ``isFull`` /
   ``isEmpty``) plus a persistent ``getTimeOfLastMeasurement()`` /
   ``setTimeOfLastMeasurement()`` pair: the time of the last measurement
   the scheduler folded into the filter. Scheduling itself is a free
   template (see ``kalmanFilter.hpp``).

``kalmanFilter.hpp``
   The ``SequentialFilter<F, M>`` concept — the contract a Kalman-style filter
   satisfies (``timeUpdate(dt)`` + ``measurementUpdate(m)``).
   It also defines ``applySequential(queue, filter, callTime)`` — the canonical scheduler
   that interleaves time and measurement updates relative to the queue's
   last-measurement time. Alternative scheduling styles (batch, iterated, ...)
   drop in as new ``apply_*`` free templates without touching the queue.

``srukf.hpp``
   The square-root uKF (van der Merwe & Wan, ICASSP 2001) in two layers:

   - a **functional core** — ``SrukfStorage<State>`` (plain data, all
     fixed-size) and free functions ``srukf::reset``,
     ``srukf::timeUpdate<State, D>``, ``srukf::measurementUpdate<State, M>``;
   - a **stateful facade** — ``SRuKF<State, Dyn>``, which holds a
     ``SrukfStorage`` and a settable ``dynamics`` member and exposes
     ``reConfigure()`` / ``reset()`` / ``timeUpdate(dt)`` /
     ``measurementUpdate<M>(m)`` plus setters/getters and a
     ``getStateAtLastMeasurement()`` / ``setStateLastMeasurement()`` pair for
     the rolling last-measurement state.

   ``reConfigure()`` re-derives the sigma-point spread (lambda, eta), the sigma
   weights, and the process-noise Cholesky from alpha, beta, and the process
   noise, leaving the estimate untouched; ``reset()`` calls ``reConfigure()``
   and additionally seeds the state and covariance from the initial conditions.

   ``timeUpdate(dt)`` always propagates **from the saved last-measurement
   state**, which it leaves untouched, so it is idempotent in ``dt`` — calling
   it twice with the same elapsed time produces the same posterior. Only
   ``measurementUpdate`` advances that saved state.


host adapter
~~~~~~~~~~~~

``algorithms/sunlineSRuKF/sunlineSRuKF.{h,cpp}`` is the only xmera-aware layer.
``SunlineSRuKF`` is a ``SysModel`` that owns the message ports and holds the
algorithm behind a ``std::unique_ptr`` (forward-declared, so the SWIG-parsed
header never sees the concept-heavy core). At ``reset`` it latches the CSS
geometry (boresights, per-sensor scale factors, count) from ``cssConfigInMsg``,
builds and validates a ``SunlineSRuKFConfig``, and constructs the algorithm.
Each ``updateState`` it reads the gyro (``navAttInMsg``) and CSS
array (``cssDataInMsg``) payloads into ``RateData`` / ``CssData`` (skipping
stale readings via per-channel timeTag tracking) and drives the filter over the
window with a single ``update(currentSeconds, cssData, rateData)`` call. The
algorithm owns the ``measurement_queue`` and runs
``applySequential(measurements, *this, currentSeconds)`` inside that ``update``
call; ``applySequential`` decides per step whether to ``timeUpdate`` (predict)
or ``measurementUpdate``. The adapter then writes the returned
``SunlineSRuKFOutput`` (state + covariance + per-kind residuals) back into the
``navAttOutMsg`` / ``filterOutMsg`` / ``filterGyroResOutMsg`` /
``filterCssResOutMsg`` payloads. It also exposes ``reInitialize()`` /
``reInitializeAll()`` pass-throughs for restarting the filter at runtime.

How the pieces compose
--------------------------------------------

Every filter built on ``filteringCore`` leans on the core filter capabilities.
The core supplies the estimator math, the numerical integrator, the
measurement scheduler, and the C++20 concepts that define the interfaces; a concrete
filter supplies only what is genuinely filter-specific: what is estimated, how
it evolves, and how it is observed.

Roles, interfaces, and what sunline provides
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 24 28 24 24

   * - Role
     - Contract (concept)
     - Core consumer
     - SunlineSRuKF supplies
   * - What is estimated
     - ``FilterState<S>``
     - ``SRuKF<State, Dyn>``, ``propagate``
     - ``StateVector<Position<3>, Velocity<3>, Bias<1>>``
   * - How the state evolves
     - ``Dynamics<D, State>``
     - ``rk4`` / ``propagate`` (in ``srukf::timeUpdate``)
     - ``SunlineDynamics`` (``ds/dt = s × ω``)
   * - How the state is observed
     - ``Measurement<M, State>``
     - ``srukf::measurementUpdate``
     - ``CssMeasurementModel`` and ``RateMeasurementModel``
   * - The drivable filter
     - ``SequentialFilter<F, M>`` (``timeUpdate`` + ``measurementUpdate``)
     - ``applySequential`` (free template)
     - ``SunlineSRuKFAlgorithm``
   * - When to time-update vs. measurement-update
     - — (concrete, not a concept)
     - ``measurement_queue`` (container) + ``applySequential`` (free function)
     - ``SunlineSRuKFAlgorithm`` owns one ``measurement_queue<Measurement, BatchSize>`` and runs ``applySequential(measurements, *this, currentSeconds)`` from its ``update``

The first three rows are the interfaces a new filter must implement; everything
in the "Core consumer" column is reused unchanged.

Static view: concepts as interfaces
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Each concept is an interface in the core; a concrete type connects to it by
*satisfying* it — no inheritance, checked at compile time:

::

   interface (concept in filteringCore)         implementation (sunlineSRuKF)
   ------------------------------------------     ---------------------------
   FilterState<State>                        <==  StateVector<Position<3>,
                                                              Velocity<3>,
                                                              Bias<1>>
   Dynamics<D, State>                        <==  SunlineDynamics (s x omega)
   Measurement<M, State>                     <==  CssMeasurementModel /
                                                  RateMeasurementModel
   SequentialFilter<Filter, Measurement>     <==  SunlineSRuKFAlgorithm

``SunlineSRuKFAlgorithm`` is the single composition root. It holds the
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

Runtime view: one update window
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A single ``update(currentSeconds, cssData, rateData)`` call does three things:
it enqueues the fresh readings, drains the queue through the filter in time
order, and returns the post-update snapshot. The four schematics below break
that down — first the top-level flow across the layers, then what
``applySequential`` drives, then the two update call chains it drives into.

**1. Top-level flow — one call across the layers**

::

   Xmera adapter                         SunlineSRuKFAlgorithm
   ------------                          ---------------------
   read NavAtt + CSS msgs
   build RateData / CssData
   algo.update(t, css, rate) ─────────►  enqueue fresh readings
                                         applySequential(queue, *this, t)
                                         (drives the two updates below)
   write output msgs         ◄─────────  return SunlineSRuKFOutput
                                         (state + covariance + residuals)

**2. What ``applySequential`` drives, per queued step**

It walks the queue earliest-first and is the only thing that decides whether the
next action is a prediction or a correction:

::

   for each queued measurement m (earliest first):
       filter.timeUpdate(m.timeTag - tLast)   // advance to m's time
       filter.measurementUpdate(m)            // fold m in
       tLast = m.timeTag
   filter.timeUpdate(t - tLast)               // advance to the output time

**3. A time update — the prediction call chain**

::

   algo.timeUpdate(dt)
     └─ srukf.timeUpdate(dt)
          └─ srukf::timeUpdate
               └─ propagate ─► rk4 ─► dynamics(t, x):  ṡ = s × ω

**4. A measurement update — the correction call chain**

The measurement is a ``std::variant``; ``std::visit`` dispatches to the overload
for the kind that arrived, which builds that kind's model and hands it to the
estimator:

::

   algo.measurementUpdate(m)         // m : variant<CssMeasurement, RateMeasurement>
     └─ std::visit ─► applyMeasurement(kind)
          └─ srukf.measurementUpdate(model)
               └─ srukf::measurementUpdate
                    └─ model.model(x) / model.noise() / model.subtract(a, b)

The queue is just a container, and the algorithm doesn't sequence anything
itself — it only exposes the ``SequentialFilter`` pair that ``applySequential``
calls. A filter family that wants different scheduling (batch, iterated,
particle resample, ...) gets a different ``apply_*`` free template (constrained
on whatever concept that family needs) and reuses the same container.
The estimator behind the algorithm is likewise an internal detail: it only has
to make ``timeUpdate`` / ``measurementUpdate`` do the right thing, and the
queue, the scheduling function, the integrator, and the concepts don't depend
on which estimator is inside.

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
compromised by a prediction. ``timeUpdate(dt)`` propagates a *working copy* of
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
  working copy to t=7 **without** moving the anchor. The output messages read
  this propagated value.

On the next call, if a new measurement arrives at t=10, the scheduler issues
``filter.timeUpdate(10 - 4) == timeUpdate(6)`` — the SRUKF propagates 6 s
forward from its preserved t=4 anchor, **not** 3 s past the previously-output
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

A multi-sensor filter consumes more than one *kind* of measurement on one
timeline. ``sunlineSRuKF`` is exactly this case — it folds CSS array readings
and gyro rates into the same state on a shared timeline. No core change is
needed for this; the core is kind-agnostic:

- ``measurement_queue``, ``applySequential``, and ``SequentialFilter`` all
  operate over an arbitrary ``Measurement`` type.
- ``srukf::measurementUpdate<M>`` is templated per measurement model, and
  each model carries its own ``M::size``, so different observation
  dimensions already work (the CSS model is size ``MaxCss``; the rate model
  is size 3, both driven through the same path).

``std::variant`` is freestanding on the target toolchain (GCC 14+ / C++26
P2407, ``__cpp_lib_freestanding_variant``), so it is an appropriate closed-set
mechanism here. The filter that knows its kinds names the set as a
``std::variant`` and dispatches with ``std::visit`` — fixed-size, no heap, no
virtual:

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
interleaves them in time order. The variant is over the **input structs**; each
``applyMeasurement`` overload builds that kind's **model** (the
``Measurement<M, State>`` object) and calls ``srukf.measurementUpdate`` — the
same input-struct-vs-model split a single-kind filter would use. Each overload also
records its own residuals (``lastCssResiduals`` / ``lastRateResiduals``), so the
bundled ``SunlineSRuKFOutput`` reports only the kinds that actually fired this
cycle.

A worked, compiling reference (real SRUKF, two observation sizes ``MaxCss``
and 3) lives in
``algorithms/sunlineSRuKF/_tests/test_sunlineSRuKFAlgorithm.cpp``.

How to use a filter
-------------------

Configuring, resetting, and stepping ``SunlineSRuKFAlgorithm`` directly (no
xmera):

.. code-block:: cpp

   using namespace filtering::sunlineSRuKF;

   SunlineSRuKFAlgorithm::State x0;
   x0.set<filtering::Position<3>>(sHat0);                       // sun heading (unit)
   x0.set<filtering::Velocity<3>>(omega0);                      // body rate
   x0.set<filtering::Bias<1>>(Eigen::Vector<double, 1>{1.0});   // bias state

   // Build a validated configuration (throws on invalid input). CSS geometry
   // (boresights, per-sensor scale factors, count) is latched from CSSConfig by
   // the host adapter at reset; here it is supplied directly.
   SunlineSRuKFConfig cfg = SunlineSRuKFConfig::create(
       0.02, 2.0, Q, x0, P0,            // alpha, beta, processNoise (7x7), initialState, initialCovariance (7x7)
       0.5, 1.5,                        // biasLowerBound, biasUpperBound
       nHat, scaleFactor, numCss,       // MaxCss x 3 boresights, MaxCss scale factors, count (0 .. MaxCss)
       0.1, sigmaCss, sigmaGyro);       // sensorThreshold, cssMeasurementNoiseStd, gyroMeasurementNoiseStd

   SunlineSRuKFAlgorithm algo(cfg);     // owns the SRUKF + measurement_queue; construction seeds the filter

   // Queue-driven path: hand the raw readings to one drive call. update() packs
   // whichever readings are present (timeTag > 0), enqueues them, and runs
   // applySequential(measurements, *this, currentSeconds) under the hood, then
   // regularizes the state and returns a single bundled snapshot.
   CssData  css;  css.timeTag  = t;  css.cosValues = cosValues;
   RateData rate; rate.timeTag = t;  rate.rate     = omegaMeas;
   SunlineSRuKFOutput out = algo.update(currentSeconds, css, rate);

   // Direct-stepping path: call the SequentialFilter pair yourself (this is
   // what applySequential calls under the hood).
   algo.timeUpdate(dt);                            // predict
   algo.measurementUpdate(RateMeasurement{...});   // fold a measurement in directly

   // Runtime resets / reconfiguration:
   algo.reInitialize();        // clear pending measurements + residuals; keep state and covariance
   algo.reInitializeAll();     // the above plus re-seed state and covariance from the config
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
   ``setConfig()``), ``reInitialize()`` / ``reInitializeAll()``, the
   ``SequentialFilter`` pair (``timeUpdate(dt)`` / ``measurementUpdate(m)``), the readouts
   (``getFilterOutput`` / ``getCovariance`` / per-kind residuals), a
   ``measurement_queue<Measurement, CAPACITY>``, plus a single-call
   ``update(...)`` that runs
   ``applySequential(this->measurements, *this, callTime)`` (or a
   different ``apply_*`` if your filter family wants different scheduling).
   The ``update`` signature is filter-specific — sunline uses
   ``update(currentSeconds, CssData, RateData)`` and returns a bundled
   ``SunlineSRuKFOutput``. Put the measurement model(s) in the ``.cpp`` — they
   only need to *satisfy* ``Measurement<M, State>``, no base class to inherit.
   If your filter needs to keep the estimate physically meaningful, post-process
   after the queue drains (sunline renormalizes the heading and clamps the bias
   in ``regularize()``) — the SRUKF itself stays agnostic to it.
#. **Add the algorithm to its module** under ``algorithms/<filter>`` and an
   ``add_subdirectory`` entry, linking ``filteringCore``.
#. **Write a gtest** (``_tests/``) that drives the algorithm class directly,
   with no xmera — config round-trip, direct ``timeUpdate`` / ``measurementUpdate``
   stepping, and the queue-driven path through ``update(...)``. See
   ``algorithms/sunlineSRuKF/_tests``.
#. **Write the host adapter** under ``algorithms/<filter>`` that marshals
   messages to and from the plain-data I/O types and links the algorithm library.

.. note::

   **Variable-size measurements** (e.g. a varying number of active coarse sun
   sensors) are handled with a fixed-capacity ``Eigen::Vector<double, MAX>`` and
   a runtime count, sliced with ``.head()`` / ``.topRows()`` — not a dynamically
   sized Eigen type. ``sunlineSRuKF`` exercises this directly: its
   ``CssMeasurement`` carries an ``Eigen::Vector<double, MaxCss>`` plus a
   ``numberOfActiveCss`` count, and ``packCssMeasurement`` fills only the active
   leading rows (those above ``sensorUseThresh``).
