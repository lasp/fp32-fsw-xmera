Executive Summary
-----------------

The ``oeStateEphem`` module computes spacecraft position and velocity by evaluating Chebyshev polynomial
representations of classical orbital elements. The central body is defined by the gravitational parameter provided.
Given a query time, the module selects the appropriate temporal arc, evaluates the polynomial coefficients for each
orbital element, and converts the resulting elements to Cartesian coordinates.

Although table fits which use spice trajectories are typically in km and km/s, all the inputs of this module are SI:
meters for radius of periapsis, radians for angles, and m^3/s^2 for gravitational parameter.

The module maintains multiple time-segmented arcs, each containing Chebyshev polynomial coefficients for the
six classical orbital elements: radius of periapsis, eccentricity, inclination, argument of periapsis, right
ascension of the ascending node, and anomaly. Ephemeris time is computed as:

.. math::

   t_{eph, corr} = (t_{call} \times 10^{-9}) + t_{eph} - t_{vehicle}

The module identifies the arc with minimum :math:`|t_{eph, corr} - t_{arc,middle}|` and evaluates the corresponding
coefficients. If the query time falls outside an arc's valid range, the scaled time is clamped to ±1, ensuring
the module always returns a valid state.

Chebyshev Polynomial Representation
------------------------------------

Each orbital element is represented as a Chebyshev polynomial expansion:

.. math::

   OE(t) = \sum_{k=0}^{n-1} c_k \cdot T_k(t_{scaled})

where :math:`c_k` are the stored Chebyshev coefficients, :math:`T_k(x)` is the Chebyshev polynomial of the
first kind of degree :math:`k`, and :math:`t_{scaled}` is the normalized time within the arc range [-1, 1].

The Chebyshev polynomials of the first kind are defined by the recurrence relation:

.. math::

   T_0(x) &= 1 \\
   T_1(x) &= x \\
   T_{k+1}(x) &= 2x \cdot T_k(x) - T_{k-1}(x)

For example, :math:`T_2(x) = 2x^2 - 1`, :math:`T_3(x) = 4x^3 - 3x`, and so forth. The first coefficient
:math:`c_0` represents the constant term, :math:`c_1` the linear term, :math:`c_2` the quadratic term, etc.

Time within each arc is normalized to [-1, 1] using:

.. math::

   t_{scaled} = \frac{t_{eph} - t_{arc,middle}}{t_{arc,radius}}

This normalization ensures optimal numerical stability for Chebyshev polynomial evaluation.


Message Connection Descriptions
--------------------------------

The following table lists all module input and output messages. Message connections are set by the
user from Python.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - stateFitOutMsg
      - :ref:`EphemerisMsgPayload`
      - Output ephemeris message containing position and velocity in a consistent frame
    * - clockCorrInMsg
      - :ref:`TDBVehicleClockCorrelationMsgPayload`
      - Input message providing ephemeris and vehicle clock time correlation


Module Assumptions and Limitations
-----------------------------------

The module assumes:

* Chebyshev coefficients accurately represent the orbital trajectory over each arc's valid time range
* All positions and velocities are expressed in the inertial reference frame
* Orbital elements are defined relative to a single central body with gravitational parameter μ

Limitations:

* Arc transitions may introduce small discontinuities if arcs are not smoothly connected


Algorithm
---------

The ephemeris computation proceeds as follows:

1. **Central body detection**: Return zero state if all radius of periapsis coefficients across all arcs are zero

2. **Arc selection**: Compute :math:`t_{eph}` and select the arc minimizing :math:`|t_{eph} - t_{arc,middle}|`

3. **Time scaling**: Normalize time to :math:`t_{scaled} = (t_{eph} - t_{arc,middle}) / t_{arc,radius}`, clamping to ±1 if necessary

4. **Coefficient evaluation**: For each orbital element, evaluate the Chebyshev polynomial expansion using the stored coefficients

5. **Anomaly conversion**: If the anomaly flag indicates mean anomaly, convert to true anomaly using the appropriate transformation (elliptic or hyperbolic)

6. **Semi-major axis calculation**: Compute :math:`a = r_p / (1 - e)` for elliptic/hyperbolic orbits, or :math:`a = 0` for parabolic orbits

7. **Cartesian conversion**: Transform orbital elements :math:`(a, e, i, \omega, \Omega, \nu)` to position and velocity vectors :math:`(\mathbf{r}, \mathbf{v})`


User Guide
----------

The central-body gravitational parameter and the number of arcs are public properties set before
``Reset``. Each arc's parameters and Chebyshev coefficients are set through the indexed ``setArc*`` methods;
the coefficient arrays always have length ``MAX_OE_COEFF`` (20), padded with zeros beyond the active count.
The ephemeris and vehicle time offsets are read once at ``Reset`` from ``clockCorrInMsg``.

.. code-block:: python

    from xmera.fp32 import oeStateEphemF32

    ephemObject = oeStateEphemF32.OEStateEphem()
    ephemObject.modelTag = "spacecraftEphemeris"
    ephemObject.centralBodyGravitationalParameter = 3.986004418e14  # m^3/s^2 (Earth)

    # Configure arc 0 (units: m for r_p, radians for angles)
    arcIndex = 0
    nCheb = 5
    pad = [0.0] * (20 - nCheb)
    ephemObject.setArcNumberOfCoefficients(arcIndex, nCheb)
    ephemObject.setArcMiddleTime(arcIndex, 1000.0)  # seconds
    ephemObject.setArcRadiusTime(arcIndex, 500.0)   # seconds
    ephemObject.setArcAnomalyFlag(arcIndex, oeStateEphemF32.TRUE_ANOMALY)  # or oeStateEphemF32.MEAN_ANOMALY

    ephemObject.setArcRadiusPeriapsisCoefficients(arcIndex, [7000000.0, 0.0, 0.0, 0.0, 0.0] + pad)
    ephemObject.setArcEccentricityCoefficients(arcIndex, [0.001, 0.0, 0.0, 0.0, 0.0] + pad)
    ephemObject.setArcInclinationCoefficients(arcIndex, [0.5, 0.0, 0.0, 0.0, 0.0] + pad)
    ephemObject.setArcArgPeriapsisCoefficients(arcIndex, [0.0, 0.0, 0.0, 0.0, 0.0] + pad)
    ephemObject.setArcRaanCoefficients(arcIndex, [0.0, 0.0, 0.0, 0.0, 0.0] + pad)
    ephemObject.setArcTrueAnomalyCoefficients(arcIndex, [0.0, 0.1, 0.0, 0.0, 0.0] + pad)

    # Connect messages
    ephemObject.clockCorrInMsg.subscribeTo(clockCorrMsg)

For trajectories requiring multiple arcs, set ``ephemObject.numberOfArcs`` and configure additional arc
indices with appropriate time ranges and coefficients. The module automatically selects the nearest arc
during evaluation. Consult the module's Python test files for additional configuration examples.
