Ephemerides Recenter
=============================

This module provides functionality to transform the ephemerides of a collection of bodies
so that they are expressed relative to a new central body, rather than their original reference (e.g., Sun or Earth).

Assumptions and Limitations
---------------

The module should be used in the following circumstances: when creating a simulation with ephemeris tables mostly all
generated with respect to a central body like the Sun or Earth, but with the spacecraft table in the simulation
centered on one of these bodies. This module allows to recenter all the bodies around the desired central body.

The module works with planets around the Sun, and with a single moon around one of those planets. Furthermore all
of the bodies must be relative to the original zero base, except one moon per body if applicable (in which case the
moon is relative to its parent body).

Module Overview
---------------

The module is implemented in C++ and consists of:

- ``BodyEphemerisPayload``: A container representing a body and its ephemeris messages.
- ``EphemeridesRecenterAlgorithm``: The module class

When building a body with the BodyEphemeris class, use:

- ``bodySpiceName``: the name of the new body
- ``originalCentralBodyName``: its original zero-base (of the input message data)
- add it to the CelestialEphemerisRecenter module with ``addBodyEphemerisToRecenter``

When setting up the module:

- ``setNewZeroBase``: sets the zero base that the spacecraft is using and for all the other bodies to switch to
- ``setPreviousCommonZeroBase``: identify the previous zero base

Algorithm Logic
-------------
- Step 1: obtain the ephemeris of the requested new central body with respect to the previous common zero :math:`r_{newC/C}`. If the new central body is a moon :math:`r_{newC/C}=r_{newC/Parent}+r_{Parent/C}`.
- Step 2: for each non-moon body :math:`b_i`, compute their ephemeris relative to the new central body :math:`r_{b_i/newC}=r_{b_i/C} - r_{newC/C}`.
- Step 3: for each non-moon body :math:`b_i` that has a moon, compute its ephemeris relative to the new central body :math:`r_{m/newC}=r_{m/Parent}+r_{Parent/newC}`

Illustration Figure
-------------
.. figure:: _static/doc_fig.png
   :width: 90%
   :alt: illustration figure of the algorithm logic

   Illustrating the algorithm logic by Sun-Earth-Moon.

Usage Example
-------------

Below is an example of using the Python interface (mocked for testing) to interact with the C++ module:

.. code-block:: python

    from celestialEphemerisRecenter import CelestialEphemerisRecenter, BodyEphemeris

    # Create module and celestial body
    module = CelestialEphemerisRecenter()
    body = BodyEphemeris()
    body.bodySpiceName = "Mars"
    body.originalCentralBodyName = "Sun"

    # Attach message functors (e.g., input/output links in Basilisk)
    body.inputEphemerisMsg = YourInputMsgFunctor()
    body.outputEphemerisMsg = YourOutputMsg()

    # Register body and configure module
    module.addBodyEphemerisToRecenter(body)
    module.setNewZeroBase("Mars")
    module.setPreviousCommonZeroBase("Sun")

    # Run module (e.g., in Basilisk dynamics loop)
    module.updateState(callTime)

Notes
-----
\- The maximum number of bodies is limited to 20.
