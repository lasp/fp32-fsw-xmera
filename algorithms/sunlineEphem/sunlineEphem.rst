Overview
--------
``sunlineEphem`` computes the Sun direction unit vector relative to the spacecraft from ephemeris data. It gives
this direction in the inertial frame and in the body frame. Downstream guidance and control modules can use these
directions as a reference.

The module layer (``SunlineEphem``) implements the SysModel interface, wiring messages and managing module
lifecycle. The algorithm (``SunlineEphemAlgorithm``) contains the stateless computation that can be used
stand-alone when the caller provides position and attitude data directly.

Module Inputs and Outputs
-------------------------
Module inputs:

.. list-table:: Module Input Messages
   :widths: 25 35 40
   :header-rows: 1

   * - Msg Variable Name
     - Msg Type
     - Description
   * - sunPositionInMsg
     - :ref:`EphemerisMsgF32Payload`
     - Read every ``updateState()``. Contains the Sun's position expressed in the inertial frame.
   * - scPositionInMsg
     - :ref:`NavTransMsgF32Payload`
     - Read every ``updateState()``. Contains the spacecraft's position expressed in the inertial frame.
   * - scAttitudeInMsg
     - :ref:`NavAttMsgF32Payload`
     - Read every ``updateState()``. Contains the spacecraft's attitude (MRPs), used to rotate the Sun vector
       from the inertial frame to body frame.

Module output:

.. list-table:: Module Output Messages
   :widths: 25 35 40
   :header-rows: 1

   * - Msg Variable Name
     - Msg Type
     - Description
   * - navStateOutMsg
     - :ref:`NavAttMsgF32Payload`
     - Ephemeris-based Sun direction unit vector expressed in the body frame, written to ``vehSunPntBdy``.
   * - sunHeadingOutMsg
     - :ref:`InertialHeadingMsgF32Payload`
     - Ephemeris-based Sun direction unit vector expressed in the inertial frame, written to ``rHat_XN_N``.

Algorithm Inputs and Outputs
----------------------------
Algorithm inputs:

.. list-table::
   :widths: 25 75
   :header-rows: 1

   * - Variable Name
     - Description
   * - r_SN_N
     - Sun position vector expressed in the inertial frame :math:`{}^{N}\boldsymbol{r}_{S/N}`.
   * - r_BN_N
     - Spacecraft position vector expressed in the inertial frame :math:`{}^{N}\boldsymbol{r}_{B/N}`.
   * - sigma_BN
     - Spacecraft attitude MRPs :math:`\boldsymbol{\sigma}_{BN}`, used to rotate the Sun direction from
       the inertial frame to the body frame.

Algorithm output:

.. list-table::
   :widths: 25 75
   :header-rows: 1

   * - Variable Name
     - Description
   * - rHat_SB_N
     - Sun direction unit vector expressed in the inertial frame :math:`{}^{N}\boldsymbol{\hat r}_{S/B}`.
       It is the zero vector if the spacecraft and Sun are colocated.
   * - rHat_SB_B
     - Sun direction unit vector expressed in the body frame :math:`{}^{B}\boldsymbol{\hat r}_{S/B}`.
       It is the zero vector if the spacecraft and Sun are colocated.

Algorithm Assumptions
---------------------
- All input position vectors must be expressed in the same inertial reference frame.

Algorithm Description
---------------------
Given Sun position :math:`{}^{N}\boldsymbol{r}_{S/N}`, spacecraft position :math:`{}^{N}\boldsymbol{r}_{B/N}`, and
spacecraft attitude MRPs :math:`\boldsymbol{\sigma}_{BN}`:

#. Compute the Sun position relative to the spacecraft in the inertial frame:

   .. math:: {}^{N}\boldsymbol{r}_{S/B} = {}^{N}\boldsymbol{r}_{S/N} - {}^{N}\boldsymbol{r}_{B/N}

#. Normalize the relative vector to get the inertial-frame Sun direction:

   .. math:: {}^{N}\boldsymbol{\hat r}_{S/B} = \text{normalize}\left({}^{N}\boldsymbol{r}_{S/B}\right)

#. Build the DCM from the spacecraft attitude MRPs and rotate the unit direction into the body frame:

   .. math::

      {}^{B}\boldsymbol{\hat r}_{S/B} =
      \text{normalize}\left([BN] \, {}^{N}\boldsymbol{\hat r}_{S/B}\right)

Normalization uses a numerically stable algorithm, ``stableNormalized()``, that returns the zero vector if
:math:`{}^{N}\boldsymbol{r}_{S/B}` is zero.

The algorithm returns unit direction vectors for distinct positions. For colocated positions (truly identical
positions), both outputs are the zero vector, as a sentinel. Downstream consumers should check for this case.
The inertial-frame direction does not use the attitude.

.. _ModuleIO_sunlineEphem_overview:
.. figure:: /../../src/fswAlgorithms/attDetermination/sunlineEphem/_Documentation/Figures/sunlineEphem_rSB_N.jpeg
    :scale: 50 %
    :align: center

    Figure 1: SunlineEphem computes the inertial-frame Sun direction vector :math:`{}^{N}\boldsymbol{r}_{S/B}`.

Module vs. Algorithm Usage
--------------------------
- Module (``SunlineEphem``, SysModel): use when operating inside the messaging framework. Connect
  ``sunPositionInMsg``, ``scPositionInMsg``, and ``scAttitudeInMsg``, call
  ``InitializeSimulation()``/``ExecuteSimulation()``, and read ``navStateOutMsg`` (body frame) and
  ``sunHeadingOutMsg`` (inertial frame).
- Algorithm (``SunlineEphemAlgorithm``): use when you have position and attitude data directly. Call the
  static ``update(r_SN_N, r_BN_N, sigma_BN)`` method. It returns a ``SunlineEphemOutput`` that holds the Sun
  direction in the inertial frame (``rHat_SB_N``) and in the body frame (``rHat_SB_B``).
