Overview
--------
``sunlineEphem`` computes the Sun direction unit vector relative to the spacecraft in the body frame based on
ephemeris data. This direction vector can be used as a reference for downstream guidance and control modules.

The module layer (``SunlineEphem``) implements the SysModel interface, wiring messages and managing module
lifecycle. The algorithm (``SunlineEphemAlgorithm``) contains the stateless computation that can be used
stand-alone when the caller provides position and attitude data directly.

Module Inputs and Outputs
-------------------------
Module inputs:

.. list-table:: Module Input Messages
   :widths: 25 25 50
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
   :widths: 25 25 50
   :header-rows: 1

   * - Msg Variable Name
     - Msg Type
     - Description
   * - navStateOutMsg
     - :ref:`NavAttMsgF32Payload`
     - Ephemeris-based Sun direction unit vector expressed in the body frame, written to ``vehSunPntBdy``.

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
   * - rHat_SB_B
     - Sun direction unit vector expressed in the body frame :math:`{}^{B}\boldsymbol{\hat r}_{S/B}`.
       Returns the zero vector if the spacecraft and Sun are colocated.

Algorithm Assumptions
---------------------
- All input position vectors must be expressed in the same inertial reference frame.

Algorithm Description
---------------------
Given Sun position :math:`{}^{N}\boldsymbol{r}_{S/N}`, spacecraft position :math:`{}^{N}\boldsymbol{r}_{B/N}`, and
spacecraft attitude MRPs :math:`\boldsymbol{\sigma}_{BN}`:

#. Compute the Sun position relative to the spacecraft in the inertial frame:

   .. math:: {}^{N}\boldsymbol{r}_{S/B} = {}^{N}\boldsymbol{r}_{S/N} - {}^{N}\boldsymbol{r}_{B/N}

#. If :math:`\|{}^{N}\boldsymbol{r}_{S/B}\| \le \epsilon`, return the zero vector. Otherwise, normalize the
   relative vector, build the DCM from the spacecraft attitude MRPs, and rotate the unit direction into the
   body frame:

   .. math::

      {}^{B}\boldsymbol{\hat r}_{S/B} =
      \text{normalize}\left([BN] \, {}^{N}\boldsymbol{\hat r}_{S/B}\right)

The algorithm returns a unit direction vector for distinct positions. For colocated positions, the zero vector
is returned as a sentinel when no direction can be computed. Downstream consumers should check for this case.

.. _ModuleIO_sunlineEphem_overview:
.. figure:: /../../src/fswAlgorithms/attDetermination/sunlineEphem/_Documentation/Figures/sunlineEphem_rSB_N.jpeg
    :scale: 50 %
    :align: center

    Figure 1: SunlineEphem computes the inertial-frame Sun direction vector :math:`{}^{N}\boldsymbol{r}_{S/B}`.

Module vs. Algorithm Usage
--------------------------
- Module (``SunlineEphem``, SysModel): use when operating inside the messaging framework. Connect
  ``sunPositionInMsg``, ``scPositionInMsg``, and ``scAttitudeInMsg``, call
  ``InitializeSimulation()``/``ExecuteSimulation()``, and read ``navStateOutMsg``.
- Algorithm (``SunlineEphemAlgorithm``): use when you have position and attitude data directly. Call the
  static ``update(r_SN_N, r_BN_N, sigma_BN)`` method, which returns the body-frame Sun direction vector.
