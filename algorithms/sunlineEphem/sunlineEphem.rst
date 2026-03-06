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

Algorithm Assumptions
---------------------
- All input position vectors must be expressed in the same inertial reference frame.
- The algorithm returns a unit direction vector for distinct positions. For colocated positions, the zero vector
  is returned as a sentinel when no direction can be computed. Downstream consumers should check for this case.

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

Test Description and Success Criteria
-------------------------------------
The Python integration test is located in
``fswAlgorithms/attDetermination/sunlineEphem/_UnitTest/test_sunlineEphem.py``. This test verifies that the
module computes the correct Sun direction vector expressed in the body frame.
In this unit test, the Sun inertial position is set at the origin by ``sunData.r_BdyZero_N = [0.0, 0.0, 0.0]``.
The spacecraft attitude is set to zero using ``vehAttData.sigma_BN = [0.0, 0.0, 0.0]``, which corresponds to an
identity rotation. This implies that the inertial and body frames are aligned.
The spacecraft's inertial position is iterated through a set of unit length vectors, with a special case where
the spacecraft and sun position vectors are identical.
For each case, the simulation will execute and the Sun direction vector will be recorded.
The truth vectors are defined in the python test, where they are compared with the estimated module output.
The test is considered successful if all the output values of the estimated and truth values are identical within
a tolerance of :math:`10^{-12}`, including outputting a zero vector for the special case where the relative
vector magnitude is zero.
