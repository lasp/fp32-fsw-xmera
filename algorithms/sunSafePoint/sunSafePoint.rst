Executive Summary
-----------------

This module provides the attitude guidance output for a sun pointing mode. This could be used for safe mode, or a power
generation mode. The input is the sun direction vector which does not have to be normalized, as well as the body rate
information. The output is the standard BSK attitude reference state message. The sun direction measurement is cross
with the desired body axis that is to point at the sun to create a principle rotation vector. The dot product between
these two vectors is used to extract the principal rotation angle. With these a tracking error MRP state is computer.
The body rate tracking errors relative to the reference frame are set equal to the measured body rates to bring the
vehicle to rest when pointing at the sun.Thus, the reference angular rate and acceleration vectors relative to the
inertial frame are nominally set to zero. If the sun vector is not available, then the reference rate is set to a
body-fixed value while the attitude tracking error is set to zero.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages. The module msg connection is set by the
user from python. The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - attGuidanceOutMsg
      - :ref:`AttGuidMsgPayload`
      - attitude guidance output message
    * - sunDirectionInMsg
      - :ref:`NavAttMsgPayload`
      - sun direction input message
    * - imuInMsg
      - :ref:`NavAttMsgPayload`
      - IMU input message

Model Description
=================

.. figure:: ./_Documentation/Figures/sunHeading.pdf
   :width: 50%
   :align: center

   Body Vector Illustrations

Module Goal
-----------

This attitude guidance module has the goal of aligning a commanded
body-fixed spacecraft vector :math:`\hat{\bm s}_{c}` with another input
vector :math:`\bm s`. If :math:`\hat{\bm s}_{c}` is for example the
solar panel normal vector, and :math:`\bm s` is the current sun heading
vector, this module will compute the attitude tracking errors to align
the solar panels towards the sun, i.e. achieve sun pointing. Sun
pointing is a mode for general recharging the spacecraft, but is also a
common guidance scenario with Safe Mode.

Besides :math:`\bm s`, the second input vector is the inertial body
angular velocity vector :math:`\bm\omega_{B/N}`. The sun pointing frame
is assumed to be at rest, thus the attitude rate tracking error is set
either equal to the body rates to bring the body to rest, or difference
with a specified rotation about the sun heading vector to achieve a
final roll about this heading vector.

As the desired sun pointing orientation is inertial, the inertial
reference frame acceleration :math:`\dot{\bm\omega}_{R/N}` is set to
zero, while the reference rate is either zero or the desired sun-heading
roll vector.

Note that this module does not establish a unique sun-pointing reference
frame. Rather, the pointing condition, align :math:`\hat{\bm s}_{c}`
with :math:`\bm s` is an under-determined 2 degree of freedom condition.
Thus, the rotation angle about :math:`\bm s` is left to be arbitrary in
this sun pointing module. For the sun pointing applications this is a
very practical result as the power generation does not depend on the
orientation about :math:`\bm s`.

Equations
---------

Good Sun Direction Vector Case
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In the following mathematical developments all vectors are assumed to be
taken with respect to a body-fixed frame :math:`\cal B`. The attitude of
the body :math:`\cal B` relative to the reference frame :math:`\cal R`
is written as a principal rotation from :math:`\cal R` to
:math:`\cal B`. Thus, the associated principal rotation vector
:math:`\hat{\bm e}` is

.. math::

   \begin{equation}
   	\label{eq:ssp:1}
   	\hat{\bm e} = \frac{\bm s \times \hat{\bm s}_{c}}{|\bm s \times \hat{\bm s}_{c}|}
   \end{equation}

Note that the sun direction vector :math:`\bm s` does not have to be a
normalized input vector.

The principal rotation angle between the two vectors is given through

.. math::

   \begin{equation}
   	\label{eq:ssp:2}
   	\Phi = \arccos \left( \frac{\bm s  \cdot \hat{\bm s}_{c} }{|\bm s|} \right)
   \end{equation}

Next, this rotation from :math:`\cal R` to :math:`\cal B` is written as
a set of MRPs through

.. math::

   \begin{equation}
   	\label{eq:ssp:3}
   	\bm\sigma_{B/R} = \tan\left(\frac{\Phi}{4}\right) \hat{\bm e}
   \end{equation}

The set :math:`\bm\sigma_{B/R}` is the attitude error of the output
attitude guidance message.

The module allows for a nominal spin rate about the sun heading axis by
specifying the module parameter ``sunAxisSpinRate`` called
:math:`\dot \theta` in this description. The nominal spin rate is thus
given by

.. math::

   \begin{equation}

   	{\vphantom{\bm\omega}}^{\mathcal{B}\!}{\bm\omega}
   _{R/N} =
   	{\vphantom{\hat{\bm s}}}^{\mathcal{B}\!}{\hat{\bm s}}
    \dot\theta
   \end{equation}

Note that this constant nominal spin is only for the case where the sun
is visible and the sun-heading vector measurement is available.

If the spacecraft is to be brought to rest
:math:`\bm\omega_{R/N} = \bm 0`, then :math:`\dot\theta` should be set
to zero. The tracking error angular velocity vector is computed using.

.. math::

   \begin{equation}
   	\label{eq:ssp:4}
   	\bm\omega_{B/R} = \bm\omega_{B/N} - \bm\omega_{R/N}
   \end{equation}

Finally, the attitude guidance message must specify the inertial
reference frame acceleration vector. This is set to zero as the roll
about the sun heading is assumed to have a constant magnitude and
inertial heading.

.. math::

   \begin{equation}
   	\dot{\bm \omega}_{R/N} = \bm 0
   \end{equation}

No Sun Direction Vector Case
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If :math:`\Phi` is less then then module parameter ``minUnitMag``, then
it is assumed that no good sun heading vector is available and the
attitude tracking error :math:`\bm\sigma_{B/R}` is set to zero.

Further, if the sun is not visible, the module allows for a non-zero
body rate to be prescribed. This allows the spacecraft to engage in a
constant rate tumble specified through the module configuration vector
``omega_RN_B``. In this case the tracking error rate is evaluate through

.. math::

   \begin{equation}
    	\label{eq:ssp:6}
   	\bm\omega_{B/R} = \bm\omega_{B/N} - \bm\omega_{R/N}

   \end{equation}

and the output message reference rate is set equal to the prescribed
:math:`\bm\omega_{R/N}` while the reference acceleration vector
:math:`\dot{\bm \omega}_{R/N}` is set to zero.

Collinear Commanded and Sun Heading Vectors
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

First consider the case where :math:`\bm s \approx \hat{\bm s}_{c}`. In
this case the cross product in Eq. `[eq:ssp:1] <#eq:ssp:1>`__ is not
well defined. Let :math:`\epsilon` be a pre-determined small angle.
Then, if :math:`\Phi < \epsilon` the attitude tracking error is set to

.. math:: \bm\sigma_{B/R} = \bm 0

However, if :math:`\bm s \approx -\hat{\bm s}_{c}`, then an eigen-axis
:math:`\hat{\bm e}` that is orthogonal to :math:`\hat{\bm s}_{c}` must
be determined. Let the body frame be defined through
:math:`{\uppercase{\mathcal B}} :\{ \hat{\lowercase{\bm B}}_{1}, \hat{\lowercase{\bm B}}_{2}, \hat{\lowercase{\bm B}}_{3}\}`.
The eigen-axis is determined first by taking a cross product with
:math:`\hat{\bm b}_{1}`:

.. math::

   \begin{equation}
   	\label{eq:ssp:7}
   	\hat{\bm e}_{180} = \frac{ \hat{\bm s}_{c} \times \hat{\bm b}_{1}}{| \hat{\bm s}_{c} \times \hat{\bm b}_{1}|}
   \end{equation}

If :math:`\hat{\bm s}_{c} \approx \hat{\bm b}_{1}`, then this
:math:`\hat{\bm e}` vector will have a small norm. In this
ill-determined case, the :math:`\hat{\bm e}` vector is determined using

.. math::

   \begin{equation}
   	\label{eq:ssp:8}
   	\hat{\bm e}_{180} = \frac{ \hat{\bm s}_{c} \times \hat{\bm b}_{2}}{| \hat{\bm s}_{c} \times \hat{\bm b}_{2}|}
   \end{equation}

As :math:`\hat{\bm s}_{c}` cannot both be aligned with
:math:`\hat{\bm b}_{1}` and :math:`\hat{\bm b}_{2}`, this algorithm
determines a unique eigen axis :math:`\hat{\bm e}_{180}` for the case
that the principal rotation angle is close to 180 degrees, or
:math:`\pi - \Phi < \epsilon`. This special case eigen axis is only
computed once in the module reset routine.

In this scenario the angular velocity tracking error is evaluated using
the same method as outlined in section `1.2.1 <#sec:withSun>`__.

Module Functions
================

- **Compute the attitude tracking error**: Determines the shortest
  rotation to align :math:`\bm s` and :math:`\hat{\bm s}_{c}`, and
  computes the corresponding three-dimensional attitude difference

- **Control spacecraft rotation**: The reference frame is assumed to be
  non-accelerating and either zero (default) or a constant spin about
  the sun heading axis.

- **Robust to no sun heading information**: If the sun heading vector is
  not available, then the attitude feedback is turned off by zeroing
  :math:`\bm\sigma_{B/R}`. Instead of driving the body rates to zero,
  the body rates are driven to a prescribed :math:`\bm\omega_{R/N}`
  vector. This allow the spacecraft to search for the sun and covers the
  case if some sun sensors are offline.

- **Robust to collinear observation vector**: The module must handle the
  cases where the commanded body relative vector and the sun direction
  vectors are nearly collinear.

Module Assumptions and Limitations
==================================

The module input vector :math:`\bm s` can be a vector of any length
except for a zero-length vector. The commanded body-relative unit
direction vector :math:`{\bm s}_{c}` is assumed to be fixed relative to
the body.

Test Description and Success Criteria
=====================================

The mathematics in this module are straight forward and can be tested in
a series of input and output evaluation tests.

Check 1
-------

Here a check is performed where the sun vector measurement :math:`\bm s`
has a non-zero length and is not aligned with :math:`\hat{\bm s}_{c}`.

Check 2
-------

The sun direction vector :math:`\bm s` is given a norm value that is
less than ``minUnitMag``. In this case the attitude tracking
:math:`\bm\sigma_{B/R}` should be set to zero. Further, the body rate
errors are now evaluated relative to a fixed :math:`\bm\omega_{R/N}`
vector.

Check 3
-------

The sun direction vector :math:`\bm s` aligned with
:math:`\hat{\bm s}_{c}`. In this case the attitude tracking
:math:`\bm\sigma_{B/R}` should be set to zero. Further, the body rate
errors are simply the inertial body angular rates.

Check 4
-------

The sun direction vector :math:`\bm s \approx -\hat{\bm s}_{c}`. In this
case the attitude tracking :math:`\bm\sigma_{B/R}` should be set to
:math:`\hat{\bm e}_{180}`. Further, the body rate errors are simply the
inertial body angular rates.

.. _check-4-1:

Check 4
-------

The sun direction vector :math:`\bm s \approx -\hat{\bm s}_{c}`, but
:math:`\hat{\bm s}_{c} = \hat{\bm b}_{1}`. In this case the attitude
tracking :math:`\bm\sigma_{B/R}` should be set to
:math:`\hat{\bm e}_{180}` that is evaluated with the cross product with
:math:`\hat{\bm b}_{2}`. Further, the body rate errors are simply the
inertial body angular rates.

User Guide
==========

Input/Output Messages
---------------------

The module has 2 required input messages, and 1 output message:

- ``attGuidanceOutMsg`` – This output message, of type
  ``AttGuidMsgPayload``, provide the attitude tracking errors and the
  reference frame states.

- ``sunDirectionInMsg`` – This input message, of type
  ``NavAttMsgPayload``, receives the sun heading vector :math:`\bm s`

- ``imuInMsg`` – This input message, of type
  ``IMUSensorBodyMsgPayload``, receives the inertial angular body rates
  :math:`\bm \omega_{B/N}`

Module Parameters and States
----------------------------

The module has the following parameter that can be configured:

- ``sHatBdyCmd`` – [REQUIRED] This 3x1 array contains the commanded
  body-relative vector :math:`\hat{\bm s}_{c}` that is to be aligned
  with the sun heading :math:`\bm s`

- ``minUnitMag`` – This double contains the minimum norm value of
  :math:`\bm s` such that a tracking error attitude solution
  :math:`\bm\sigma_{B/R}` is still computed. If the norm is less than
  this, then :math:`\bm\sigma_{B/R}` is set to zero. The default
  ``minUnitMag`` value is zero.

- ``omega_RN_B`` – This vectors specifies the body-fixed search rate to
  rotate and search for the sun if no good sun direction vector is
  visible. Default value is a zero vector.

- ``smallAngle`` – This double specifies what is considered close for
  :math:`\bm s` and :math:`\hat{\bm s}_{c}` to be collinear. Default
  value is zero.

- ``sunAxisSpinRate`` – Specifies the nominal spin rate about the sun
  heading vector. This is only used if a sun heading solution is
  available. Default value is zero bring the spacecraft to rest.
