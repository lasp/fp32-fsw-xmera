Executive Summary
-----------------
This module is responsible for correcting the raw CSS output values to the expected cosine values. This requires a
pre-calibrated Chebyshev residual model which calculates the expected deviation from the expected CSS cosine output
given a raw CSS measurement at a given distance from the sun.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg connection is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - sensorListInMsg
      - :ref:`CSSArraySensorMsgPayload`
      - input message that contains CSS data
    * - cssArrayOutMsg
      - :ref:`CSSArraySensorMsgPayload`
      - output message of corrected CSS data

Model Description
=================

.. figure:: ./_Documentation/Figures/CSSCalibration.pdf
   :width: 50%
   :align: center

   Example of CSS output (blue) relative to a cosine curve (red)

This module

- Reads in raw CSS measurement from the ``CSSArraySensorMsgPayload``
  message type.

- Iterates through each raw CSS measurement, normalizing the measurement
  and checking that the input is within sensible bounds.

- Corrects the measurement based on calibrated residual function based
  on Chebyshev polynomials.

- Outputs a ``CSSArraySensorMsgPayload`` with the corrected CSS
  Measurements.

Equations
---------
Residual Function
~~~~~~~~~~~~~~~~~

The correction applied to each CSS measurement is based on a function
that maps a raw CSS measurement to the expected cosine response for that
measurement (i.e. the distance from red curve to the blue curve in
Fig. 1). This function is modeled using a Chebyshev
polynomial series to the :math:`N`-th power represented by the following
form:

.. math::

   \begin{equation}
   \delta x = \sum_{i=0}^{N} C_i*T_i(x_{\text{meas}})
   \end{equation}

where :math:`T_i(x)` represents the Chebyshev polynomials, and
:math:`C_i` are the pre-determined scaling factors.

This correction to the raw measurement is then applied using:

.. math::

   \begin{equation}
   x_{\text{corr}} = x_{\text{meas}} + \delta x
   \end{equation}

Chebyshev Polynomial Computation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The procedure to compute the Chebyshev polynomials, :math:`T_i(x)`, is
as follows:

#. Suppose we want to evaluate Chebyshev polynomial of order :math:`i`
   at :math:`x_0`, :math:`(T_i(x_0))`

#. The first two order of Chebyshev polynomials can be evaluated using
   the following form

   .. math::

      \begin{equation}
      T_0(x) = 1
      \end{equation}

   .. math::

      \begin{equation}
      T_1(x) = x
      \end{equation}

#. The Chebyshev polynomial of order :math:`i > 1` can be computed using
   the values of Chebyshev polynomials of order :math:`i-1` and
   :math:`i-2` and the following recursive formula:

   .. math::

      \begin{equation}
      T_{i+1}(x) = 2xT_i(x) - T_{i-1}(x)
      \end{equation}

#. Apply this formula up to the order :math:`i` to evaluate Chebyshev
   polynomial of order :math:`i` at :math:`x_0`.

Module Notes
============
The Chebyshev residual function is calibrated to a single distance from
the sun. As the spacecraft moves farther away from this distance, the
Chebyshev model loses accuracy. These model discrepancies grow
increasingly apparent at high sun angles.
