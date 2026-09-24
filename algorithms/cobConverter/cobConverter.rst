Executive Summary
-----------------

Module reads in a message containing the pixel data extracted from the center of brightness (COB) measurement and
transforms into a position measurement. The written message contains a heading (unit vector) and a covariance
(measurement noise).

The published heading is the center of mass (COM), estimated from the COB using a Sun phase angle correction. The
correction uses the "Binary" method, which assumes a brightness of either 1 or 0 in the image of the body, and is
applied on every cycle. The COM offset itself, along with the uncorrected COB heading, is reported on the diagnostic
message.

Optionally, Brown-Conrady distortion coefficients can be provided; the measured normalized image-plane coordinate is
then undistorted by inverting the Brown-Conrady model before the heading vector is computed.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg connection is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for:

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - opnavCOBInMsg
      - :ref:`OpNavCOBMsgPayload`
      - Input center of brightness message written out by the image processing module
    * - opnavFilterInMsg
      - :ref:`FilterMsgPayload`
      - Input filter message (used for outlier detection and COM phase-angle uncertainty)
    * - navAttInMsg
      - :ref:`NavAttMsgPayload`
      - Input navigation message containing the spacecraft attitude in the inertial frame and the Sun direction in the body frame (e.g. the aggregated output of :ref:`navAggregate`)
    * - opnavUnitVecOutMsg
      - :ref:`OpNavUnitVecMsgPayload`
      - Output COM heading vector and its covariance, inertial frame only
    * - cobConverterDiagnosticOutMsg
      - :ref:`CobConverterDiagnosticMsgPayload`
      - Output diagnostic message: the COM heading and covariance in the camera and body frames, the uncorrected COB heading, the pixel-space centers, the phase-angle correction metadata, and whether the COM outlier check was triggered

Detailed Module Description
---------------------------

Measurement mapping
^^^^^^^^^^^^^^^^^^^

This module models the measurement that will be ingested by the following filter. This is essentially a stand-alone
measurement model for the filter composed via messaging.

After reading the input center of brightness message which contains the pixel location of the center
of brightness and the number of pixels that were found, the module uses the configured camera parameters
to compute all necessary optics values.

The main relations used between all of the camera values can be found in `this paper by J. A. Christian
<https://doi.org/10.1109/ACCESS.2021.3051914>`__.

The camera calibration matrix :math:`[K]` maps normalized image-plane coordinates to pixel coordinates (Christian
Eq. 6, 16, 18, 21). For a field of view :math:`\mathrm{fov}_x, \mathrm{fov}_y` and detector resolution
:math:`\mathrm{res}_x, \mathrm{res}_y`, the normalized image-plane extents are

.. math::

    p_x = 2 \tan(\mathrm{fov}_x/2), \qquad p_y = 2 \tan(\mathrm{fov}_y/2)

giving diagonal entries :math:`K_{11} = \mathrm{res}_x / p_x` and :math:`K_{22} = \mathrm{res}_y / p_y`. Assuming the
principal point :math:`(u_p, v_p)` is at the image center, :math:`u_p = \mathrm{res}_x/2` and
:math:`v_p = \mathrm{res}_y/2`, giving

.. math::

    [K] = \left[
    \begin{array}{ccc}
    K_{11} & 0 & u_p \\
    0 & K_{22} & v_p \\
    0 & 0 & 1
    \end{array}
    \right]

:math:`d_x = K_{11}` and :math:`d_y = K_{22}` (used below) are the pixels per unit normalized image-plane coordinate
along each axis.

The COB pixel :math:`\bar{\mathbf{u}}_{COB} = [\mathrm{cob}_x, \mathrm{cob}_y, 1]^T` is shifted to the center of mass
(COM) following `this paper by S. Bhaskaran <https://doi.org/10.1109/AERO.1998.687921>`__. For a phase angle
:math:`\alpha`, the Binary offset factor is

.. math::

    \gamma = \frac{4}{3 \pi} (1 - \cos\alpha)

and the angular offset is :math:`\tan\beta = \gamma R / \rho`, with object radius :math:`R` and range :math:`\rho`. Along
the Sun direction :math:`\phi` in the image plane,

.. math::

    \mathrm{com}_x = \mathrm{cob}_x - d_x \tan\beta \cos\phi, \qquad
    \mathrm{com}_y = \mathrm{cob}_y - d_y \tan\beta \sin\phi

At :math:`\alpha = 0`, :math:`\gamma = 0` and the COM coincides with the COB. The COM pixel
:math:`\bar{\mathbf{u}}_{COM} = [\mathrm{com}_x, \mathrm{com}_y, 1]^T` is mapped to the normalized image plane,
undistorted to :math:`\mathbf{h} = [x, y, 1]^T` (see `Camera distortion calibration`_), and converted to the
COM-to-spacecraft unit vector

.. math::

    [x_d, y_d, 1]^T = [K]^{-1} \bar{\mathbf{u}}_{COM}, \qquad
    \hat{\mathbf{r}}^C = -\frac{\mathbf{h}}{|\mathbf{h}|}, \qquad
    \hat{\mathbf{r}}^N = [NC] \hat{\mathbf{r}}^C

The COB heading, computed the same way from :math:`\bar{\mathbf{u}}_{COB}`, is used only for outlier detection and the
diagnostic message.

The variance of the phase-angle offset :math:`\beta` follows from the partials of the Geometric model correction
with respect to the flyby and asteroid states:

.. math::

    \frac{\partial \beta_G}{\partial \mathbf{r}} = -\frac{4R}{3\pi r}  \: \frac{(1 - \cos \alpha)}{1 +
    \left( \frac{4R}{3\pi r} (1 - \cos \alpha) \right)^2}   \: \frac{    \mathbf{\hat{r}}  ^T}{r}



    \frac{\partial \beta_G}{\partial R} = \frac{4}{3\pi r}   \: \frac{(1 - \cos \alpha)}{1 + \left( \frac{4R}{3\pi r}
    (1 - \cos \alpha) \right)^2}



    \frac{\partial \beta_G}{\partial \alpha} = \frac{4R}{3\pi r}   \:  \frac{\sin \alpha}{1 + \left( \frac{4R}{3\pi r}
    (1 - \cos \alpha) \right)^2}


The next equation shows the partial of the phase angle with respect to the flyby and asteroid states:

.. math::

    \frac{\partial \alpha}{\partial \mathbf{r}} = \frac{ -  \mathbf{\hat{s}}  ^T}{r \sin \alpha}
    \left( \mathbf{I}  -  \mathbf{\hat{r}}     \mathbf{\hat{r}}  ^T \right)


Gathering the partials gives the variance:

.. math::

    \sigma_{\beta}^2 = \left( \frac{\partial \beta}{\partial \mathbf{r}} + \frac{\partial \beta}{\partial \alpha}
    \frac{\partial \alpha}{\partial \mathbf{r}} \right) [\mathbf{P}] \left( \frac{\partial \beta}{\partial \mathbf{r}} +
    \frac{\partial \beta}{\partial \alpha} \frac{\partial \alpha}{\partial \mathbf{r}} \right)^T +
    \left( \frac{\partial \beta}{\partial R}\right)^2 \sigma_{R}^2



where :math:`[P]` is the filter position covariance matrix and :math:`\sigma_{R}^2` is the object's radius uncertainty.

The COM heading covariance combines COB pixel noise, the phase-angle offset and the attitude error. In the normalized
image plane, with :math:`N` detected pixels,

.. math::

    P_{xy} = S \, \frac{N}{4\pi} I_2 \, S^T + \sigma_{\beta}^2 \left(1 + \tan^2\beta\right)^2 \mathbf{a}\mathbf{a}^T,
    \qquad S = \mathrm{diag}(1/d_x, 1/d_y), \quad \mathbf{a} = [\cos\phi, \sin\phi]^T

where :math:`(1 + \tan^2\beta)^2` converts :math:`\sigma_{\beta}^2` to the variance of :math:`\tan\beta`. The
distortion Jacobian is neglected. :math:`P_{xy}` is mapped to :math:`\hat{\mathbf{r}}^C` by

.. math::

    J = -\frac{1}{|\mathbf{h}|^3}
    \left[
    \begin{array}{cc}
    y^2 + 1 & -xy \\
    -xy & x^2 + 1 \\
    -x & -y
    \end{array}
    \right],
    \qquad P^C = J P_{xy} J^T

The attitude error-MRP covariance :math:`P_{att}` (body frame) is added in the inertial frame:

.. math::

    P^N = [NC] P^C [NC]^T + 16 \, [NB] [\hat{\mathbf{r}}^B \times] P_{att} [\hat{\mathbf{r}}^B \times]^T [NB]^T

where 16 converts MRP to small-angle variance (:math:`\delta\theta \approx 4\,\delta\sigma`).

The heading and covariance are also rotated to the camera and body frames for the diagnostic message. If the incoming
image is not valid, the module writes empty messages.

Camera distortion calibration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The Brown-Conrady model maps the ideal normalized coordinate :math:`(x_u, y_u)` to the measured (distorted) one
:math:`(x_d, y_d)`. With :math:`r^2 = x_u^2 + y_u^2` and :math:`L = 1 + k_1 r^2 + k_2 r^4 + k_3 r^6`,

.. math::

    x_d &= x_u L + \Delta x_t, \quad \Delta x_t = 2 p_1 x_u y_u + p_2 \left( r^2 + 2 x_u^2 \right) \\
    y_d &= y_u L + \Delta y_t, \quad \Delta y_t = 2 p_2 x_u y_u + p_1 \left( r^2 + 2 y_u^2 \right)

where :math:`k_1, k_2, k_3` are the radial and :math:`p_1, p_2` the tangential distortion coefficients. A radial
polynomial :math:`L` decreasing in :math:`r` is barrel distortion; increasing is pincushion.

The module inverts this model by fixed-point iteration from :math:`(x_u, y_u) = (x_d, y_d)`:

.. math::

    x_u \leftarrow \frac{x_d - \Delta x_t}{L}, \qquad y_u \leftarrow \frac{y_d - \Delta y_t}{L}

stopping when :math:`\max(|x_d - x_u L - \Delta x_t|, |y_d - y_u L - \Delta y_t|) \le 10^{-6} S`, with
:math:`S = \max(1, |x_d|, |y_d|, |x_u L|, |y_u L|)` finite, or after 50 iterations. Strong distortion near the edge
of the field of view may not converge; this is flagged but does not invalidate the heading. With all coefficients
zero (the default) the module is an ideal pinhole camera.

COM outlier detection
^^^^^^^^^^^^^^^^^^^^^

When ``outlierDetectionEnabled`` is true, the measured COM heading :math:`\hat{\mathbf{r}}^N` is compared with the
heading predicted by the filter position :math:`\mathbf{r}^N` (spacecraft relative to the body, from
:ref:`FilterMsgPayload`). Both point from the COM to the spacecraft:

.. math::

    \hat{\mathbf{r}}_{nav}^N = \frac{\mathbf{r}^N}{|\mathbf{r}^N|}, \qquad
    e = | \hat{\mathbf{r}}^N - \hat{\mathbf{r}}_{nav}^N | \approx \theta

where :math:`\theta` is the angle between them. Unless :math:`e < n_\sigma \sigma`, with :math:`n_\sigma` =
``numStandardDeviations``, the output unit vector is invalidated and ``comErrorOutlierTrigger`` is set.

:math:`\sigma` is the RMS of :math:`e`. If ``specifiedStandardDeviation`` is true, ``standardDeviation`` :math:`s` is the
per-axis 1-sigma of the COM pixel error [px], and :math:`\sigma = s \sqrt{1/d_x^2 + 1/d_y^2}` (pixel scale at
boresight). Otherwise, treating the measurement and filter errors as independent,

.. math::

    P_{nav}^N = \frac{(I - \hat{\mathbf{r}}_{nav}\hat{\mathbf{r}}_{nav}^T) [P] (I - \hat{\mathbf{r}}_{nav}\hat{\mathbf{r}}_{nav}^T)}
    {|\mathbf{r}^N|^2}, \qquad \sigma = \sqrt{\operatorname{tr}\left(P^N + P_{nav}^N\right)}

where :math:`[P]` is the filter position covariance and :math:`P_{nav}^N` is its linearized effect on the predicted
heading; the projection removes the range error. For an isotropic in-plane error, :math:`e` is Rayleigh distributed, and
a valid measurement is rejected with probability :math:`\exp(-n_\sigma^2)`, about :math:`10^{-4}` for
:math:`n_\sigma = 3`. The filter state is used as received, without propagation to the image time.

User Guide
----------
The module uses two-phase initialization: set the public configuration properties, connect the input messages, then
add the module to the simulation task (``reset()`` validates the configuration and builds the algorithm)::

    from xmera.fp32 import cobConverterF32 as cobConverter

    module = cobConverter.CobConverter()
    module.radius = R_obj
    module.radiusUncertainty = R_obj_uncertainty
    module.attitudeCovariance = covar_att_BN_B
    module.fieldOfViewX = fovX
    module.fieldOfViewY = fovY
    module.resolutionX = resX
    module.resolutionY = resY
    module.bodyToCameraMrp = sigma_CB
    module.cameraId = 0

    module.opnavCOBInMsg.subscribeTo(cobInMsg)
    module.opnavFilterInMsg.subscribeTo(filterInMsg)
    module.navAttInMsg.subscribeTo(attInMsg)

    sim.AddModelToTask(taskName, module)

``attitudeCovariance`` is the covariance of the attitude error MRP, in the body frame.

The outlier detection is disabled by default; enable it and configure the sigma-based gate by::

    module.outlierDetectionEnabled = True
    module.numStandardDeviations = 3  # default 3
    module.specifiedStandardDeviation = True
    module.standardDeviation = 100  # [px]; only used when specifiedStandardDeviation is True; otherwise the standard
                                     # deviation is dynamically computed by the module

The Brown-Conrady distortion coefficients are optional and default to zero. To apply a lens calibration, populate a
``CalibrationCoefficients`` struct and assign it to the module::

    coefficients = cobConverter.CalibrationCoefficients()
    coefficients.k1 = k1  # radial
    coefficients.k2 = k2
    coefficients.k3 = k3
    coefficients.p1 = p1  # tangential
    coefficients.p2 = p2
    module.calibrationCoefficients = coefficients
