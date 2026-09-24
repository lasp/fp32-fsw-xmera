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
      - Output COM heading vector and covariance, inertial frame only. The COM is always computed; with no phase-angle correction it degenerates to the COB
    * - cobConverterDiagnosticOutMsg
      - :ref:`CobConverterDiagnosticMsgPayload`
      - Output diagnostic message: the COM heading and covariance in the camera and body frames, the uncorrected COB heading, the pixel-space centers, the phase-angle correction metadata, and whether the COB outlier check was triggered

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

:math:`d_x = 1/K_{11}` and :math:`d_y = 1/K_{22}` (used below) are the reciprocals of these diagonal entries -- the
normalized image-plane extent of one pixel along each axis.

The average angular field of view per pixel (iFOV) used elsewhere in this derivation is
:math:`\psi_{i,x} = \mathrm{fov}_x / \mathrm{res}_x` and :math:`\psi_{i,y} = \mathrm{fov}_y / \mathrm{res}_y` -- an
average-scale approximation, since Christian's exact tangent projection has a slightly position-dependent local
angular scale.

With this, the unit vector in the camera frame from focal point to center of brightness in the image plane is found by
(equivalent to setting z=1):

.. math::

    \mathbf{r}_{COB}^C &= [K]^{-1} \mathbf{\bar{u}}_{COB}

where :math:`\mathbf{\bar{u}}_{COB} = [\mathrm{cob}_x, \mathrm{cob}_y, 1]^T` with the pixel coordinates of the center of
brightness :math:`\mathrm{cob}_x` and :math:`\mathrm{cob}_y`, :math:`[K]` is the camera calibration matrix and
:math:`\mathbf{r}_{COB}^C` is the unit vector describing the physical heading to the target in the camera frame.
With distortion coefficients set, :math:`[K]^{-1} \mathbf{\bar{u}}` is first undistorted (see `Camera distortion calibration`_).

The covariance of the COB error is found using the number of detected pixels and the camera parameters, given by:

.. math::

    P = \frac{\mathrm{numPixels}}{4 \pi \cdot \left\| \mathbf{\bar{u}}_{COB} \right\|^2}
    \left(
    \left[
    \begin{array}{ccc}
    d_x^2 & 0 & 0 \\
    0 & d_y^2 & 0 \\
    0 & 0 & 1
    \end{array}
    \right]
    \right)


where :math:`d_x` and :math:`d_y` (defined above) are the normalized image-plane extent of one pixel along each axis.
This covariance matrix is then transformed into the body frame and added to the covariance of the attitude error.


The offset factor :math:`\gamma` due to the Sun phase angle correction is obtained for a phase angle
:math:`\alpha` using

.. math::

    \gamma = \frac{4}{3 \pi} (1 - \cos\alpha)

for the Binary method. Note that :math:`\gamma = 0` when :math:`\alpha = 0`, in which case the COM coincides with
the COB. The correction for the COM
location is performed according to `this paper by S. Bhaskaran <https://doi.org/10.1109/AERO.1998.687921>`__. First, the
object radius :math:`R` in meters is converted to the object radius in pixel units :math:`R_c` by

.. math::

    R_c = \frac{R K_x f}{\rho} = \frac{R d_x}{\rho}

where :math:`K_x = d_x/f`, :math:`f` is the focal length in meters, and :math:`\rho` is the distance from the
body center to the spacecraft in meters. Using the sun direction in the image plane :math:`\phi`, the COM location in
pixel space is then computed using

.. math::

    \mathrm{com}_x = \mathrm{cob}_x - \gamma \frac{R d_x}{\rho} \cos\phi \\
    \mathrm{com}_y = \mathrm{cob}_y - \gamma \frac{R d_y}{\rho} \sin\phi

where :math:`d_x` and :math:`d_y` scale the angular offset :math:`\gamma R / \rho` into pixels per axis.

Finally, similar to the COB unit vector, the COM unit vector is obtained by

.. math::

    \mathbf{r}_{COM}^C &= [K]^{-1} \mathbf{\bar{u}}_{COM}

where :math:`\mathbf{\bar{u}}_{COM} = [\mathrm{com}_x, \mathrm{com}_y, 1]^T`, undistorted as for the COB.


The covariance of the COM error is found by firstly computing the total derivative of the angular error. Which can be
found by calculating the partials of the Geometric model correction with respect to the flyby and asteroid states:

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


This leads to standard deviation equation which is done by gathering the partials in the following equation:

.. math::

    \sigma_{\beta}^2 = \left( \frac{\partial \beta}{\partial \mathbf{r}} + \frac{\partial \beta}{\partial \alpha}
    \frac{\partial \alpha}{\partial \mathbf{r}} \right) [\mathbf{P}] \left( \frac{\partial \beta}{\partial \mathbf{r}} +
    \frac{\partial \beta}{\partial \alpha} \frac{\partial \alpha}{\partial \mathbf{r}} \right)^T +
    \left( \frac{\partial \beta}{\partial R}\right)^2 \sigma_{R}^2



where :math:`[P]` is the filter position covariance matrix and :math:`\sigma_{R}^2` is the object's radius uncertainty.

.. warning::

    The equation below (historical / incorrect) was the original formula used to find the COM covariance matrix.
    It applies :math:`\cos \phi` and :math:`\sin \phi` to the first power directly on the diagonal, which is not a
    valid variance decomposition: since :math:`\phi` ranges over the full :math:`(-\pi, \pi]`, :math:`\cos \phi` (or
    :math:`\sin \phi`) is negative across roughly half that range, which -- given
    :math:`\sigma_{\beta}^2 / \psi_i^2` is routinely many orders of magnitude larger than :math:`d_x^2`/:math:`d_y^2`
    -- would drive these diagonal entries negative, producing a matrix that is not a valid (PSD) covariance. It is
    kept here only for historical reference; it is superseded by the corrected equation further below.

.. math::

    W_{\text{correction, old/incorrect}} = \left(
    \left[
    \begin{array}{ccc}
    d_x^2 + \dfrac{\sigma_{\beta}^2}{\psi_{i,x}} \cos \phi & 0 & 0 \\
    0 & d_y^2 + \dfrac{\sigma_{\beta}^2}{\psi_{i,y}} \sin \phi & 0 \\
    0 & 0 & 1
    \end{array}
    \right]
    \right)

The corrected equation instead treats :math:`\sigma_{\beta}^2` as the variance of a single scalar magnitude directed
along the sun-line :math:`(\cos \phi, \sin \phi)` in the image plane (zero variance perpendicular to it). Rotating
that 1-D angular variance into the image :math:`x`/:math:`y` axes is a similarity transform by the rotation matrix

.. math::

    R(\phi) = \left[
    \begin{array}{cc}
    \cos \phi & -\sin \phi \\
    \sin \phi & \cos \phi
    \end{array}
    \right]

giving:

.. math::

    R(\phi) \: \mathrm{diag}(\sigma_{\beta}^2, 0) \: R(\phi)^T = \sigma_{\beta}^2
    \left[
    \begin{array}{cc}
    \cos^2 \phi & \cos \phi \sin \phi \\
    \cos \phi \sin \phi & \sin^2 \phi
    \end{array}
    \right]

which is positive semi-definite for any :math:`\phi`, since it is a similarity transform of a non-negative diagonal
matrix. Converting from angular units (:math:`\mathrm{rad}^2`) to normalized image-plane units takes two separate
conversions that are kept distinct elsewhere in this derivation: angle to pixels via the iFOV
:math:`\psi_{i,x}, \psi_{i,y}` (an average-scale approximation), then pixels to normalized image-plane coordinates
via :math:`d_x, d_y` (the exact, tangent-based per-axis pixel scale used for the baseline term above). These two
conversions are not interchangeable in general -- :math:`d_x / \psi_{i,x}` deviates from 1 by roughly 26% at the wide
end of the supported field-of-view range -- so both steps are applied via the diagonal congruence transform
:math:`D (\cdot) D` with :math:`D = \mathrm{diag}(d_x/\psi_{i,x}, d_y/\psi_{i,y})`, which likewise preserves
positive semi-definiteness. Adding the baseline COB pixel-noise diagonal keeps the sum positive semi-definite, since
the sum of two positive semi-definite matrices is positive semi-definite:

.. math::

    W_{\text{correction}} = \left(
    \left[
    \begin{array}{ccc}
    d_x^2 + \sigma_{\beta}^2 \left(\dfrac{d_x}{\psi_{i,x}}\right)^2 \cos^2 \phi &
    \sigma_{\beta}^2 \dfrac{d_x d_y}{\psi_{i,x} \psi_{i,y}} \cos \phi \sin \phi & 0 \\
    \sigma_{\beta}^2 \dfrac{d_x d_y}{\psi_{i,x} \psi_{i,y}} \cos \phi \sin \phi &
    d_y^2 + \sigma_{\beta}^2 \left(\dfrac{d_y}{\psi_{i,y}}\right)^2 \sin^2 \phi & 0 \\
    0 & 0 & 1
    \end{array}
    \right]
    \right)


where :math:`\psi_{i,x}` and :math:`\psi_{i,y}` are the iFOV. This matrix is then transformed into the body frame and
added to the covariance of the attitude error and COB error. All individual covariance matrices, and thus the total
covariance matrix, describe the measurement noise of a unit vector.

By reading the camera orientation and the current body attitude in the inertial frame, the final step is to rotate
the covariance and heading vector in all the relevant frames for modules downstream. This is done simply by
converting MRPs to DCMs and performing the matrix multiplication.
If the incoming image is not valid, the module writes empty messages.

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

An outlier detection may be performed for the COB. In this case, the filter message :ref:`FilterMsgPayload` is used to
predict the location of the COB. If the location of the COB coming from the image is significantly different from the
predicted COB, it is considered an outlier and the output unit vector is invalidated. For the output message to be
valid, the following condition must be fulfilled:

.. math::

    e_{COB} = | \mathbf{u}_{COB} - \mathbf{u}_{COB, predicted} | \le n_\sigma \cdot \sigma

where :math:`\mathbf{u}_{COB} = [\mathrm{cob}_x, \mathrm{cob}_y]^T` are the x-y coordinates of the COB coming from the
image, :math:`\mathbf{u}_{COB, predicted}` are the x-y coordinates of the predicted COB, :math:`n_\sigma` are the number
of standard deviations specified by the module input "numStandardDeviations". The standard deviation :math:`\sigma` is
either fixed via the ``standardDeviation`` property (when ``specifiedStandardDeviation`` is true), or automatically
obtained by the module using the attitude covariance matrix (specified as parameter of the module) as well as the
covariance of the COB estimation and the filter covariance (both of which are automatically computed).

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

The COB outlier detection is disabled by default; enable it and configure the sigma-based gate by::

    module.outlierDetectionEnabled = True
    module.numStandardDeviations = 3  # default 3
    module.specifiedStandardDeviation = True
    module.standardDeviation = 100  # only used when specifiedStandardDeviation is True; otherwise the standard
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
