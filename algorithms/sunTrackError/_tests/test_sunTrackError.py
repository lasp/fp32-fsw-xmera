#
#   Unit Test Script
#   Module Name: sunTrackError
#
#   The combined algorithm input->output behavior is covered by the C++ integrated regression
#   test (test_sunTrackError_integrated.cpp). This pytest verifies that the adapter's public
#   configuration properties round-trip through the SWIG-exposed interface.
#

import numpy as np

from xmera.fp32 import sunTrackErrorF32


def test_sunTrackError_config_roundtrip():
    """Public configuration properties round-trip through the SWIG interface."""
    module = sunTrackErrorF32.SunTrackError()
    module.modelTag = "sunTrackError"

    angleRate = 0.0123
    module.angleRate = angleRate
    np.testing.assert_allclose(module.angleRate, angleRate, rtol=1e-6, atol=1e-6)

    sensitiveHat_B = [0.1, -0.9, 0.2]
    module.sensitiveHat_B = sensitiveHat_B
    np.testing.assert_allclose(
        np.array(module.sensitiveHat_B).flatten(), sensitiveHat_B, rtol=1e-6, atol=1e-6
    )


if __name__ == "__main__":
    test_sunTrackError_config_roundtrip()
