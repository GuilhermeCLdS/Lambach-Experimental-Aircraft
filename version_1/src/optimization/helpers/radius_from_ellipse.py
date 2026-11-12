import aerosandbox.numpy as np


def radius_from_ellipse(x, a, b):
    """Elliptical fuselage radius at longitudinal coordinate x.

    x is measured from the fuselage centre; a and b are the semi-length and
    semi-diameter. Returns 0 outside the body.
    """
    return b * np.sqrt(np.maximum(1 - (x / a) ** 2, 0.0))
