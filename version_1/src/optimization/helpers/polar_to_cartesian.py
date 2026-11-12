import aerosandbox.numpy as np


def polar_to_cartesian(r, theta):
    """Convert a radius and angle in the y-z plane to (y, z) coordinates."""
    return r * np.cos(theta), r * np.sin(theta)
