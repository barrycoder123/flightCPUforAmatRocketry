import numpy as np

def lla_jacobian(r_ecef, HAE=True):
    """
    Computes the Jacobian of (lat, lon, alt) in [rad, rad, m (HAE)] w.r.t ECEF position in [meters]. This uses the Geodetic WGS-84 ellipsoid,
    which is the general standard defining lat/lon.

    It may be useful to allow the altitude to be w.r.t standard pressure, but that has not been added at this point

    The Jacobian is defined such that :math:`\\nabla_{ecef} f = \\nabla_{lla} f * J`

    Parameters
    ----------
    r_ecef : (3,) array-like
        ECEF position vector in meters

    HAE : bool, default=True
        if True, the altitude will be the height above the elipsoid. If False, the height will be the radius from the center of the Earth or height above
        mean sea level (the jacobian will be the same) todo:

    Returns
    -------
    J : (3, 3) ndarray
        Jacobian of (lat, lon, alt) in [rad, rad, m (HAE)] w.r.t ECEF position

    Notes
    -----
    The partials were verified by sweeping over random ECEF coordinates and taking the numerical derivative
    The phase shifts arctan2 were omitted as they didn't seem to matter

    Derived by Tyler Klein, 06/2022
    """

    x = r_ecef[0]
    y = r_ecef[1]
    z = r_ecef[2]

    # looking at eceftolla, we can calculate the partials
    grad_lon = np.array([[-y, x, 0]]) / (x * x + y * y)  # gradient of longitude w.r.t (x,y,z)

    b = np.sqrt((a ** 2) * (1 - e ** 2))
    ep = np.sqrt((a ** 2 - b ** 2) / b ** 2)
    p = np.sqrt(x ** 2 + y ** 2)
    grad_p = np.array([x, y, 0]) / p

    th = np.arctan2(a * z, b * p)
    dth_dp_coeff = - a * z / (b * p * p)
    grad_th = np.array([[dth_dp_coeff * grad_p[0], dth_dp_coeff * grad_p[1], a / (b * p)]]) / (1.0 + ((a * z) / (b * p)) ** 2)

    # the latitude is a PAIN
    num = z + ep ** 2 * b * np.sin(th) ** 3
    grad_num = np.array([0, 0, 1.0]) + 3 * ep ** 2 * b * np.sin(th) ** 2 * np.cos(th) * grad_th
    den = p - e ** 2 * a * np.cos(th) ** 3
    grad_den = grad_p + 3 * e ** 2 * a * np.cos(th) ** 2 * np.sin(th) * grad_th
    lat = np.arctan2(num, den)
    grad_lat = ((1.0 / den) * grad_num - (num / den ** 2) * grad_den) * 1.0 / (1.0 + (num / den) ** 2)

    if HAE:
        grad_N = grad_lat * a * (e ** 2 * np.sin(lat) * np.cos(lat)) / (1 - (e ** 2) * np.sin(lat) ** 2) ** (3 / 2)
        grad_alt = grad_p / np.cos(lat) + grad_lat * p * np.sin(lat) / np.cos(lat) ** 2 - grad_N
    else:
        raise NotImplementedError('Need to subtract out ellipsoid height')

    J = np.vstack((grad_lat, grad_lon, grad_alt))
    return J