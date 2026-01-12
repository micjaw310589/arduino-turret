import numpy as np

def parallaxCorrection(_hor, _ver, _dist):
    hor, ver, dist = _hor, _ver, _dist
        
    x_off = float(0.0)
    y_off = float(119.5)
    z_off = float(-18.76)

    dist_mm = dist * 10.0
    alpha = (hor - 90) * np.pi / 180.0
    beta = (ver - 90) * np.pi / 180.0

    x_s = dist_mm * np.sin(beta) * np.cos(alpha)
    y_s = dist_mm * np.sin(beta) * np.sin(alpha)
    z_s = dist_mm * np.cos(beta)

    x_g = x_s + x_off 
    y_g = y_s + y_off
    z_g = z_s + z_off

    dist_g = np.sqrt(x_g * x_g + y_g * y_g + z_g * z_g)
    alpha_g = np.arctan2(y_g, x_g)
    beta_g = np.arccos(z_g / dist_g)

    print("hor " + str(alpha_g * 180.0 / np.pi + 90) + " ver " + str(beta_g * 180.0 / np.pi + 90))
    
    return 0

def licz(_hor, _ver, _dist):
    hor, ver, dist = (_hor) * np.pi / 180.0, (_ver) * np.pi / 180.0, _dist*10.0
    d = dist * np.sin(ver)
    dp = d * np.sin(hor)
    xp = 119.5 + dist * np.sin(ver) * np.cos(hor)
    yp = -18.75 + dist * np.cos(ver)
    ap = np.sqrt(dp * dp + yp * yp - 50.95 * 50.95)
    x = np.arctan2(yp, dp)
    y = np.arctan(ap / 50.95)
    a = np.arctan(ap / xp)
    b = np.pi - x - y
    #dist2 =
    dist2 = 0.0
    a = a * 180.0 / np.pi
    b = b * 180.0 / np.pi
    return [a, b, dist2]

parallaxCorrection(100.0, 93.0, 100.0)

print(licz(90, 90.0, 100.0))
