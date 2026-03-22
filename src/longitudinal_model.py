import numpy as np

def build_longitudinal_model(aircraft, derivatives):

    m = aircraft.mass
    Iy = aircraft.Iy
    g = 9.81

    # Extract derivatives
    Xu = derivatives["Xu"]
    Xw = derivatives["Xw"]
    Xq = derivatives["Xq"]

    Zu = derivatives["Zu"]
    Zw = derivatives["Zw"]
    Zq = derivatives["Zq"]

    Mu = derivatives["Mu"]
    Mw = derivatives["Mw"]
    Mq = derivatives["Mq"]

    # Build A matrix
    A_lon = np.array([
        [Xu/m, Xw/m, Xq/m, -g],
        [Zu/m, Zw/m, Zq/m, 0],
        [Mu/Iy, Mw/Iy, Mq/Iy, 0],
        [0, 0, 1, 0]
    ])

    return A_lon
