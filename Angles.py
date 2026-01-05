import math

class turret:
    def __init__(self, X=0.0, Y=0.0, Z=100.0):
        self.X = float(X)
        self.Y = float(Y)
        self.Z = float(max(1e-6, Z))
        # offsets applied before computing angles (linear offsets in same units as X/Y/Z)
        self._ox = 0.0
        self._oy = 0.0
        self._oz = 0.0
        # optional angular corrections (degrees)
        self._pan_correction = 0.0
        self._tilt_correction = 0.0
        # computed angles
        self._theta_x = 90.0
        self._theta_y = 90.0

    def offsets(self, ox=0.0, oy=0.0, oz=0.0, pan_correction=0.0, tilt_correction=0.0):
        """
        ox,oy,oz: linear offsets (same units as X,Y,Z) applied to coordinates before computing angles.
        pan_correction, tilt_correction: additive degree corrections applied after computing angles.
        """
        self._ox = float(ox)
        self._oy = float(oy)
        self._oz = float(oz)
        self._pan_correction = float(pan_correction)
        self._tilt_correction = float(tilt_correction)

    def getAngles(self):
        # apply linear offsets
        X = float(self.X + self._ox)
        Y = float(self.Y + self._oy)
        Z = float(max(1e-6, self.Z + self._oz))

        # PAN (theta_x): left-right rotation around vertical axis
        # atan2(x, z) -> angle between forward (z) and X offset
        # positive X should produce pan > 90 (to the right)
        pan_rad = math.atan2(X, Z)       # note order: (x,z)
        pan_deg = math.degrees(pan_rad)  # -180..180
        theta_pan = 90.0 + pan_deg       # center at 90

        # TILT (theta_y): rotation around horizontal axis (pitch)
        horiz = math.hypot(X, Z)
        tilt_rad = math.atan2(Y, horiz)
        tilt_deg = math.degrees(tilt_rad)
        # define convention: 90 = center (horizontal). positive Y (up) reduces tilt angle.
        theta_tilt = 90.0 - tilt_deg

        # apply simple angular corrections
        theta_pan += self._pan_correction
        theta_tilt += self._tilt_correction

        # clamp to 0..180 (servo-safe range)
        self._theta_x = max(0.0, min(180.0, theta_pan))
        self._theta_y = max(0.0, min(180.0, theta_tilt))

        return (self._theta_x, self._theta_y)

    def getTheta_x(self):
        return float(self._theta_x)

    def getTheta_y(self):
        return float(self._theta_y)