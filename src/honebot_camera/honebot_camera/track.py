"""Single-target tracking on top of the per-frame person detector (pure, unit-tested, no ROS/cv2).

Why this exists: a detector only answers "where are people in *this* frame". On its own that gives two
failure modes the camera suffers from: a one-frame false positive becomes the target instantly, and a
missed detection (odd pose mid-trick) drops the target so the next frame picks whoever is biggest.
Real people persist across frames and move smoothly; phantoms flicker. So:

  * Candidates must first pass cheap plausibility checks (`plausible`): roughly person-shaped, not a
    speck, not floating at the top of the image, and, when UWB says how far the skater is, roughly the
    size a person at that distance would be.
  * A track needs `confirm_hits` consecutive cycles of agreeing detections before it is trusted.
  * A trusted track predicts where the skater will be (constant-velocity Kalman filter on bearing) and
    associates the nearest detection inside a gate around that prediction. The gate grows with the
    prediction uncertainty, so after a few misses it is wider, but never beyond `gate_max`.
  * Missed cycles are coasted for up to `coast_time`; then the track is dropped and acquisition restarts.

Angles are in the ROBOT frame (radians, +left): robot bearing = commanded pan + image bearing. Tracking in
the robot frame means the camera panning does not look like the skater moving. The caller (camera_node)
does the pixel <-> bearing conversions with aim.py and knows the current pan.
"""

import math
from dataclasses import dataclass
from typing import Optional, Sequence

from honebot_camera.detector import Detection


# --- plausibility ------------------------------------------------------------------------------------

@dataclass
class PlausibilityParams:
    min_aspect: float = 0.6          # box height / width. Standing ~2-4, crouching ~1, mid-trick can be < 1.
    max_aspect: float = 6.0          # taller than this is a pole, a door edge, a tree trunk
    min_height_frac: float = 0.05    # of frame height; smaller is a speck (or too far to matter)
    min_bottom_y_norm: float = -0.3  # bottom edge must be below this (-1 top .. +1 bottom): nothing stands
                                     # in the top third of a roughly level camera; a jump is still fine
    person_height_m: float = 1.7
    size_tol_low: float = 0.4        # accept box height within [low, high] x expected height at the UWB
    size_tol_high: float = 2.5       # distance (crouching halves it, box padding enlarges it)


def expected_height_frac(distance_m: float, hfov: float, width: int, height: int,
                         person_height_m: float = 1.7) -> float:
    """Height of a person at `distance_m`, as a fraction of frame height (pinhole, level camera)."""
    if distance_m <= 0.0:
        return math.inf
    f_px = (width / 2.0) / math.tan(hfov / 2.0)
    return f_px * person_height_m / distance_m / height


def plausible(d: Detection, width: int, height: int, p: PlausibilityParams,
              expected_h_frac: Optional[float] = None) -> bool:
    """Could this box be the skater at all? `expected_h_frac` from expected_height_frac() when UWB is fresh."""
    if d.width <= 0.0 or d.height <= 0.0:
        return False
    aspect = d.height / d.width
    if aspect < p.min_aspect or aspect > p.max_aspect:
        return False
    h_frac = d.height / height
    if h_frac < p.min_height_frac:
        return False
    if (2.0 * d.y2 / height - 1.0) < p.min_bottom_y_norm:
        return False
    if expected_h_frac is not None and math.isfinite(expected_h_frac) and expected_h_frac > 0.0:
        ratio = h_frac / expected_h_frac
        if ratio < p.size_tol_low or ratio > p.size_tol_high:
            return False
    return True


# --- Kalman filter on (bearing, bearing rate) ---------------------------------------------------------

class BearingKF:
    """Constant-velocity Kalman filter for one angle. State x = [bearing, rate]; 2x2 covariance P."""

    def __init__(self, bearing: float, meas_std: float, accel_std: float, init_rate_std: float = 1.0):
        self.b = bearing
        self.v = 0.0
        r = max(1e-12, meas_std ** 2)
        self.p00 = r
        self.p01 = 0.0
        self.p11 = init_rate_std ** 2
        self.r = r
        self.q = max(0.0, accel_std ** 2)

    def predict(self, dt: float):
        dt = max(0.0, dt)
        self.b += self.v * dt
        # P = F P F^T + Q, F = [[1, dt], [0, 1]], Q = q * [[dt^4/4, dt^3/2], [dt^3/2, dt^2]]
        p00 = self.p00 + 2.0 * dt * self.p01 + dt * dt * self.p11 + self.q * dt ** 4 / 4.0
        p01 = self.p01 + dt * self.p11 + self.q * dt ** 3 / 2.0
        p11 = self.p11 + self.q * dt * dt
        self.p00, self.p01, self.p11 = p00, p01, p11

    def innovation_std(self) -> float:
        return math.sqrt(self.p00 + self.r)

    def update(self, z: float):
        s = self.p00 + self.r
        if s <= 0.0:
            return
        k0 = self.p00 / s
        k1 = self.p01 / s
        y = z - self.b
        self.b += k0 * y
        self.v += k1 * y
        # P = (I - K H) P
        p00 = (1.0 - k0) * self.p00
        p01 = (1.0 - k0) * self.p01
        p11 = self.p11 - k1 * self.p01
        self.p00, self.p01, self.p11 = p00, p01, p11


# --- tracker ---------------------------------------------------------------------------------------------

@dataclass
class TrackParams:
    confirm_hits: int = 3            # consecutive agreeing detections before a candidate is trusted
    coast_time: float = 1.0          # s of misses a confirmed track survives
    gate_min: float = math.radians(6.0)    # association gate is max(gate_min, n_sigma * innovation std)
    gate_max: float = math.radians(25.0)   # ... but never wider than this
    n_sigma: float = 3.0
    meas_std: float = math.radians(1.0)    # box-centre jitter of a steady person
    accel_std: float = math.radians(90.0)  # how quickly the bearing rate may change (rad/s^2)
    hint_gate: float = math.radians(15.0)  # seed from the detection nearest the UWB bearing if within this


@dataclass
class Candidate:
    bearing: float                   # robot frame, radians
    det: Detection


@dataclass
class TrackState:
    bearing: float                   # filtered estimate, robot frame
    rate: float                      # rad/s
    confirmed: bool
    matched: bool                    # a detection was associated this cycle
    det: Optional[Detection]         # the associated detection (None while coasting)
    hits: int                        # consecutive hits (tentative) / total hits (confirmed)
    coast: float                     # seconds since the last match (0 when matched)


class TargetTracker:
    """Feed it every detector cycle (also cycles with no detections). Returns a TrackState while there is a
    tentative or confirmed track, else None. Only `confirmed` states should be acted on."""

    def __init__(self, params: Optional[TrackParams] = None):
        self.p = params or TrackParams()
        self.kf: Optional[BearingKF] = None
        self.confirmed = False
        self.hits = 0
        self.last_match_t: Optional[float] = None
        self.last_t: Optional[float] = None

    @property
    def active(self) -> bool:
        return self.kf is not None

    def reset(self):
        self.kf = None
        self.confirmed = False
        self.hits = 0
        self.last_match_t = None

    def _sync_kf_noise(self):
        if self.kf is not None:
            self.kf.r = max(1e-12, self.p.meas_std ** 2)
            self.kf.q = max(0.0, self.p.accel_std ** 2)

    def gate(self) -> float:
        self._sync_kf_noise()
        g = self.p.n_sigma * self.kf.innovation_std()
        return min(self.p.gate_max, max(self.p.gate_min, g))

    def predicted_bearing(self) -> Optional[float]:
        return self.kf.b if self.kf is not None else None

    def _seed(self, t: float, cands: Sequence[Candidate], hint: Optional[float]) -> TrackState:
        if hint is not None:
            best = min(cands, key=lambda c: abs(c.bearing - hint))
            if abs(best.bearing - hint) > self.p.hint_gate:
                best = max(cands, key=lambda c: c.det.area)
        else:
            best = max(cands, key=lambda c: c.det.area)
        self.kf = BearingKF(best.bearing, self.p.meas_std, self.p.accel_std)
        self.confirmed = self.p.confirm_hits <= 1
        self.hits = 1
        self.last_match_t = t
        return self._state(True, best.det, 0.0)

    def _state(self, matched: bool, det: Optional[Detection], coast: float) -> TrackState:
        return TrackState(self.kf.b, self.kf.v, self.confirmed, matched, det, self.hits, coast)

    def update(self, t: float, cands: Sequence[Candidate], hint: Optional[float] = None) -> Optional[TrackState]:
        """One detector cycle at time t with the plausible candidates (may be empty) and an optional UWB
        bearing hint (robot frame) used only to choose which candidate to start a new track on."""
        dt = 0.0 if self.last_t is None else t - self.last_t
        self.last_t = t

        if self.kf is not None:
            # Drop stale tracks before predicting/associating (e.g. after a multi-second video freeze).
            if self.confirmed and (t - self.last_match_t) > self.p.coast_time:
                self.reset()
            elif not self.confirmed and dt > self.p.coast_time:
                self.reset()

        if self.kf is None:
            if not cands:
                return None
            return self._seed(t, cands, hint)

        self._sync_kf_noise()
        self.kf.predict(dt)
        gate = self.gate()
        best = None
        if cands:
            best = min(cands, key=lambda c: abs(c.bearing - self.kf.b))
            if abs(best.bearing - self.kf.b) > gate:
                best = None

        if best is not None:
            self.kf.update(best.bearing)
            self.hits += 1
            self.last_match_t = t
            if not self.confirmed and self.hits >= self.p.confirm_hits:
                self.confirmed = True
            return self._state(True, best.det, 0.0)

        # Miss.
        coast = t - self.last_match_t
        if not self.confirmed:
            # A tentative track must be seen on consecutive cycles; one miss and we start over (possibly on
            # one of this cycle's candidates).
            self.reset()
            return self._seed(t, cands, hint) if cands else None
        if coast > self.p.coast_time:
            self.reset()
            return self._seed(t, cands, hint) if cands else None
        return self._state(False, None, coast)


def tracked_image_bearing(state: TrackState, pan: float) -> float:
    """Where the tracked bearing sits relative to the camera axis right now (robot -> camera frame)."""
    return state.bearing - pan

