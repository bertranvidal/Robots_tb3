import numpy as np
from enum import Enum, auto


class WallFollowerState(Enum):
    STRAIGHT = auto()
    TURN_LEFT = auto()
    TURN_RIGHT = auto()
    TURN_AROUND = auto()


class WallFollower:
    LINEAR_SPEED_MAX = 0.22
    SENSOR_RANGE_MIN = 0.16
    SENSOR_RANGE_MAX = 8.0
    TRACK = 0.16
    WHEEL_RADIUS = 0.033
    WHEEL_SPEED_MAX = LINEAR_SPEED_MAX / WHEEL_RADIUS

    def __init__(self, dt: float, logger=None, simulation: bool = False) -> None:
        self._dt = dt
        self._logger = logger
        self._simulation = simulation

        self.state = WallFollowerState.STRAIGHT

        self.kp = 0.58
        self.kd = 0.085
        self.last_error = 0.0

        self.front_thresh = 0.36
        self.clear_thresh = 0.54
        self.side_thresh = 0.41

        self.confirm_hits = 3
        self.front_counter = 0
        self.clear_counter = 0

        self.diag_limit = 0.31

        self.max_w = 1.48

        self.turn_speed = 0.055
        self.slow_speed = 0.085

        #self.startup_counter = 0


    def compute_commands(
        self,
        z_scan: list[float],
        z_v: float,
        z_w: float
    ) -> tuple[float, float]:


        # if self.startup_counter < 10:
        #     self.startup_counter += 1
        #     return 0.1, 0.0


        n = len(z_scan)
        if n == 0:
            return 0.0, 0.0

        scan = np.array(z_scan, dtype=np.float32)
        scan = np.clip(scan, self.SENSOR_RANGE_MIN, self.SENSOR_RANGE_MAX)

        front_dist = np.nanmean(np.concatenate((scan[:10], scan[-10:])))
        left_dist = np.nanmean(scan[15:60])
        right_dist = np.nanmean(scan[180:225])

        k = max(1, n // 12)
        diag_left = np.nanmean(scan[k - 3:k + 4])
        diag_right = np.nanmean(scan[-k - 3:-k + 4])

        if front_dist < self.front_thresh:
            self.front_counter += 1
        else:
            self.front_counter = 0

        v = self.LINEAR_SPEED_MAX
        w = 0.0

        if self.state == WallFollowerState.STRAIGHT:

            if diag_right < self.diag_limit:
                return self.slow_speed, +0.57
            if diag_left < self.diag_limit:
                return self.slow_speed, -0.57

            if (
                self.front_counter >= self.confirm_hits
                and left_dist < self.side_thresh
                and right_dist < self.side_thresh
            ):
                self.state = WallFollowerState.TURN_AROUND
                self.clear_counter = 0

            elif self.front_counter >= self.confirm_hits:
                self.state = (
                    WallFollowerState.TURN_LEFT
                    if left_dist > right_dist
                    else WallFollowerState.TURN_RIGHT
                )
                self.clear_counter = 0

            else:
                desired = 0.30   # prueba 0.28–0.32
                error = desired - left_dist

                if abs(error) < 0.045:
                    w = 0.0
                    self.last_error = 0.0
                else:
                    deriv = (error - self.last_error) / self._dt
                    w = self.kp * error + self.kd * deriv
                    self.last_error = error

                if abs(w) > 0.62:
                    v = self.slow_speed

        elif self.state == WallFollowerState.TURN_LEFT:
            v = self.turn_speed
            w = +self.max_w

            if front_dist > self.clear_thresh:
                self.clear_counter += 1
            else:
                self.clear_counter = 0

            if self.clear_counter >= self.confirm_hits:
                self.state = WallFollowerState.STRAIGHT
                self.clear_counter = 0
                self.front_counter = 0
                self.last_error = 0.0

        elif self.state == WallFollowerState.TURN_RIGHT:
            v = self.turn_speed
            w = -self.max_w

            if front_dist > self.clear_thresh:
                self.clear_counter += 1
            else:
                self.clear_counter = 0

            if self.clear_counter >= self.confirm_hits:
                self.state = WallFollowerState.STRAIGHT
                self.clear_counter = 0
                self.front_counter = 0
                self.last_error = 0.0

        elif self.state == WallFollowerState.TURN_AROUND:
            v = self.turn_speed
            w = +self.max_w

            if front_dist > self.clear_thresh:
                self.clear_counter += 1
            else:
                self.clear_counter = 0

            if self.clear_counter >= self.confirm_hits:
                self.state = WallFollowerState.STRAIGHT
                self.clear_counter = 0
                self.front_counter = 0
                self.last_error = 0.0

        w = max(min(w, self.max_w), -self.max_w)
        v = min(v, self.LINEAR_SPEED_MAX)

        return v, w
