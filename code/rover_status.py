import numpy as np
import time


ROVER_STABLE_ANGLE_RANGE = 5


class RoverStatus(object):

    x_pos = [0.0, 0.0, 0.0]
    y_pos = [0.0, 0.0, 0.0]
    yaw = [0.0, 0.0, 0.0]
    last_update_time = [-1.0]
    last_snapshot_yaw = [1.0]
    last_snapshot_x_pos = [0.0]
    last_snapshot_y_pos = [0.0]

    def __init__(self, rover):
        self.rover = rover

    def reset_status(self):
        self.x_pos[0] = 0.0
        self.x_pos[1] = 0.0
        self.x_pos[2] = 0.0
        self.y_pos[0] = 0.0
        self.y_pos[1] = 0.0
        self.y_pos[2] = 0.0
        self.yaw[0] = 0.0
        self.yaw[1] = 0.0
        self.yaw[2] = 0.0
        self.last_update_time[0] = -1.0
        self.last_snapshot_yaw[0] = -1.0
        self.last_snapshot_x_pos[0] = 0.0
        self.last_snapshot_y_pos[0] = 0.0

    def snapshot(self):
        self.last_snapshot_yaw[0] = self.rover.yaw
        self.last_snapshot_x_pos[0] = self.rover.pos[0]
        self.last_snapshot_y_pos[0] = self.rover.pos[1]

    def is_unstable(self):
        if ROVER_STABLE_ANGLE_RANGE < abs(self.rover.pitch) < (360 - ROVER_STABLE_ANGLE_RANGE):
            return True
        elif ROVER_STABLE_ANGLE_RANGE < abs(self.rover.roll) < (360 - ROVER_STABLE_ANGLE_RANGE):
            return True
        else:
            return False

    def has_vision(self):
        if self.rover.nav_angles is not None:
            return True
        else:
            return False

    def has_vision_for_navigate_during_forward(self):
        if len(self.rover.nav_angles) >= self.rover.stop_forward:
            return True
        else:
            return False

    def has_vision_for_start_moving(self):
        if len(self.rover.nav_angles) >= self.rover.go_forward:
            return True
        else:
            return False

    def get_navigation_image(self):
        return self.rover.vision_image[:, :, 2]

    def get_rock_image(self):
        return self.rover.vision_image[:, :, 1]

    def is_not_moving(self):
        __pos_not_change = (np.max(np.abs(self.x_pos - np.mean(self.x_pos))) < 0.1
                            and np.max(np.abs(self.y_pos - np.mean(self.y_pos))) < 0.1)
        __yaw_not_change = np.max(np.abs(self.yaw - np.mean(self.yaw))) < 0.1
        if __pos_not_change and __yaw_not_change:
            return True
        else:
            return False

    def is_moving(self):
        if self.rover.vel > 0.2:
            return True
        else:
            return False

    def update_status(self):
        __time = time.time()
        if (__time - self.last_update_time[0]) > 1.0:
            self.x_pos[2] = self.x_pos[1]
            self.x_pos[1] = self.x_pos[0]
            self.x_pos[0] = self.rover.pos[0]
            self.y_pos[2] = self.y_pos[1]
            self.y_pos[1] = self.y_pos[0]
            self.y_pos[0] = self.rover.pos[1]
            self.yaw[2] = self.yaw[1]
            self.yaw[1] = self.yaw[0]
            self.yaw[0] = self.rover.yaw
            self.last_update_time[0] = __time

    def get_yaw_diff_since_snapshot(self):
        return abs(self.last_snapshot_yaw[0] - self.rover.yaw)
