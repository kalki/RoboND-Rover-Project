import numpy as np
import time
from queue import Queue

ROVER_STABLE_ANGLE_RANGE = 5
OLD_WAY_THRESHOLD = 30
OLD_WAY_COOLDDOWN = 60


class RoverStatus(object):

    x_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    y_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    yaw = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    last_update_time = [-1.0]
    current_mode_time = [-1.0]
    last_snapshot_yaw = [1.0]
    last_snapshot_x_pos = [0.0]
    last_snapshot_y_pos = [0.0]
    action_buffer = Queue()
    old_action_history = set()
    last_old_way_time = [-1.0]

    def __init__(self, rover):
        self.rover = rover

    def reset_status(self):
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

    def get_obstacles_map(self):
        return self.rover.worldmap[:, :, 0]

    def get_navigation_map(self):
        return self.rover.worldmap[:, :, 2]

    def get_obstacles_image(self):
        return self.rover.vision_image[:, :, 0]

    def get_navigation_image(self):
        return self.rover.vision_image[:, :, 2]

    def get_rock_image(self):
        return self.rover.vision_image[:, :, 1]

    def is_not_moving(self):
        __pos_not_change = (np.max(np.abs(self.x_pos - np.mean(self.x_pos))) < 0.2
                            and np.max(np.abs(self.y_pos - np.mean(self.y_pos))) < 0.2)
        __yaw_not_change = np.max(np.abs(self.yaw - np.mean(self.yaw))) < 2
        if __pos_not_change and __yaw_not_change:
            return True
        else:
            return False

    def is_moving(self):
        if self.rover.vel > 0.2:
            return True
        else:
            return False

    def stop_recording_position(self):
        self.x_pos.pop(0)
        self.x_pos.append(int(time.time()) % 255 + 200)
        self.y_pos.pop(0)
        self.y_pos.append(int(time.time()) % 255 + 200)
        self.yaw.pop(0)
        self.yaw.append(int(time.time()) % 360)

    def update_status(self):
        __time = time.time()
        if (__time - self.last_update_time[0]) > 1.0:
            __x = self.rover.pos[0]
            __y = self.rover.pos[0]
            __yaw = self.rover.yaw
            self.x_pos.pop(0)
            self.x_pos.append(__x)
            self.y_pos.pop(0)
            self.y_pos.append(__y)
            self.yaw.pop(0)
            self.yaw.append(__yaw)
            self.last_update_time[0] = __time

            __x = int(__x / 2) * 3
            __y = int(__y / 2) * 3
            __yaw = int(__yaw / 15) * 15
            __current_action = (__x, __y, __yaw)
            self.action_buffer.put(__current_action)
            if self.action_buffer.qsize() > OLD_WAY_COOLDDOWN:
                __action = self.action_buffer.get()
                self.old_action_history.add(__action)

            if __current_action in self.old_action_history:
                if self.last_old_way_time[0] < 0:
                    self.last_old_way_time[0] = time.time()
                    print("OLD WAY FOUND")
            else:
                if self.last_old_way_time[0] > 0:
                    self.last_old_way_time[0] = -1.0
                    print("OLD WAY CLEAR")

    def get_yaw_diff_since_snapshot(self):
        return abs(self.last_snapshot_yaw[0] - self.rover.yaw)

    def is_on_old_way_too_long(self):
        if self.last_old_way_time[0] > 0:
            s = time.time() - self.last_old_way_time[0]
            if s > OLD_WAY_THRESHOLD:
                return True
            else:
                return False
        else:
            return False

    def set_mode_time(self):
        self.current_mode_time[0] = time.time()

    def get_mode_time(self):
        return time.time() - self.current_mode_time[0]
