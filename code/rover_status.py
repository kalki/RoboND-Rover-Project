import numpy as np
import time
from queue import Queue
import navigatable_area as nav
from navigatable_area import NavigationArea

ROVER_STABLE_ANGLE_RANGE = 2
SAME_WAY_THRESHOLD = 20
SAME_WAY_COOLDDOWN = 60


# Class to judge rover status
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
    last_same_way_enter_time = [-1.0]

    def __init__(self, rover):
        self.rover = rover
        self.na = NavigationArea(self.get_navigation_image())

    # If rover pitch and roll is not close zero, camera input can not be used
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

    # Consider rover is not moving if its position and yaw is not change in last several seconds
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

    # See speed limit base on navigatable area
    def decide_speed_rate(self):
        __speed_rate = 1.0
        __na = self.na
        if not __na.is_area_open(nav.L_FRONT_FAR_NAME) or not __na.is_area_open(nav.R_FRONT_FAR_NAME):
            __speed_rate = 0.5
        return __speed_rate

    # Clear rover snapshot information
    def clear_snapshot(self):
        self.last_snapshot_yaw[0] = -1.0
        self.last_snapshot_x_pos[0] = 0.0
        self.last_snapshot_y_pos[0] = 0.0

    # Store current rover status
    def snapshot(self):
        self.last_snapshot_yaw[0] = self.rover.yaw
        self.last_snapshot_x_pos[0] = self.rover.pos[0]
        self.last_snapshot_y_pos[0] = self.rover.pos[1]

    # When picking rock, position change is very small, and it is often switch between moving and picking mode,
    # It is possible to trigger not moving condition in moving mode and strat turning and lose visual of rock.
    # To prevent this
    def stop_recording_position(self):
        self.x_pos.pop(0)
        self.x_pos.append(time.time())
        self.y_pos.pop(0)
        self.y_pos.append(time.time())
        self.yaw.pop(0)
        self.yaw.append(time.time())

    # Update position history and action history, to decide if rover is moving and on same way it travelled before
    def update_status(self, force=False):
        __time = time.time()

        # Update only every second or specified
        if force or (__time - self.last_update_time[0]) > 1.0:
            __x = self.rover.pos[0]
            __y = self.rover.pos[1]
            __yaw = self.rover.yaw
            self.x_pos.pop(0)
            self.x_pos.append(__x)
            self.y_pos.pop(0)
            self.y_pos.append(__y)
            self.yaw.pop(0)
            self.yaw.append(__yaw)
            self.last_update_time[0] = __time

            # Round action to simplify judgement it is same way
            __x = int(__x / 4) * 4
            __y = int(__y / 4) * 4
            __yaw = int(__yaw / 30) * 30
            __current_action = (__x, __y, __yaw)
            # Action will not be used immediatly, it has a configured cooldown to prevent it trigger all the time
            # 60 seconds is long enough for any action that will keep rover in same place and angle
            self.action_buffer.put(__current_action)
            if self.action_buffer.qsize() > SAME_WAY_COOLDDOWN:
                # Move action before cooldown to history
                __action = self.action_buffer.get()
                self.old_action_history.add(__action)

            # If action happends before, start count time
            if __current_action in self.old_action_history:
                # Count time
                if self.last_same_way_enter_time[0] < 0:
                    self.last_same_way_enter_time[0] = time.time()
                    print("REPEATED ACTION FOUND")
            else:
                # action not happen before, reset timer
                if self.last_same_way_enter_time[0] > 0:
                    self.last_same_way_enter_time[0] = -1.0
                    print("REPEATED ACTION RESET")

    def get_yaw_diff_since_snapshot(self):
        return abs(self.last_snapshot_yaw[0] - self.rover.yaw)

    def is_on_same_way_too_long(self):
        if self.last_same_way_enter_time[0] > 0:
            s = time.time() - self.last_same_way_enter_time[0]
            if s > SAME_WAY_THRESHOLD:
                return True
            else:
                return False
        else:
            return False

    def set_mode_time(self):
        self.current_mode_time[0] = time.time()

    def get_mode_time(self):
        return time.time() - self.current_mode_time[0]

    def is_left_front_open(self):
        li_open = self.na.is_area_open(nav.L_IMPACT_NAME)
        lfc_open = self.na.is_area_open(nav.L_FRONT_CLOSE_NAME)
        return li_open and lfc_open

    def is_right_front_open(self):
        ri_open = self.na.is_area_open(nav.R_IMPACT_NAME)
        rfc_open = self.na.is_area_open(nav.R_FRONT_CLOSE_NAME)
        return ri_open and rfc_open

    def is_left_front_blocked(self):
        li_block = self.na.is_area_blocked(nav.L_IMPACT_NAME)
        lfc_block = self.na.is_area_blocked(nav.L_FRONT_CLOSE_NAME)
        return li_block or lfc_block

    def is_right_front_blocked(self):
        ri_block = self.na.is_area_blocked(nav.R_IMPACT_NAME)
        rfc_block = self.na.is_area_blocked(nav.R_FRONT_CLOSE_NAME)
        return ri_block or rfc_block

    def is_close_front_blocked(self):
        lfc_block = self.na.is_area_blocked(nav.L_FRONT_CLOSE_NAME)
        rfc_block = self.na.is_area_blocked(nav.R_FRONT_CLOSE_NAME)
        return lfc_block and rfc_block

    def is_left_guide_has_visual(self):
        lg_open = self.na.is_area_open(nav.L_GUIDE_NAME)
        return not lg_open

    def is_left_edge_missing(self):
        lec_clear = self.na.is_area_clear(nav.L_EDGE_CLOSE_NAME)
        lef_clear = self.na.is_area_clear(nav.L_EDGE_FAR_NAME)
        lg_clear = self.na.is_area_clear(nav.L_GUIDE_NAME)
        return lec_clear and lef_clear and lg_clear

    def is_both_edge_blocked(self):
        lef_block = self.na.is_area_blocked(nav.L_EDGE_FAR_NAME)
        ref_block = self.na.is_area_blocked(nav.R_EDGE_FAR_NAME)
        return lef_block and ref_block

    def is_left_edge_hard_left(self):
        lec_clear = self.na.is_area_clear(nav.L_EDGE_CLOSE_NAME)
        lef_block = self.na.is_area_blocked(nav.L_EDGE_FAR_NAME)
        lef_clear = self.na.is_area_clear(nav.L_EDGE_FAR_NAME)
        lg_clear = self.na.is_area_clear(nav.L_GUIDE_NAME)
        lff_block = self.na.is_area_blocked(nav.L_FRONT_FAR_NAME)
        rff_block = self.na.is_area_blocked(nav.R_FRONT_FAR_NAME)
        rfc_open = self.na.is_area_open(nav.R_FRONT_CLOSE_NAME)
        return (lec_clear and lef_clear and not lg_clear) or\
               (not lef_block and (not lff_block) and rff_block) or \
               (not lef_block and (not lff_block) and not rfc_open)

    def is_left_edge_left(self):
        lef_block = self.na.is_area_blocked(nav.L_EDGE_FAR_NAME)
        ref_block = self.na.is_area_blocked(nav.R_EDGE_FAR_NAME)
        lef_open = self.na.is_area_open(nav.L_EDGE_FAR_NAME)
        return (not lef_block and ref_block) or lef_open

    def is_left_edge_normal(self):
        lef_noraml = self.na.is_area_normal(nav.L_EDGE_FAR_NAME)
        return lef_noraml

    def is_left_edge_right(self):
        lef_block = self.na.is_area_blocked(nav.L_EDGE_FAR_NAME)
        return lef_block

    def is_left_edge_hard_right(self):
        lff_block = self.na.is_area_blocked(nav.L_FRONT_FAR_NAME)
        return self.is_left_edge_normal() and lff_block
