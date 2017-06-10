import numpy as npimport navigatable_area as navdef to_polar_coords(x_pixel, y_pixel):    __dist = np.sqrt(x_pixel ** 2 + y_pixel ** 2)    __angles = np.arctan2(y_pixel, x_pixel)    return __dist, __anglesclass RoverStatus(object):    ROVER_STABLE_ANGLE_RANGE = 5    def __init__(self, rover):        self.rover = rover    def is_unstable(self):        global ROVER_STABLE_ANGLE_RANGE        if ROVER_STABLE_ANGLE_RANGE < abs(self.rover.pitch) < (360 - ROVER_STABLE_ANGLE_RANGE):            return True        elif ROVER_STABLE_ANGLE_RANGE < abs(self.rover.roll) < (360 - ROVER_STABLE_ANGLE_RANGE):            return True        else:            return False    def has_vision(self):        if self.rover.nav_angles is not None:            return True        else:            return False    def get_navigation_image(self):        return self.rover.vision_image[:, :, 2]class RoverControl(object):    # LEFT_STEEL = 7.5    # RIGHT_STEEL = 7.5    LEFT_STEEL = 9    RIGHT_STEEL = 9    HARD_LEFT_STEEL = 15    HARD_RIGHT_STEEL = 15    THROTTLE_SET = 0.2    def __init__(self, rover):        self.rover = rover    def do_nothing(self):        self.rover.throttle = 0        self.rover.steer = 0    def straight_speedup(self):        global THROTTLE_SET        self.rover.throttle = THROTTLE_SET        self.rover.steer = 0        self.rover.brake = 0# This is where you can build a decision tree for determining throttle, brake and steer# commands based on the output of the perception_step() functiondef decision_step(rover):    rs = RoverStatus(rover)    rc = RoverControl(rover)    if not rs.is_unstable():        rc.do_nothing()        return    if rs.has_vision():        __nav_img = rs.get_navigation_image()        if rover.mode == 'forward':            if len(rover.nav_angles) >= rover.stop_forward:                speed_rate = 1.0                if not nav.is_farfront_area_open(__nav_img):                    speed_rate = 0.5                if rover.vel < rover.max_vel * speed_rate:                    rover.throttle = rover.throttle_set                else:                    # Else coast                    rover.throttle = 0                    rover.brake = 0                                rover.frame_count += 1                if int(rover.frame_count) % 60 == 3:                    rover.vel_history[2] = rover.vel_history[1]                    rover.vel_history[1] = rover.vel_history[0]                    rover.vel_history[0] = rover.vel                __average_vel = 1                if rover.vel_history[0] >= 0 and rover.vel_history[1] >= 0 and rover.vel_history[2] >= 0:                    __average_vel = np.mean(rover.vel_history)                if __average_vel <= 0.1:                    print("Stucked, ROLL_AVOID")                    rover.throttle = 0                    rover.brake = rover.brake_set                    rover.steer = 0                    rover.last_yaw = rover.yaw                    rover.mode = 'roll_avoid'                elif nav.is_front_area_blocked(__nav_img):                    print("Blocked, AVOID")                    rover.throttle = 0                    rover.brake = rover.brake_set                    rover.steer = 0                    rover.mode = 'avoid'                elif nav.is_left_edge_normal(__nav_img) \                        or nav.is_left_edge_blocked(__nav_img) \                        or nav.is_left_edge_left(__nav_img):                    print("Found left edge, FOLLOW_LEFT")                    rover.mode = 'follow_left'                else:                    rover.steer = np.clip(np.mean(rover.nav_angles * 180 / np.pi), -15, 15)            # If there's a lack of navigable terrain pixels then go to 'stop' mode            elif len(rover.nav_angles) < rover.stop_forward:                print("No enough navigatable area, STOP")                rover.throttle = 0                rover.brake = rover.brake_set                rover.steer = 0                rover.mode = 'stop'        elif rover.mode == 'find_left_edge':            if rover.vel < rover.max_vel:                # Set throttle value to throttle setting                rover.throttle = rover.throttle_set            else:                # Else coast                rover.throttle = 0                rover.brake = 0            rover.frame_count += 1            if int(rover.frame_count) % 60 == 3:                rover.vel_history[2] = rover.vel_history[1]                rover.vel_history[1] = rover.vel_history[0]                rover.vel_history[0] = rover.vel            __average_vel = 1            if rover.vel_history[0] >= 0 and rover.vel_history[1] >= 0 and rover.vel_history[2] >= 0:                __average_vel = np.mean(rover.vel_history)            if __average_vel <= 0.1:                print("Stucked, ROLL_AVOID")                rover.throttle = 0                rover.brake = rover.brake_set                rover.steer = 0                rover.last_yaw = rover.yaw                rover.mode = 'roll_avoid'            elif nav.is_front_area_blocked(__nav_img):                print("Blocked, AVOID")                rover.throttle = 0                rover.brake = rover.brake_set                rover.steer = 0                rover.mode = 'avoid'            elif nav.is_left_edge_normal(__nav_img) \                    or nav.is_left_edge_blocked(__nav_img) \                    or nav.is_left_edge_left(__nav_img):                print("Found left edge, FOLLOW_LEFT")                rover.mode = 'follow_left'            else:                if np.abs(rover.last_yaw - rover.yaw) > 90:                    print("Not found, FORWARD")                    rover.last_yaw = -1                    rover.vel_history = np.array([-1.1, -1.1, -1.1])                    rover.mode = 'forward'                else:                    rover.throttle = 0                    rover.brake = 0                    rover.steer = 15        elif rover.mode == 'follow_left':            speed_rate = 1.0            if not nav.is_farfront_area_open(__nav_img):                speed_rate = 0.5            if rover.vel < rover.max_vel * speed_rate:                # Set throttle value to throttle setting                rover.throttle = rover.throttle_set            else:                # Else coast                rover.throttle = 0                rover.brake = 0            rover.frame_count += 1            if int(rover.frame_count) % 60 == 3:                rover.vel_history[2] = rover.vel_history[1]                rover.vel_history[1] = rover.vel_history[0]                rover.vel_history[0] = rover.vel            __average_vel = 1            if rover.vel_history[0] >= 0 and rover.vel_history[1] >= 0 and rover.vel_history[2] >= 0:                __average_vel = np.mean(rover.vel_history)            if __average_vel <= 0.1:                print("Stucked, ROLL_AVOID")                rover.throttle = 0                rover.brake = rover.brake_set                rover.steer = 0                rover.last_yaw = rover.yaw                rover.mode = 'roll_avoid'            elif nav.is_front_area_blocked(__nav_img):                print("Blocked, AVOID")                rover.throttle = 0                rover.brake = rover.brake_set                rover.steer = 0                rover.mode = 'avoid'            elif nav.is_left_edge_clear(__nav_img) and nav.is_left_guide_clear(__nav_img):                print("Left edge missing, FIND")                rover.throttle = 0                rover.brake = rover.brake_set                rover.steer = 0                rover.last_yaw = rover.yaw                rover.vel_history = np.array([-1.1, -1.1, -1.1])                rover.mode = 'find_left_edge'            elif nav.is_left_edge_blocked(__nav_img) and nav.is_right_edge_blocked(__nav_img):                rover.steer = 0            elif nav.is_left_edge_blocked(__nav_img):                rover.steer = -7.5            elif nav.is_right_edge_blocked(__nav_img):                rover.steer = 7.5            elif nav.is_left_edge_clear(__nav_img) and not nav.is_left_guide_clear(__nav_img):                rover.steer = 15            elif nav.is_left_edge_open(__nav_img) or nav.is_left_edge_left(__nav_img):                rover.steer = 7.5            elif nav.is_left_edge_normal(__nav_img) \                    and nav.is_right_front_blocked(__nav_img) \                    and not nav.is_left_front_blocked(__nav_img):                rover.steer = 15            elif nav.is_left_edge_normal(__nav_img) :                rover.steer = 0        elif rover.mode == 'roll_avoid':            if rover.vel > 0.2:                rover.throttle = 0                rover.brake = rover.brake_set                rover.steer = 0            else:                if np.abs(rover.last_yaw - rover.yaw) > 15:                    rover.last_yaw = -1                    rover.vel_history = np.array([-1.1, -1.1, -1.1])                    rover.mode = 'forward'                else:                    rover.throttle = 0                    rover.brake = 0                    rover.steer = -15        elif rover.mode == 'avoid':            if rover.vel > 0.2:                rover.throttle = 0                rover.brake = rover.brake_set                rover.steer = 0            # If we're not moving (vel < 0.2) then do something else            elif rover.vel <= 0.2:                if not nav.is_front_area_blocked(__nav_img):                    rover.vel_history = np.array([-1.1, -1.1, -1.1])                    rover.mode = 'forward'                else:                    rover.throttle = 0                    rover.brake = 0                    rover.steer = -15        # If we're already in "stop" mode then make different decisions        elif rover.mode == 'stop':            # If we're in stop mode but still moving keep braking            if rover.vel > 0.2:                rover.throttle = 0                rover.brake = rover.brake_set                rover.steer = 0            # If we're not moving (vel < 0.2) then do something else            elif rover.vel <= 0.2:                # Now we're stopped and we have vision data to see if there's a path forward                if len(rover.nav_angles) < rover.go_forward:                    rover.throttle = 0                    # Release the brake to allow turning                    rover.brake = 0                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning                    # Could be more clever here about which way to turn                    rover.steer = -15                # If we're stopped but see sufficient navigable terrain in front then go!                if len(rover.nav_angles) >= rover.go_forward:                    # Set throttle back to stored value                    rover.throttle = rover.throttle_set                    # Release the brake                    rover.brake = 0                    # Set steer to mean angle                    rover.steer = np.clip(np.mean(rover.nav_angles * 180 / np.pi), -15, 15)                    rover.mode = 'forward'    # Just to make the rover do something     # even if no modifications have been made to the code    else:        rc.straight_speedup()    if rover.near_sample and rover.vel == 0 and not rover.picking_up:        rover.send_pickup = True    return rover