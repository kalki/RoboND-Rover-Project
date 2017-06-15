import numpy as np

MODE_FORWARD = 'forward'
MODE_FIND_LEFT = 'find_left_edge'
MODE_FOLLOW_LEFT = 'follow_left'
MODE_TURN_FIX_ANGEL = 'turn_fix'
MODE_STOP = 'stop'
MODE_ROCK_APPROACH = 'rock_approach'
MODE_NAVIGATE = 'navigate'
MODE_FINISH = 'finish'

LEFT_STEEL = 7.5
RIGHT_STEEL = -7.5
HARD_LEFT_STEEL = 15
HARD_RIGHT_STEEL = -15


def get_dist_angle(source, target):
    __s_x = source[0]
    __s_y = source[1]
    __t_x = target[0]
    __t_y = target[1]
    __yaw = np.arctan2(__t_y - __s_y, __t_x - __s_x) * 180 / np.pi
    __dist = np.sqrt((__t_x - __s_x) ** 2 + (__t_y - __s_y) ** 2)
    return __dist, __yaw


def is_close(p1, p2):
    __x1 = int(p1[0])
    __y1 = int(p1[1])
    __x2 = int(p2[0])
    __y2 = int(p2[1])
    if np.abs(__x1 - __x2) <= 1 and np.abs(__y1 - __y2) <= 1:
        return True
    else:
        return False


class RoverController(object):

    MODE_FORWARD = MODE_FORWARD
    MODE_FIND_LEFT = MODE_FIND_LEFT
    MODE_FOLLOW_LEFT = MODE_FOLLOW_LEFT
    MODE_TURN_FIX_ANGEL = MODE_TURN_FIX_ANGEL
    MODE_STOP = MODE_STOP
    MODE_ROCK_APPROACH = MODE_ROCK_APPROACH
    MODE_NAVIGATE = MODE_NAVIGATE
    MODE_FINISH = MODE_FINISH
    goal = [(0, 0)]
    waypoint = [(0, 0)]
    reached_waypoint = [(0, 0)]

    def __init__(self, rover, rover_status):
        self.rover = rover
        self.rover_status = rover_status

    def do_nothing(self):
        self.rover.brake = 0
        self.rover.throttle = 0
        self.rover.steer = 0

    def speed_brake(self):
        self.rover.throttle = 0
        self.rover.brake = self.rover.brake_set

    def speed_up(self, speed_rate=1.0):

        if self.rover.vel < self.rover.max_vel * speed_rate:
            self.rover.throttle = self.rover.throttle_set
            self.rover.brake = 0
        else:
            self.rover.throttle = 0
            self.rover.brake = 0

    def speed_limit(self, speed_rate=1.0):

        if self.rover.vel > self.rover.max_vel * speed_rate * 1.5:
            self.rover.throttle = 0
            self.rover.brake = self.rover.brake_set
        elif self.rover.vel > self.rover.max_vel * speed_rate * 1.1:
            self.rover.throttle = 0
            self.rover.brake = self.rover.brake_set * 0.4
        elif self.rover.vel < self.rover.max_vel * speed_rate * 0.9:
            self.rover.throttle = self.rover.throttle_set
            self.rover.brake = 0
        else:
            self.rover.throttle = 0
            self.rover.brake = 0

    def direction_straight(self):
        self.rover.steer = 0

    def direction_left(self):
        self.rover.steer = LEFT_STEEL

    def direction_hard_left(self):
        self.rover.steer = HARD_LEFT_STEEL

    def direction_right(self):
        self.rover.steer = RIGHT_STEEL

    def direction_hard_right(self):
        self.rover.steer = HARD_RIGHT_STEEL

    def direction_to_yaw(self, yaw):
        self.rover.steer = np.clip(yaw, -15, 15)

    def direction_to(self, angle):
        self.rover.steer = np.clip(angle * 180 / np.pi, -15, 15)

    def direction_navigatable(self):
        self.rover.steer = np.clip(np.mean(self.rover.nav_angles * 180 / np.pi), -15, 15)
        return self.rover.steer

    def mode_to_stop(self):
        print("STOP")
        self.direction_straight()
        self.speed_brake()
        self.rover_status.set_mode_time()
        self.rover.mode = MODE_STOP

    def mode_to_turn_fix(self):
        print("TURN FIX")
        self.direction_straight()
        self.speed_brake()
        self.rover_status.snapshot()
        self.rover_status.set_mode_time()
        self.rover.mode = MODE_TURN_FIX_ANGEL

    def mode_to_rock_approach(self):
        print("ROCK PICK")
        self.speed_brake()
        self.rover_status.set_mode_time()
        self.rover.mode = MODE_ROCK_APPROACH

    def mode_to_find_left(self):
        print("FIND LEFT EDGE")
        self.speed_brake()
        self.direction_straight()
        self.rover_status.snapshot()
        self.rover_status.set_mode_time()
        self.rover.mode = MODE_FIND_LEFT

    def mode_to_follow_left(self):
        print("FOLLOW LEFT EDGE")
        self.rover_status.reset_status()
        self.rover_status.set_mode_time()
        self.rover.mode = MODE_FOLLOW_LEFT

    def mode_to_forward(self):
        print("FORWARD")
        self.rover_status.reset_status()
        self.rover_status.set_mode_time()
        self.rover.mode = MODE_FORWARD

    def mode_to_navigate(self, goal):
        print("NAVIGATE to: {}".format(goal))
        self.goal[0] = (goal[0], goal[1])
        self.speed_brake()
        self.rover_status.set_mode_time()
        self.rover.mode = MODE_NAVIGATE

    def mode_to_finish(self):
        print("FINISHED")
        self.speed_brake()
        self.direction_straight()
        self.rover_status.set_mode_time()
        self.rover.mode = MODE_FINISH

    def head_to_waypoint(self):

        if is_close(self.rover.pos, self.waypoint[0]):
            self.do_nothing()
            self.reached_waypoint[0] = self.waypoint[0]
            self.waypoint[0] = (0, 0)
            print("Way point {:} reached".format(self.reached_waypoint[0]))
        else:
            __t_dist, __t_yaw = get_dist_angle(self.rover.pos, self.waypoint[0])

            if __t_yaw < 0:
                __t_yaw = __t_yaw + 360
            __yaw_diff = (__t_yaw - self.rover.yaw)

            if np.abs(__yaw_diff) > 10 and __t_dist > 10 \
                    and self.rover_status.is_right_front_open() and self.rover_status.is_left_front_open():
                if self.rover.vel > 0.2:
                    self.speed_brake()
                else:
                    self.do_nothing()
                    self.direction_to_yaw(int(__yaw_diff / 2))
            elif (not self.rover_status.is_right_front_open() or not self.rover_status.is_left_front_open()) \
                    and np.abs(__yaw_diff) > 45 and __t_dist > 10:
                if self.rover.vel > 0.2:
                    self.speed_brake()
                else:
                    self.do_nothing()
                    self.direction_to_yaw(int(__yaw_diff / 2))
            else:
                if __t_dist > 30:
                    self.speed_up()
                elif __t_dist > 5:
                    if self.rover.vel * 10 - 5 > __t_dist:
                        self.rover.brake = 0.0
                        self.rover.throttle = 0
                    else:
                        self.rover.brake = 0
                        self.rover.throttle = 0.4
                elif __t_dist > 2:
                    self.speed_limit(0.3)
                else:
                    self.speed_limit(0.1)

                if not self.rover_status.is_right_front_open() or not self.rover_status.is_left_front_open():
                    if not self.rover_status.is_right_front_open():
                        self.direction_left()
                    else:
                        self.direction_right()
                else:
                    self.direction_to_yaw(int(__yaw_diff / 2))

    def set_waypoint(self, waypoint):
        print("Way point set to {:}".format(waypoint))
        self.waypoint[0] = waypoint

    def get_waypoint(self):
        return self.waypoint[0]

    def set_goal(self, goal):
        self.goal[0] = goal

    def get_goal(self):
        return self.goal[0]

    def is_goal_reached(self):
        if self.reached_waypoint[0] == self.goal[0] and (self.goal[0] != (0, 0)):
            return True
        else:
            return False
