import numpy as np

MODE_FORWARD = 'forward'
MODE_FIND_LEFT = 'find_left_edge'
MODE_FOLLOW_LEFT = 'follow_left'
MODE_AVOID_FIX_ANGEL = 'roll_avoid'
MODE_AVOID = 'avoid'
MODE_STOP = 'stop'
MODE_ROCK_APPROACH = 'rock_approach'

LEFT_STEEL = 7.5
RIGHT_STEEL = -7.5
HARD_LEFT_STEEL = 15
HARD_RIGHT_STEEL = -15


class RoverController(object):

    MODE_FORWARD = MODE_FORWARD
    MODE_FIND_LEFT = MODE_FIND_LEFT
    MODE_FOLLOW_LEFT = MODE_FOLLOW_LEFT
    MODE_AVOID_FIX_ANGEL = MODE_AVOID_FIX_ANGEL
    MODE_AVOID = MODE_AVOID
    MODE_STOP = MODE_STOP
    MODE_ROCK_APPROACH = MODE_ROCK_APPROACH

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
        elif self.rover.vel < self.rover.max_vel * speed_rate:
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

    def direction_to(self, angle):
        self.rover.steer = np.clip(angle * 180 / np.pi, -15, 15)

    def direction_navigatable(self):
        self.rover.steer = np.clip(np.mean(self.rover.nav_angles * 180 / np.pi), -15, 15)

    def mode_to_avoid(self):
        print("Switch to AVOID")
        self.direction_straight()
        self.speed_brake()
        self.rover.mode = MODE_AVOID

    def mode_to_stop(self):
        print("Switch to STOP")
        self.direction_straight()
        self.speed_brake()
        self.rover.mode = MODE_STOP

    def mode_to_avoid_fix(self):
        print("Switch to AVOID FIX")
        self.direction_straight()
        self.speed_brake()
        self.rover_status.snapshot()
        self.rover.mode = MODE_AVOID_FIX_ANGEL

    def mode_to_rock_approach(self):
        print("Switch to ROCK APP")
        self.speed_brake()
        self.rover.mode = MODE_ROCK_APPROACH

    def mode_to_find_left(self):
        print("Switch to FIND LEFT")
        self.speed_brake()
        self.direction_straight()
        self.rover_status.snapshot()
        self.rover.mode = MODE_FIND_LEFT

    def mode_to_follow_left(self):
        print("Switch to FOLLOW LEFT")
        self.rover_status.reset_status()
        self.rover.mode = MODE_FOLLOW_LEFT

    def mode_to_forward(self):
        print("Switch to FORWARD")
        self.rover_status.reset_status()
        self.rover.mode = MODE_FORWARD
