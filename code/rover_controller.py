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

    def direction_to_yaw(self, yaw):
        self.rover.steer = np.clip(yaw, -15, 15)

    def direction_to(self, angle):
        self.rover.steer = np.clip(angle * 180 / np.pi, -15, 15)

    def direction_navigatable(self):
        self.rover.steer = np.clip(np.mean(self.rover.nav_angles * 180 / np.pi), -15, 15)

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
