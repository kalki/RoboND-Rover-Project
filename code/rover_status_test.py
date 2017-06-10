import time
from rover_status import RoverStatus


class R(object):

    def __init__(self, x, y, yaw):
        self.pos = (x, y)
        self.yaw = yaw


r1 = R(1, 1, 1)
r2 = R(2, 1, 1)
r3 = R(3, 1, 1)
r4 = R(3, 1, 31)

s1 = RoverStatus(r1)
s1.update_status()

time.sleep(1.1)

s2 = RoverStatus(r2)
s2.update_status()

time.sleep(1.1)

s3 = RoverStatus(r3)
s3.snapshot()
s3.update_status()

s4 = RoverStatus(r4)
s4.update_status()

print(s4.get_yaw_diff_since_snapshot())
