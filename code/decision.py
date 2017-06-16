import timeimport numpy as npfrom rover_status import RoverStatusfrom rover_controller import RoverControllerimport rover_controller as rocimport rock_locator as rolimport path_finder as pf# Some middle position of mapRETURN_POINT = (100, 90)# Get not closed naviagtable point in explored map# If find a path from rover to map coner and consider all non obstacles and open area.def get_unnavigated_point(rover):    __start_x = int(rover.pos[0] / (rover.worldmap.shape[1] / 2)) * rover.worldmap.shape[1]    __start_y = int(rover.pos[1] / (rover.worldmap.shape[0] / 2)) * rover.worldmap.shape[0]    __start = (__start_x, __start_y)    __goal = (rover.pos[0], rover.pos[1])    __img_obstacles = rover.worldmap[:, :, 0]    __img_navigatable = rover.worldmap[:, :, 2]    __img = np.ones_like(__img_obstacles)    __obstacles_select = __img_obstacles > 0    __navigatable_select = __img_navigatable > 0    __img[__obstacles_select] = 0    __img[__navigatable_select] = 1    __path = pf.search_path(__img, __start, __goal)    if len(__path) <= 0:        return None    __p = pf.get_fisrt_outside_point(__path, __img_navigatable)    return __p# This is where you can build a decision tree for determining throttle, brake and steer# commands based on the output of the perception_step() functiondef decision_step(rover):    rs = RoverStatus(rover)    rc = RoverController(rover, rs)    r_img = rs.get_rock_image()    # Do nothing is rover is not stable since imge input will be wrong    # If rover is not stable but stuck, try to do something to get out    if rs.is_unstable():        rs.update_status()        if rs.is_not_moving():            if int(time.time() / 2) % 8 == 0:                rc.do_nothing()                rc.direction_hard_right()            elif int(time.time() / 2) % 8 == 2:                rc.direction_straight()                rc.speed_brake()            elif int(time.time() / 2) % 8 == 4:                rc.direction_straight()                rc.speed_up()            else:                rc.do_nothing()        return rover    # Do something when no vision    if not rs.has_vision():        rc.speed_up()        rc.direction_straight()        return rover    # Do nothing when task is finished    if rover.mode == rc.MODE_FINISH:        return rover    # Pick rock is has rock is defined area    if rover.mode == rc.MODE_ROCK_APPROACH:        # Picking is moving slow, it is possible trigger not moving process when switch between moving and pcking        # Set changing value in position history to prevent it.        rs.stop_recording_position()        # Send pick command if possible        if rover.near_sample and rover.vel == 0 and not rover.picking_up:            rover.send_pickup = True        # Keep move close if rock is in sight        if rol.is_rock_insight(r_img):            # Get distance and angle of closest rock            __d, __a = rol.get_closest_rock_polar(r_img)            # Brake to trigger pick            if rover.near_sample:                rc.speed_brake()            # Move slower and turn more when distance is closing            else:                if __d > 40:                    rc.speed_limit(0.3)                    rc.direction_to(__a/6)                elif __d > 20:                    rc.speed_limit(0.2)                    rc.direction_to(__a/3)                elif __d > 10:                    rc.speed_limit(0.1)                    rc.direction_to(__a)                # Distance < 10 means near sample                else:                    if rs.is_moving():                        rc.speed_brake()                    else:                        rc.do_nothing()        # Change to moving forward mode if rock is not in sight, maybe is picked, or just because camera angle change        else:            rc.mode_to_forward()        # Left picking mode is stuck, it is possible when rock is at left side but right side is not passable.        # Because rover only approach to rock from right side to prevent brake left edge follow rule        if rs.get_mode_time() > 20:            rc.mode_to_turn_fix()        return rover    # Mode to move to target on explored map    if rover.mode == rc.MODE_NAVIGATE:        __start = (rover.pos[0], rover.pos[1])        __goal = rc.get_goal()        __waypoint = rc.get_waypoint()        # Left navigate mode if goal is reach        if rc.is_goal_reached():            # If goal is return point, means task finished            if __goal == RETURN_POINT:                rc.set_goal((0, 0))                rc.mode_to_finish()                return rover            # Or continur move forward            else:                rc.set_goal((0, 0))                rc.mode_to_forward()                return rover        # Get a new way point if goal is not empty but way point is empty (usuallly it mean way point reached)        if __goal != (0, 0) and __waypoint == (0, 0):            # Get path and reduce it, use last point in reduce path (it has no start in it)            # Although rover may be on last waypoint, but usually that is OK because reduce path from rover to goal            # will has same next point, no matter rover is on last way point or neighbour of it            __navigatable = rs.get_navigation_map()            __path = pf.search_path(__navigatable, __start, __goal)            if len(__path) > 0:                __path = pf.reduce_path(__path, __navigatable)                __w = __path[-1]                # Remove way point close to current rover position because it always consider as reached and cause                # dead loop                if roc.is_close(__w, __start):                    __w = __path[-2]                rc.set_waypoint(__w)                __waypoint = rc.get_waypoint()        # Move to way point if it is set        if __waypoint != (0, 0):            rc.head_to_waypoint()        else:            # Left navigate mode if no way point and blocked,            if rs.is_not_moving():                rc.mode_to_turn_fix()            elif rs.is_front_blocked():                rc.mode_to_turn_fix()            # Random walk if not blocked            else:                rs.update_status()                rc.speed_up()                rc.direction_navigatable()        return rover    # Moving mode, base on navigatable vision image or left edge, it also:    # 1. Turn fix angle if blocked    # 2. Stop if no enough vision    # 3. Pick rock if rock insight    # 4. Navigate to return or unnavigated point if rover repeat old actions    if rover.mode == rc.MODE_FORWARD or rover.mode == rc.MODE_FOLLOW_LEFT:        # Normal move is has enough pixel for navigate        if rs.has_vision_for_navigate_during_forward():            # If rover is repeat old actions (position and angle), try to get away            if rs.is_on_same_way_too_long():                # If has no navigated point, move to it, rr move to return point and finish                __p = get_unnavigated_point(rover)                if __p is not None:                    __goal = __p                else:                    __goal = RETURN_POINT                # If there is path from rover to target, switch to navigate mode, or keep walking                __start = (rover.pos[0], rover.pos[1])                __navigatable = rs.get_navigation_map()                __path = pf.search_path(__navigatable, __start, __goal)                if len(__path) > 0:                    rc.mode_to_navigate(__goal)                    return rover            # pick rock if rock in sight            if rol.is_rock_insight(r_img):                rc.mode_to_rock_approach()            # Normal move if no special case            else:                # Decide speed                rs.update_status()                speed_rate = rs.decide_speed_rate()                rc.speed_up(speed_rate)                # Turn a fixed angle if not moving, blocked                if rs.is_not_moving():                    rc.mode_to_turn_fix()                elif rs.is_front_blocked():                    rc.mode_to_turn_fix()                elif rs.is_left_front_blocked():                    rc.mode_to_turn_fix()                else:                    # Move base on navigatable vision image                    if rover.mode == rc.MODE_FORWARD:                        # Switch to follow edge mode if left side has obstacles                        if rs.is_left_guide_has_visual():                            rc.mode_to_follow_left()                        else:                            rc.direction_navigatable()                    # Move base on left edge                    else:                        # If entire left side is clear, switch to                        if rs.is_left_edge_missing():                            rc.mode_to_find_left()                        # Decide direction                        elif rs.is_both_edge_blocked():                            rc.direction_straight()                        elif rs.is_left_edge_hard_left():                            rc.do_nothing()                            rc.direction_hard_left()                        elif rs.is_left_edge_left():                            rc.direction_left()                        elif rs.is_left_edge_right():                            rc.direction_right()                        elif rs.is_left_edge_hard_right():                            rc.do_nothing()                            rc.direction_hard_right()                        elif rs.is_left_edge_normal():                            rc.direction_straight()        # Stop if has no enough pixel for navigate        else:            rc.mode_to_stop()    # Mode to turn left 60 degree to see if there is obstacles to follow at left side of rover    # Switch to forward mode if no found    elif rover.mode == rc.MODE_FIND_LEFT:        if rs.is_moving():            rc.speed_brake()            rc.direction_straight()        else:            if rs.get_yaw_diff_since_snapshot() > 60:                rc.mode_to_forward()            elif rs.is_left_guide_has_visual():                rc.mode_to_follow_left()            else:                rc.do_nothing()                rc.direction_hard_left()    # Mode to stop and turn right 15 degree    elif rover.mode == rc.MODE_TURN_FIX_ANGEL:        if rs.is_moving():            rc.speed_brake()            rc.direction_straight()        else:            if rs.get_yaw_diff_since_snapshot() > 15:                rc.mode_to_forward()            else:                rc.do_nothing()                rc.direction_hard_right()    # Mode to stop and turn right until has enough visual    elif rover.mode == rc.MODE_STOP:        if rs.is_moving():            rc.speed_brake()            rc.direction_straight()        else:            if not rs.has_vision_for_start_moving():                rc.do_nothing()                rc.direction_hard_right()            else:                rc.mode_to_forward()    return rover