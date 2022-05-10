import numpy as np
import time
import stretch_body.xbox_controller
import stretch_body.robot
import teleop
def rotate_robot(robot, angle):
    while angle > np.pi:
        angle -= 2*np.pi
    while angle < -np.pi:
        angle += 2*np.pi
    robot.base.rotate_by(angle, v_r=10)
    robot.push_command()
    time.sleep(2 * abs(angle))

def goto_orbit(robot, orbit_radius, base_angle, orbit_angle, target_orbit_radius, target_base_angle, target_orbit_angle, target_face_angle):
    pos = orbit_radius * np.array([np.cos(orbit_angle), np.sin(orbit_angle)])
    target_pos = target_orbit_radius * np.array([np.cos(target_orbit_angle), np.sin(target_orbit_angle)])
    delta_pos = target_pos - pos
    traj_angle = np.arctan2(delta_pos[1], delta_pos[0])

    rotate_robot(robot, traj_angle - base_angle)
    
    robot.base.translate_by(np.linalg.norm(delta_pos), v_m=1)
    robot.push_command()
    time.sleep(10 * np.linalg.norm(delta_pos))
    
    rotate_robot(robot, target_base_angle - traj_angle)

    robot.head.move_to('head_pan', target_face_angle)
    

def move(robot, current, target, orbit_radius, target_orbit_radius):
    orbit_angle_dict = {
        "left" : np.pi/2,
        "back" : np.pi,
        "right" : np.pi/2 * 3,
        "leftback" : np.pi*3/4,
        "backleft" : np.pi*3/4,
        "rightback" : np.pi*5/4,
        "backright" : np.pi*5/4,
    }
    base_angle_dict = {
        "leftback" : np.pi*5/4,
        "backleft" : np.pi*1/4,
        "rightback" : np.pi*3/4,
        "backright" : np.pi*7/4,
    }
    face_angle_dict = {
        "left" : -np.pi/2,
        "back" : 0,
        "right" : np.pi,
        "leftback" : -np.pi/2,
        "backleft" : 0,
        "rightback" : 0,
        "backright" : np.pi,
    }
    middle = current + target
    middle_radius = (orbit_radius + target_orbit_radius) / 2
    goto_orbit(robot, orbit_radius, 0, orbit_angle_dict[current], middle_radius, base_angle_dict[middle], orbit_angle_dict[middle], face_angle_dict[middle])
    goto_orbit(robot, middle_radius, base_angle_dict[middle], orbit_angle_dict[middle], target_orbit_radius, 0, orbit_angle_dict[target], face_angle_dict[target])

def test_move(robot):
    
    robot.head.move_to('head_pan', 0)
    time.sleep(1)
    current = "back"
    next = "right"
    current_dis = 1
    next_dis = 0.5
    move(robot, current, next, current_dis, next_dis)


if __name__=="__main__":
    robot=stretch_body.robot.Robot()
    robot.startup()
    # main()
    teleop.teleop(robot)
    test_move(robot)
    teleop.teleop(robot)
