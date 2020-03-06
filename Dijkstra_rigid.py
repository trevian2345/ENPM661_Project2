import argparse
from baseRobot import Robot


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Project 2:  Navigation of a rigid, circular robot from a start point"
                                                 "to an end point.")
    parser.add_argument('initialX', type=int, help='X-coordinate of initial node of the robot')
    parser.add_argument('initialY', type=int, help='Y-coordinate of initial node of the robot')
    parser.add_argument('goalX', type=int, help='X-coordinate of goal node of the robot')
    parser.add_argument('goalY', type=int, help='Y-coordinate of goal node of the robot')
    parser.add_argument('radius', type=int, help='Indicates the radius of the rigid robot')
    parser.add_argument("clearance", type=int,
                        help="Indicates the minimum required clearance between the rigid robot and obstacles")
    args = parser.parse_args()

    sx = args.initialX
    sy = args.initialY
    gx = args.goalX
    gy = args.goalY
    r = args.radius
    c = args.clearance
    start_pos = (sx, sy)
    goal_pos = (gx, gy)
    rigidRobot = Robot(start_pos, goal_pos, r, c, rigid=True)
