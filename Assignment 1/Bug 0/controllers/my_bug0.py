"""bug_0_algo controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
#Done in discussion with Kavin RV
from controller import Robot, GPS
import math
from sympy import Polygon, Point, Ellipse

# TREE = 

def calc_dir(p1, p2):
    x = p1[0]-p2[0]
    y = p1[1]-p2[1]
    return math.atan2(y, x)
    
    
def gen_line(p1, p2):

    x1, y1 = p1[0], p1[1]
    x2, y2 = p2[0], p2[1]
    a = -y1 + y2
    b = x1 - x2
    c = - b * y1 - a * x1
    return {"a": a, "b": b, "c": c}


def dist(p1, p2):
    x1, y1 = p1[0], p1[1]
    x2, y2 = p2[0], p2[1]
    dist = math.sqrt(((x2-x1)*(x2-x1))+((y2-y1)*(y2-y1)))
    dist = round(dist, 2)
    return dist


def calc_poly_dist(poly, p):

    v = map(Point, poly)
    poly = Polygon(*v)
    point = p

    return poly.distance(Point(point[0], point[1])) * (int(poly.encloses_point(Point(point[0], point[1]))) * (-2) + 1)



def calc_poly_tang(poly, p):

    v = map(Point, poly)
    poly = Polygon(*v)
    point = p

    rad = poly.distance(Point(point))
    circ = Ellipse(Point(point), rad, rad)
    cls_point = poly.intersection(circ)
    x_tang = cls_point[0][0] - point[0]
    y_tang = cls_point[0][1] - point[1]
    return math.atan2(y_tang, x_tang)


def poly_intersect(poly1, poly2):

    v11, v12, v13, v14 = map(Point, poly1)
    v21, v22, v23, v24 = map(Point, poly2)

    poly1 = Polygon(v11, v12, v13, v14)
    poly2 = Polygon(v21, v22, v23, v24)

    ret = list(map(list, poly1.intersection(poly2)))
    return ret



def move_robot(robot):

    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())
    goal_poly = [(0.5, 1.0), (0.5, 0.6), (0.1, 0.6), (0.1, 1.0)]
    tree_poly = [(0.4, 0.9), (0.4, 0.7), (0.2, 0.7), (0.2, 0.9)]
    goal = (-0.1, 0.5)
    
    # Motor instantiation
    left_motor = robot.getDevice("left wheel motor")
    right_motor = robot.getDevice("right wheel motor")
    
    left_motor.setPosition(float("inf"))
    left_motor.setVelocity(0.0)
    
    right_motor.setPosition(float("inf"))
    right_motor.setVelocity(0.0)
    
    # Sonor Sensor
    so_sen = []
    for i in range(8):
        s = f"ps{str(i)}"
        so_sen.append(robot.getDevice(s))
        so_sen[i].enable(timestep)
        dist_val.append(0)
        
        
    front_gps = robot.getDevice("front_gps")
    front_gps.enable(timestep)
    
    m_gps = robot.getDevice("mid_gps")
    m_gps.enable(timestep)
        
    
    # Main loop:
    while robot.step(timestep) != -1:
        

        for i, s in enumerate(so_sen):
            dist_val[i] = s.getValue()
        
        p1 = (float(front_gps.getValues()[0]), float(front_gps.getValues()[1]))
        p2 = (float(m_gps.getValues()[0]), float(m_gps.getValues()[1]))
        
        targ_angle = calc_dir(goal, p1)
        curr_angle = calc_dir(p1, p2)
        
        if dist(goal, p2) < 0.1:
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            print("Target reached")
            break;

            
        front_wall = max(dist_val[7], dist_val[1]) > 78
        right_wall = dist_val[2] > 80
        if abs(targ_angle - curr_angle) > 0.2 and not front_wall and not right_wall:
            if targ_angle > curr_angle:
                left_speed = -0.75*wanna_speed
                right_speed = wanna_speed
            else:
                right_speed = -0.75*wanna_speed
                left_speed = wanna_speed
                
        elif front_wall:
            left_speed = -wanna_speed
            right_speed = wanna_speed
        else: 
            left_speed = wanna_speed
            right_speed = 0.9*wanna_speed
        
        
        left_motor.setVelocity(min(left_speed, 6.28))
        right_motor.setVelocity(min(right_speed, 6.28))
        
    
# create the Robot instance.
dist_val = []
max_speed = 6.28
wanna_speed = 6.28
robot = Robot()
move_robot(robot)