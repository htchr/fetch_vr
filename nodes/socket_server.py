#!/home/jack/catkin_ws/src/fetch_vr/venv/bin/python3

import asyncio
import websockets
import rospy
# import robot_api
from head import Head
from vr_teleop import PublishThread
from fetch_data_center.msg import RobotStatus, ManipulationStatus, NavigationGoal
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from math import pi
import csv

class Websocket_Pan_Tilt():
    def __init__(self):
        self.head = Head()
        repeat = rospy.get_param("~repeat_rate", 0.0)
        self.speed = rospy.get_param("~speed", 0.5)
        self.turn = rospy.get_param("~turn", 0.5)
        self.teleop = PublishThread(repeat)
        self.teleop.wait_for_subscribers()
        self.init_rads = None
        self.tilt = 0
        self.pan = 0
        self.step = 0.1
        # self.head.pan_tilt(0, 0) # reset head
        self.user_status = ' '
        self.man_status = ' '
        self.pose = ' ; '
        rospy.Subscriber('user_input/status', RobotStatus, self.user_input_status_cbk)
        rospy.Subscriber('manipulation/status', ManipulationStatus, self.manipulation_status_cbk)
        rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.map_pose_cbk)
        self.coords_pub = rospy.Publisher('user_input/image_coords', Point, queue_size=1)
        # nav goals
        self.nav_goal_pub = rospy.Publisher('user_input/navigation_goal', NavigationGoal, queue_size=10)
        rospy.sleep(1)
        # locations_path = rospy.get_param('~locations_path')
        locations_path = '/home/jack/catkin_ws/src/fetch_data_center/config/makerspace_locations.csv'
        self.nav_goals = {}
        with open(locations_path, 'r') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                nav_goal = NavigationGoal()
                nav_goal.location_name = row['location_name']
                nav_goal.location_type = int(row['location_type'])
                nav_goal.pose.position.x = float(row['position_x'])
                nav_goal.pose.position.y = float(row['position_y'])
                nav_goal.pose.orientation.z = float(row['orientation_z'])
                nav_goal.pose.orientation.w = float(row['orientation_w'])
                self.nav_goals[row['location_name']] = nav_goal

    def user_input_status_cbk(self, msg):
        self.user_status = msg

    def manipulation_status_cbk(self, msg):
        self.man_status = msg

    def map_pose_cbk(self, msg):
        pose = ''
        pose += (str(msg.pose.pose.position.x) + ',')
        pose += (str(msg.pose.pose.position.y) + ',')
        pose += (str(msg.pose.pose.position.z) + ';')
        pose += (str(msg.pose.pose.orientation.x) + ',')
        pose += (str(msg.pose.pose.orientation.y) + ',')
        pose += (str(msg.pose.pose.orientation.z) + ',')
        pose += str(msg.pose.pose.orientation.w)
        self.pose = pose

    """
    def pan_tilt(self, rads):
        if self.init_rads == None:
            self.init_rads = rads
            return
        pitch = (rads[0] - self.init_rads[0]) % 2*pi
        yaw = (rads[1] - self.init_rads[1]) % 2*pi
        if pitch > pi:
            pitch = (2*pi - pitch) * -1
        if yaw > pi:
            yaw = (2*pi - yaw) * -1
        self.head.pan_tilt(yaw*-1, pitch)
    """

    def drive(self, dirs):
        x, y, z, th = 0.0, 0.0, 0.0, 0.0
        if dirs[1] > 0.75:
            x = 1
        elif dirs[1] < -0.75:
            x = 0
        if dirs[0] > 0.75:
            th = -1
        elif dirs[0] < -0.75:
            th = 1
        self.teleop.update(x, y, z, th, self.speed, self.turn)
    
    async def echo(self, websocket, path):
        try:
            async for msg in websocket:
                print(msg)
                rads = [(float(i) * pi)/180 for i in msg.split(';')[0][1:-1].split(',')]
                dirs = [float(i) for i in msg.split(';')[1][1:-1].split(',')]
                coords = [int(float(i)) for i in msg.split(';')[2][1:-1].split(',')]
                grabBool = bool(int(msg.split(';')[3]))
                homeBool = bool(int(msg.split(';')[4]))
                toolWallBool = bool(int(msg.split(';')[5]))
                tableBool = bool(int(msg.split(';')[6]))
                stopBool = bool(int(msg.split(';')[7]))
                # need to make bool for cam to move eyes
                # self.pan_tilt(rads)
                self.drive(dirs)
                if grabBool:
                    print('grab')
                    self.coords_pub.publish(Point(coords[0], coords[1], 0))
                if homeBool:
                    print('home')
                    self.nav_goal_pub.publish(self.nav_goals['Home'])
                if toolWallBool:
                    print('tool wall')
                    self.nav_goal_pub.publish(self.nav_goals['Tool Wall'])
                if tableBool:
                    print('table')
                    self.nav_goal_pub.publish(self.nav_goals['Table 1'])
                if stopBool:
                    print('stop')
                    self.nav_goal_pub.publish(NavigationGoal())
                await websocket.send(f"{self.user_status};{self.man_status};{self.pose}")
        except Exception as e:
            print(e)

if __name__ == '__main__':
    rospy.init_node('fetch_vr_websocket_server')
    while rospy.Time.now() == 0:
        pass
    ws_pt = Websocket_Pan_Tilt()
    start_server = websockets.serve(ws_pt.echo, "10.155.234.220", 8765)
    print('ready')
    # print(ws_pt.nav_goals['Home'])
    # ws_pt.nav_goal_pub.publish(ws_pt.nav_goals['Home'])
    asyncio.get_event_loop().run_until_complete(start_server)
    asyncio.get_event_loop().run_forever()

