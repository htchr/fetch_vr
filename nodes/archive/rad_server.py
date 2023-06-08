#!/home/jack/catkin_ws/src/fetch_vr/venv/bin/python3

import asyncio
import websockets
import rospy
import robot_api
from math import pi

class Websocket_Pan_Tilt():
    def __init__(self):
        self.head = robot_api.Head()
        self.init_rads = None
        self.tilt = 0
        self.pan = 0
        self.step = 0.1
        self.head.pan_tilt(0, 0) # reset head

    def pan_tilt(self, rads):
        if self.init_rads == None:
            self.init_rads = rads
            return
        pitch = rads[0] - self.init_rads[0] - self.tilt
        yaw = rads[1] - self.init_rads[1] - self.pan
        if pitch < pi and abs(pitch - self.tilt) > self.step:
            self.tilt += self.step
        elif abs(pitch - self.tilt) > self.step:
            self.tilt -= self.step
        if yaw < pi and abs(yaw - self.pan) > self.step:
            self.pan += self.step
        elif abs(yaw - self.pan) > self.step:
            self.pan -= self.step
        self.head.pan_tilt(self.pan, self.tilt)

    async def echo(self, websocket, path):
        try:
            async for msg in websocket:
                rads = [(float(i) * pi)/180 for i in msg[1:-1].split(',')]
                print(rads)
                self.pan_tilt(rads)
                await websocket.send(f"Echo: {msg}")
        except Exception as e:
            print(e)

if __name__ == '__main__':
    rospy.init_node('hmd_pan_tilt_demo')
    while rospy.Time.now() == 0:
        pass
    ws_pt = Websocket_Pan_Tilt()
    rospy.sleep(3)
    start_server = websockets.serve(ws_pt.echo, "10.155.234.220", 8765)
    asyncio.get_event_loop().run_until_complete(start_server)
    asyncio.get_event_loop().run_forever()

