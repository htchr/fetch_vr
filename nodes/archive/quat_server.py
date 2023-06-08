#!/home/jack/catkin_ws/src/fetch_vr/venv/bin/python3

import asyncio
import websockets
import rospy
import robot_api
import tf.transformations as tf_trans

rospy.init_node('hmd_pan_tilt_demo')
head = robot_api.Head()
initQ = None
pan = 0
tilt = 0

def pan_tilt(robo_head, q1, q2):
    """extract yaw and pitch from 2 quaternions
    move robot head to match
    robo_head: head class
    q1: normal direction
    q2: current direction
    returns: None"""
    global pan
    global tilt
    e1 = tf_trans.euler_from_quaternion(q1)
    e2 = tf_trans.euler_from_quaternion(q2)
    yaw = e2[2] - e1[2]
    pitch = e2[1] - e1[1]
    if yaw > pan:
        pan += 0.1
    elif yaw < pan:
        pan -= 0.1
    if pitch > tilt:
        tilt += 0.1
    elif pitch < tilt:
        tilt -= 0.1
    print(f'yaw: {yaw}, pitch: {pitch}')
    print(f'pan: {pan}, tilt: {tilt}')
    robo_head.pan_tilt(pan, tilt)

async def echo(websocket, path):
    global head
    global initQ
    try:
        async for msg in websocket:
            q = [float(s) for s in msg[1:-1].split(',')]
            if initQ == None:
                initQ = q
                print('init')
            else:
                pan_tilt(head, initQ, q)
            print(f"Received message: {msg}")
            await websocket.send(f"Echo: {msg}")
    except Exception as e:
        print(e)

start_server = websockets.serve(echo, "10.155.234.220", 8765)
asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()

