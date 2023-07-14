#! /usr/bin/env python3

import rospy, onnxruntime, random, os, rospkg, onnx
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

current_state = State()
current_pos = PoseStamped()
current_vel = TwistStamped()
counter = 0
scale = 0.2
z_offset = 0.1
rospack = rospkg.RosPack()
onnxPath = rospack.get_path('offboard_py') + '/scripts/DroneLanding-8078831.onnx'

onnx_model = onnx.load(onnxPath)
onnx.checker.check_model(onnx_model)
model = onnxruntime.InferenceSession(onnxPath)

def state_cb(msg):
    global current_state
    current_state = msg

def pos_cb(msg):
    global current_pos
    current_pos = msg

def vel_cb(msg):
    global current_vel
    current_vel = msg

def get_state():
    state = []
    state.append(current_vel.twist.linear.y)
    state.append(current_vel.twist.linear.z + z_offset)
    state.append(current_vel.twist.linear.x)
    state.append(current_pos.pose.position.x)
    state.append(current_pos.pose.position.z)
    state.append(current_pos.pose.position.y)
    return state

def action():
    state = get_state()
    ort_inputs = {model.get_inputs()[0].name: [state]}
    action = model.run(None, ort_inputs)
    action = np.multiply(action[2][0], [2, 2, 2])
    action = action * scale
    twist = Twist()
    twist.linear.x = action[1]
    twist.linear.y = action[0]
    twist.linear.z = action[2]
    rospy.loginfo(action)
    local_vel_pub.publish(twist)
    
    

if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    local_pos_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback = pos_cb)
    local_vel_sub = rospy.Subscriber("mavros/local_position/velocity_local", TwistStamped, callback = vel_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    local_vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    rospy.loginfo("arming")

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    rospy.loginfo("setmode")


    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(40)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()
    pos = (-random.random()*4, random.random()*4, random.random()*2 + 4)
    pose.pose.position.x = pos[0]
    pose.pose.position.y = pos[1]
    pose.pose.position.z = pos[2]
    rospy.loginfo(f"Takeoff to x:{pos[0]:.3f}, y:{pos[1]:.3f}, z:{pos[2]:.3f}")

    # Send a few setpoints before starting
    for i in range(20):
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")

                last_req = rospy.Time.now()

        if current_state.mode == 'OFFBOARD' and counter < 800:
            counter += 1
            rospy.loginfo(counter)

        if counter < 800:
            local_pos_pub.publish(pose)
        else:
            # inference
            if current_state.mode == 'OFFBOARD':
                if current_pos.pose.position.z < 0.1:
                    offb_set_mode.custom_mode = 'AUTO.LAND'
                    if(set_mode_client.call(offb_set_mode).mode_sent == True):
                        rospy.loginfo("Land Complete")
                        break
                else:
                    action()

        rate.sleep()

