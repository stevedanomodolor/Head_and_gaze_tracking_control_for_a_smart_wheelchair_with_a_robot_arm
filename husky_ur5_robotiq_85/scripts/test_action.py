#! /usr/bin/env python3
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
# from control_msgs.msg import GripperCommandAction
import control_msgs.msg
def gripper_action():
#     # Creates the SimpleActionClient, passing the type of the action
#     # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('/gripper/gripper_cmd', control_msgs.msg.GripperCommandAction)
#
#     # Waits until the action server has started up and started
#     # listening for goals.
    client.wait_for_server()
    goal = control_msgs.msg.GripperCommandGoal()
    goal.command.position = 0.8
    goal.command.max_effort = -1
    client.send_goal(goal)
#
#     # Creates a goal to send to the action server.
#     goal = actionlib_tutorials.msg.FibonacciGoal(order=20)
#
#     # Sends the goal to the action server.
#     client.send_goal(goal)
#
#     # Waits for the server to finish performing the action.
    client.wait_for_result()
#
#     # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('gripper_action')
        result = gripper_action()
        print(str(result))
        # result = fibonacci_client()
        # print("Result:", ', '.join([str(n) for n in result.sequence]))
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
    print("testing complete")
