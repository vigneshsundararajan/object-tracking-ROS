#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel, SetModelState
from gazebo_msgs.msg import ModelState
from pynput import keyboard

def position_node():
    # Create a publisher object with Twist
    pub_model = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=1)
    # Declare the node, and register it with a unique name
    rospy.init_node('model_service_node', anonymous=True)
    # Define the execution rate object (10Hz)
    rate = rospy.Rate(10)

    # Create message object with a specific type
    state_msg = ModelState()
    state_msg.pose.position.x = 2
    state_msg.pose.position.y = 0
    key = 'q'
    '''
        This is the main node loop
    '''

    while not rospy.is_shutdown():
        with keyboard.Events() as events:
            event = events.get(1e6)
            if event.key == keyboard.KeyCode.from_char('s'):
                key = 's'
            elif event.key == keyboard.KeyCode.from_char('w'):
                key = 'w'
            elif event.key == keyboard.KeyCode.from_char('a'):
                key = 'a'
            elif event.key == keyboard.KeyCode.from_char('d'):
                key = 'd'

        state_msg.model_name = 'cricket_ball'
        rospy.wait_for_service('/gazebo/set_model_state')

        if key == 'w':
            state_msg.pose.position.y+=0.1
        elif key =='s':
            state_msg.pose.position.y-=0.1
        elif key == 'd':
            state_msg.pose.position.x += 0.1
        elif key == 'a':
            state_msg.pose.position.x -= 0.1

        #print(state_msg.pose.position.x)
        #state_msg.pose.position.x = 1
        #state_msg.pose.position.y = 1
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state(state_msg)

        except rospy.ServiceException:
            print("Service call failed: ")


        # Sleep the necessary amount of time to keep a 10Hz execution rate
        rate.sleep()

if __name__ == '__main__':
    try:
        position_node()
    except rospy.ROSInterruptException:
        pass