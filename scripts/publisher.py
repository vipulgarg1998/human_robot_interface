#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import String
from mavros_msgs.msg import OverrideRCIn
from pynput import keyboard

# import keyboard

rospy.init_node('publisher', anonymous=True)
commands_topic = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)

key_pressed = None


def on_press(key):
    global key_pressed
    try:
        # print('alphanumeric key {0} pressed'.format(key.char))
        key_pressed = key.char


    except AttributeError:
        print('special key {0} pressed'.format(key))

def on_release(key):
    # print('{0} released'.format(
    #     key))
    global key_pressed
    key_pressed = None
    # if key == keyboard.Key.esc:
    #     # Stop listener
    #     key_pressed = None
    #     # return False

def start_listener():
    listener = keyboard.Listener(
        on_press=on_press,
        on_release=on_release)
    listener.start()

def talker():
    rate = rospy.Rate(100) # 1hz
    global key_pressed, commands_topic
    i = 0
    while not rospy.is_shutdown():
        commands= OverrideRCIn()
        # print("FFFFFFFFFFFFFFFFFFF")
        commands.channels=[1500 , 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 0,0,0,0,0,0,0,0]
        # print(key_pressed)

        if key_pressed == 'w':
            commands.channels[2] = 1700
            # print("FFFFFFFFFFFFFFFFFFF")
        elif(key_pressed == 'z'):
            commands.channels[2] = 1300
        # if(key_pressed is not None):
        #     if key_pressed == 'w':
        #         print("FFFFFFFFFFFFFFFFFFF") 
        #     commands.channels[2] = 1600
        # elif keyboard.is_pressed('z'): 
        #     commands.channels[2] = 1400
        
        commands_topic.publish(commands)
        # print("I am publishing")
        rate.sleep()

if __name__ == '__main__':
    try:
        start_listener()
        talker()
    except rospy.ROSInterruptException:
        pass
