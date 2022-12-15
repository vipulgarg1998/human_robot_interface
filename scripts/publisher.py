#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import String
from mavros_msgs.msg import OverrideRCIn
from pynput import keyboard

class TeleopBluerov:
    def __init__(self) -> None:
        rospy.init_node('teleop_bluerov', anonymous=True)
        self.publisher = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)

        self.key_pressed = None

    def on_press(self, key):
        try:
            self.key_pressed = key.char
            velocity = self.get_velocity(self.key_pressed)
            self.publish_velocity(velocity)
        except AttributeError:
            print('special key {0} pressed'.format(key))

    def on_release(self, key):
        self.key_pressed = None
        self.publish_velocity()

    def start_listener_async(self):
        listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        listener.start()

    def start_listener_sync(self):
        with keyboard.Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            listener.join()

    def get_velocity(self, key_pressed):
        channels = [1500 , 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 0,0,0,0,0,0,0,0]
        if key_pressed == 'q':
            channels[2] = 1600
        elif(key_pressed == 'z'):
            channels[2] = 1400
        elif(key_pressed == 'w'):
            channels[4] = 1600
        elif(key_pressed == 's'):
            channels[4] = 1400
        elif(key_pressed == 'a'):
            channels[5] = 1400
        elif(key_pressed == 'd'):
            channels[2] = 1600
        elif(key_pressed == 'j'):
            channels[3] = 1400
        elif(key_pressed == 'l'):
            channels[3] = 1600
        
        return channels

    def publish_velocity(self, channels = None):
        commands = OverrideRCIn()
        commands.channels= channels if channels is not None else [1500 , 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1100, 1100, 0,0,0,0,0,0,0,0]
        self.publisher.publish(commands)

if __name__ == '__main__':
    try:
        teleop = TeleopBluerov()
        teleop.start_listener_sync()
    except rospy.ROSInterruptException:
        pass
