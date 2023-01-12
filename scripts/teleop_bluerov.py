#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import String, Float64
from mavros_msgs.msg import OverrideRCIn
from pynput import keyboard
import os

class TeleopBluerov:
    def __init__(self) -> None:
        rospy.init_node('teleop_bluerov', anonymous=True)
        self.publisher = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
        self.sway_effort_sub = rospy.Subscriber("/sway/effort", Float64, self.sway_callback)
        self.surge_effort_sub = rospy.Subscriber("/surge/effort", Float64, self.surge_callback)
        self.heave_effort_sub = rospy.Subscriber("/heave/effort", Float64, self.heave_callback)

        self.key_pressed = None
        self.auto = False
        self.default_channels = [1500 , 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1100, 1100, 0,0,0,0,0,0,0,0]

        self.clear = lambda : os.system('tput reset')
        self.debug = True

    def on_press(self, key):
        try:
            self.key_pressed = key.char
            if(not self.auto):
                velocity = self.get_velocity(self.key_pressed)
                self.publish_velocity(velocity)
        except AttributeError:
            self.handle_control(key)
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

    def handle_control(self, key_pressed):
        if(key_pressed == keyboard.Key.esc):
            rospy.signal_shutdown("Esc key is pressed")
            quit()
        elif(key_pressed == keyboard.Key.tab):
            self.auto = not self.auto
            print(f"Autonous mode is {self.auto}")

    def get_velocity(self, key_pressed):
        channels = self.default_channels
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
        commands.channels= channels if channels is not None else self.default_channels
        self.publisher.publish(commands)

    def sway_callback(self, msg):
        if(not self.auto):
            return
        if(self.debug):
            print(f"Doing Sway with value {msg.data}")
        channels = self.default_channels
        channels[5] = -int(msg.data) + 1500
        self.publish_velocity(channels)

    def surge_callback(self, msg):
        if(not self.auto):
            return
        channels = self.default_channels
        channels[6] = int(msg.data) + 1500
        self.publish_velocity(channels)

    def heave_callback(self, msg):
        if(not self.auto):
            return
        if(self.debug):
            print(f"Doing Heave with value {msg.data}")
        channels = self.default_channels
        channels[4] = int(msg.data) + 1500
        self.publish_velocity(channels)
    
    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        teleop = TeleopBluerov()
        teleop.start_listener_async()
        teleop.spin()
        
    except rospy.ROSInterruptException:
        pass
