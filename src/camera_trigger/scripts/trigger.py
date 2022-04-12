#!/usr/bin/env python
#im sorry mr mortensen
from pynput import keyboard
import rospy
from read_camera.msg import Parcel #Parcel msg

saved_data = None

def on_press(key):

    try:
        print('special key pressed: {0}'.format(key))
        #print("key: ", key)
        #if key er enter
        if key == key.enter:
            saved_data = rospy.wait_for_message("/vision/parcel_raw", Parcel, timeout=None)
            print("saved data: ", saved_data)
            #publish here to another topic
            pub.publish(saved_data)
        elif key == key.space:
            saved_data = rospy.wait_for_message("/vision/parcel_raw", Parcel, timeout=None)
            print("saved data: ", saved_data)
            pub.publish(saved_data)
    except AttributeError:
        print("error :((((")

def on_release(key):
        return False


if __name__ == '__main__':
    rospy.init_node('trigger_node', anonymous=True)
    #define pub her
    pub = rospy.Publisher('/vision/frame_acq/parcel_info', Parcel, queue_size=10)
    print("hi jepper")
    while not rospy.is_shutdown():
        with keyboard.Listener(
            on_press=on_press,
            on_release=on_release) as listener:
            listener.join()