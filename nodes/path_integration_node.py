#!/usr/bin/env python
from __future__ import print_function
import Queue
import roslib
import rospy
import numpy
import std_msgs.msg
import cv2

from operator import attrgetter
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from multi_tracker.msg import Trackedobject, Trackedobjectlist
from led_scheduler import LedScheduler
from path_integration.msg import PathIntegrationData


class PathIntegrationNode(object):

    def __init__(self,nodenum=1):

        self.nodenum = str(nodenum)
        rospy.init_node('path_integration_node')
        self.tracked_objects_sub = rospy.Subscriber('/multi_tracker/' + self.nodenum + '/tracked_objects', Trackedobjectlist, self.tracked_objects_callback)

        self.fly_queue = Queue.Queue()

        self.led_pins = rospy.get_param('led_pins', [3,10])
        self.food_positions = rospy.get_param('food_positions', [(140,76),(224,163)])
        self.scheduler_base_params = rospy.get_param(
                'scheduler_base_params', 
                { 
                    'on_value' : 100, 
                    'off_value' : 0, 
                    'on_duration' : 1.0, 
                    'minimum_off_duration': 9.0, 
                    }
                )
	self.food_width = rospy.get_param('food_width', 10)    # x threshhold, place holder 
	self.food_height = rospy.get_param('food_height', 10)  # y threshhold, place holder


        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/image_mono', Image, self.image_callback)
        
        self.led_schedulers = []
        for pin in self.led_pins:
            params = dict(self.scheduler_base_params)
            params['pin'] = pin
            self.led_schedulers.append(LedScheduler(params))


        self.data_pub = rospy.Publisher('path_integration_data', PathIntegrationData, queue_size=10) 

    def image_callback(self,imgmsg):
        cv_image = self.bridge.imgmsg_to_cv2(imgmsg,desired_encoding='mono8')
        cv_image_bgr = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)

        for pos in self.food_positions:
            cx, cy = pos
            x0 = int(cx - self.food_width/2.0)
            y0 = int(cy - self.food_height/2.0)
            x1 = int(cx + self.food_width/2.0)
            y1 = int(cy + self.food_height/2.0)
            cv2.rectangle(cv_image_bgr, (x0,y0), (x1,y1), (0,0,255), 1)
            
        cv2.imshow('roi image',cv_image_bgr)
        cv2.waitKey(1)


    def tracked_objects_callback(self,data):
        number_of_objects = len(data.tracked_objects)
        if number_of_objects > 0: 
            fly = max(data.tracked_objects, key = attrgetter('size'))
            self.fly_queue.put(fly)

    def on_food_test(self,fly,food_position):
        food_x, food_y = food_position
        test_x = abs(fly.position.x - food_x) < self.food_width/2.0 
        test_y = abs(fly.position.y - food_y) < self.food_height/2.0 
        return test_x and test_y

    def run(self):
        while not rospy.is_shutdown():
            while (self.fly_queue.qsize() > 0):
                fly = self.fly_queue.get()
                for scheduler, food_position in zip(self.led_schedulers, self.food_positions):
                    fly_on_food = self.on_food_test(fly,food_position)
                    #print(fly_on_food, fly.position.x, fly.position.y, food_position)
                    scheduler.update(rospy.Time.now().to_time(), fly_on_food)

                    header = std_msgs.msg.Header()
                    header.stamp = rospy.Time.now()
                    data = PathIntegrationData( 
                                header, 
                                fly, 
                                fly_on_food,
                                food_position,
                                scheduler.led_pin, 
                                scheduler.activation_count
                                )
                    self.data_pub.publish(data)

            # Dummy update - so that scheduler keeps updating when there are no fly events
            for scheduler in self.led_schedulers:
                scheduler.update(rospy.Time.now().to_time(), False)
        

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    node = PathIntegrationNode() 
    node.run()
