#!/usr/bin/env python
from __future__ import print_function
import Queue
import roslib
import rospy
import numpy
import std_msgs.msg

from operator import attrgetter

from multi_tracker.msg import Trackedobject, Trackedobjectlist
from led_scheduler import LedScheduler
from path_integration.msg import PathIntegrationData


class PathIntegrationNode(object):

    def __init__(self,nodenum=1):

        self.nodenum = str(nodenum)
        rospy.init_node('path_integration_node')
        self.tracked_objects_sub = rospy.Subscriber('/multi_tracker/' + self.nodenum + '/tracked_objects', Trackedobjectlist, self.tracked_objects_callback)

        self.fly_queue = Queue.Queue()
        self.led_scheduler = LedScheduler()

	self.food_x = rospy.get_param('food_x', 100)           # food x coordinate, place holder checked
	self.food_y = rospy.get_param('food_y', 245 )          # food y coordinate, place holder checked
	self.food_width = rospy.get_param('food_width', 30)    # x threshhold, place holder 
	self.food_height = rospy.get_param('food_height', 30)  # y threshhold, place holder

        self.data_pub = rospy.Publisher('path_integration_data', PathIntegrationData, queue_size=10) 

    def tracked_objects_callback(self,data):
        number_of_objects = len(data.tracked_objects)
        if number_of_objects > 0: 
            fly = max(data.tracked_objects, key = attrgetter('size'))
            self.fly_queue.put(fly)

    def on_food_test(self,fly):
        test_x = abs(fly.position.x - self.food_x) < self.food_width/2.0 
        test_y = abs(fly.position.y - self.food_y) < self.food_height/2.0 
        return test_x and test_y

    def run(self):
        while not rospy.is_shutdown():
            while (self.fly_queue.qsize() > 0):
                fly = self.fly_queue.get()
                fly_on_food = self.on_food_test(fly) 
                self.led_scheduler.update(rospy.Time.now().to_time(), fly_on_food)
                
                header = std_msgs.msg.Header()
                header.stamp = rospy.Time.now()
                data = PathIntegrationData( 
                            header, 
                            fly, 
                            fly_on_food, 
                            self.led_scheduler.activation_count
                            )
                self.data_pub.publish(data)

            # Dummy update - so that scheduler keeps updating when there are no fly events
            self.led_scheduler.update(rospy.Time.now().to_time(), False)

        

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    node = PathIntegrationNode() 
    node.run()

