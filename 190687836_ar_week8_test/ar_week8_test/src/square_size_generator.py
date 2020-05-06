#!/usr/bin/env python
			#code by abdul gaffar abdul khadeer
import rospy
import random
from std_msgs.msg import Float32

def square_size_generator():
    pub = rospy.Publisher('square_size_generator1', Float32, queue_size=10) # to create a topic to publish the randomly genrated poitns on
    rospy.init_node('square_size_generator', anonymous=True)
    rate = rospy.Rate(0.05) #  to give a delay of 20 seconds 
    while not rospy.is_shutdown():
	# random point generation
       	squaresize=random.uniform(0.05,0.2)
        pub.publish(squaresize)
        rate.sleep()

if __name__ == '__main__':
    try:
        square_size_generator()
    except rospy.ROSInterruptException:
        pass
