#!/usr/bin/env python
import rospy
import numpy as np
import math
from duckietown_msgs.msg import  Twist2DStamped, LanePose, BoolStamped # ADDED (ONLY BOOLSTAMPED ADDED)
from geometry_msgs.msg import PoseStamped # ADDED
import tf # ADDED for quaternion to euler transform

class lane_controller(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.lane_reading = None

        self.pub_counter = 0

        # Setup parameters
        self.setGains()
        self.isReachedSafeDistance = False

        # Publicaiton
        self.pub_car_cmd = rospy.Publisher("~car_cmd",Twist2DStamped,queue_size=1)

        # Subscriptions
        self.sub_lane_reading = rospy.Subscriber("~lane_pose", LanePose, self.cbPose, queue_size=1)
        # self.stop_line = rospy.Subscriber("stop_line_filter_node/at_stop_line", BoolStamped, self.at_stop, queue_size=1) #ADDED
        self.stop_tag = rospy.Subscriber("/duckiepark/aruco_single/pose", PoseStamped,self.at_stop_tag, queue_size=1) #ADDED
        self.controlled_parking = None
        
        # safe shutdown
        rospy.on_shutdown(self.custom_shutdown)

        # timer
        self.gains_timer = rospy.Timer(rospy.Duration.from_sec(1.0), self.getGains_event)
        rospy.loginfo("[%s] Initialized " %(rospy.get_name()))

 ##########################################################
    #ADDED
    def at_stop(self,msg):
        car_control_msg = Twist2DStamped()
        if (msg.data==True):
            # Stop listening
            self.sub_lane_reading.unregister()
            self.v_bar = 0.0
            car_control_msg.header = msg.header
            car_control_msg.v = 0.0
            car_control_msg.omega=0.0
            self.publishCmd(car_control_msg)
            rospy.sleep(0.5) #To make sure that it gets published.      

    def at_stop_tag(self,msg):
        car_control_msg = Twist2DStamped()
        
        stop_distance = 0.25 # Stop distance for parking
        
        if (msg.pose.position.z < stop_distance):
        
            # Stop listening
            self.sub_lane_reading.unregister()
            
            # Stop the vehicle
            car_control_msg.header = msg.header
            car_control_msg.v = 0.0
            car_control_msg.omega=0.0
            self.pub_car_cmd.publish(car_control_msg)
            rospy.sleep(2) # Wait 2 seconds
            self.stop_tag.unregister() #Unregister from this subscription
            self.controlled_parking = rospy.Subscriber("/duckiepark/aruco_single/pose", PoseStamped,self.control_parking, queue_size=1) # Subscribe again to pose and go to control_parking function to control parking
            
    def control_parking(self,msg):
        car_control_msg = Twist2DStamped() # Required for sending commands to vehicle
        car_control_msg.header = msg.header
        
        #Reverse duckie and go to a further distance for safe parking
        safe_distance = 0.35 # Safe distance to start parking
        
        if(msg.pose.position.z < safe_distance and self.isReachedSafeDistance == False):
            car_control_msg.v = -(self.v_bar)/2
            self.pub_car_cmd.publish(car_control_msg)
        elif(self.isReachedSafeDistance == False):             
            # Stop when reached to the safe distance
            self.isReachedSafeDistance = True
            car_control_msg.v=0.0
            car_control_msg.omega=0.0
            self.pub_car_cmd.publish(car_control_msg)
            rospy.sleep(2) # Wait 2 seconds
        elif(self.isReachedSafeDistance == True):    
            #__Start to control__
            # Set the control gains
            k_rho = 0.5 # set (k_rho > 0) for stability
            k_alpha = 0.55 # set (k_alpha > k_rho) for stability
            k_beta = -0.1 # set (k_beta < 0) for stability
            park_distance = 0.15 # distance to tag for parking 
            
            # Take the measurements
            quaternion = (msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            theta = euler[1]
            
            deltaX = (msg.pose.position.z * math.sin(theta)) - (msg.pose.position.x * math.cos(theta))
            deltaY = -(msg.pose.position.z * math.cos(theta)) - (msg.pose.position.x * math.sin(theta)) + park_distance
            
            # Calculate polar coordinate representations(rho, alpha, beta)
            rho = math.sqrt(deltaX**2 + deltaY**2) 
            alpha = - (math.pi/2.0) - theta + math.atan2(deltaY, deltaX) # in radians
            beta = - (math.pi/2.0) - theta - alpha
            
            # Set the velocity and angular velocity
            if ((rho < 0.1) and (-0.088 < alpha < 0.088) and (-0.088 < beta < 0.088)): 
                # 0.088 rad=5 degrees
                # Parking is done
                car_control_msg.v=0.0
                car_control_msg.omega=0.0
                self.pub_car_cmd.publish(car_control_msg)
            else:
                car_control_msg.v= k_rho*rho*
                car_control_msg.omega= (k_alpha*alpha + k_beta*beta)
                self.pub_car_cmd.publish(car_control_msg)
                

          
        ############################################################

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def setGains(self):
        v_bar = 0.5 # nominal speed, 0.5m/s
        k_theta = -2.0
        k_d = - (k_theta ** 2) / ( 4.0 * v_bar)
        theta_thres = math.pi / 6
        d_thres = math.fabs(k_theta / k_d) * theta_thres
        d_offset = 0.0

        self.v_bar = self.setupParameter("~v_bar",v_bar) # Linear velocity
        self.k_d = self.setupParameter("~k_d",k_theta) # P gain for theta
        self.k_theta = self.setupParameter("~k_theta",k_d) # P gain for d
        self.d_thres = self.setupParameter("~d_thres",theta_thres) # Cap for error in d
        self.theta_thres = self.setupParameter("~theta_thres",d_thres) # Maximum desire theta
        self.d_offset = self.setupParameter("~d_offset",d_offset) # a configurable offset from the lane position

    def getGains_event(self, event):
        v_bar = rospy.get_param("~v_bar")
        k_d = rospy.get_param("~k_d")
        k_theta = rospy.get_param("~k_theta")
        d_thres = rospy.get_param("~d_thres")
        theta_thres = rospy.get_param("~theta_thres")
        theta_thres = rospy.get_param("~theta_thres")
        d_offset = rospy.get_param("~d_offset")

        params_old = (self.v_bar,self.k_d,self.k_theta,self.d_thres,self.theta_thres, self.d_offset)
        params_new = (v_bar,k_d,k_theta,d_thres,theta_thres, d_offset)

        if params_old != params_new:
            rospy.loginfo("[%s] Gains changed." %(self.node_name))
            rospy.loginfo("old gains, v_var %f, k_d %f, k_theta %f, theta_thres %f, d_thres %f, d_offset %f" %(params_old))
            rospy.loginfo("new gains, v_var %f, k_d %f, k_theta %f, theta_thres %f, d_thres %f, d_offset %f" %(params_new))
            self.v_bar = v_bar
            self.k_d = k_d
            self.k_theta = k_theta
            self.d_thres = d_thres
            self.theta_thres = theta_thres
            self.d_offset = d_offset

    
    def custom_shutdown(self):
        rospy.loginfo("[%s] Shutting down..." %self.node_name)
        
        # Stop listening
        self.sub_lane_reading.unregister()

        # Send stop command
        car_control_msg = Twist2DStamped()
        car_control_msg.v = 0.0
        car_control_msg.omega = 0.0
        self.publishCmd(car_control_msg)
        rospy.sleep(0.5) #To make sure that it gets published.
        rospy.loginfo("[%s] Shutdown" %self.node_name)


    def publishCmd(self,car_cmd_msg):

        #wheels_cmd_msg = WheelsCmdStamped()
        #wheels_cmd_msg.header.stamp = stamp
        #speed_gain = 1.0
        #steer_gain = 0.5
        #vel_left = (speed_gain*speed - steer_gain*steering)
        #vel_right = (speed_gain*speed + steer_gain*steering)
        #wheels_cmd_msg.vel_left = np.clip(vel_left,-1.0,1.0)
        #wheels_cmd_msg.vel_right = np.clip(vel_right,-1.0,1.0)

        self.pub_car_cmd.publish(car_cmd_msg)
        #self.pub_wheels_cmd.publish(wheels_cmd_msg)

    def cbPose(self,lane_pose_msg):
        self.lane_reading = lane_pose_msg 

        cross_track_err = lane_pose_msg.d - self.d_offset
        heading_err = lane_pose_msg.phi

        car_control_msg = Twist2DStamped()
        car_control_msg.header = lane_pose_msg.header
        car_control_msg.v = self.v_bar/2.0 #*self.speed_gain #Left stick V-axis. Up is positive
        
        if math.fabs(cross_track_err) > self.d_thres:
            cross_track_err = cross_track_err / math.fabs(cross_track_err) * self.d_thres

        if (car_control_msg.v == 0.0): #ADDED
            car_control_msg.omega = 0.0 #ADDED
        else: # ADDED (dont forget to indent back the next line)
            car_control_msg.omega = (self.k_d * cross_track_err + self.k_theta * heading_err)/1.2 #*self.steer_gain #Right stick H-axis. Right is negative
        
        # controller mapping issue
        # car_control_msg.steering = -car_control_msg.steering
        # print "controls: speed %f, steering %f" % (car_control_msg.speed, car_control_msg.steering)
        # self.pub_.publish(car_control_msg)
        self.publishCmd(car_control_msg)

        # debuging
        # self.pub_counter += 1
        # if self.pub_counter % 50 == 0:
        #     self.pub_counter = 1
        #     print "lane_controller publish"
        #     print car_control_msg

if __name__ == "__main__":
    rospy.init_node("lane_controller",anonymous=False)
    lane_control_node = lane_controller()
    rospy.spin()
