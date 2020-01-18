#!/usr/bin/python

"""
Copyright (c) 2020, Ubiquity Robotics
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of display_node nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

"""
Example client program for sending multiple move_basic commands
"""

import rospy

# our custom messages for the commands we will be using
import getopt, sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import actionlib
from   move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf

import traceback
import time



def printUsage():
    print """-h --help       - This help menu
    -c --continue   - Continuous operation, no prompts to continue
    -w --waypoints  - Select a waypoint list by name as below
                      line         A simple line
                      box          A square box
                      octagon      An octagon pattern
                      figure8      A figure 8 dual octagon
    -s --scale      - A different scale for the pattern 
    -x --offsetX    - An offset for X map placement of the pattern 
    -y --offsetY    - An offset for Y map placement of the pattern """


class Controller:

    #######################################################################################
    # Define lists of waypoints in terms of X,Y values in meters and robot angular yaw pose
    # The entries on each waypoint are as follows:
    #   1) X coordinate in the map in meters
    #   2) Y coordinate in the map in meters
    #   3) Angle in radians that the robot assumes once at X,Y
    #   4) A simple text comment

    # A simple line of one meter length
    figureLine = [
    [ 0.00,  0.00,  0.000, "MOVE: Leg A" ],
    [ 0.50,  0.00,  0.000, "MOVE: Leg B" ]
    ]

    # A box pattern
    figureBox = [
    [ 0.00,  0.00,  0.000, "MOVE: Leg A" ],
    [ 0.50,  0.00,  1.570, "MOVE: Leg B" ],
    [ 0.50,  0.50,  3.140, "MOVE: Leg C" ],
    [ 0.00,  0.50, -1.570, "MOVE: Leg D" ]
    ]


    # An octagon pattern which is a simplified way to do a circle type of pattern
    figureOctagon = [
    [ 0.40,  0.30,  0.000, "MOVE: Leg A" ],
    [ 0.60,  0.10, -0.785, "MOVE: Leg B" ],
    [ 0.60, -0.10, -1.571, "MOVE: Leg C" ],
    [ 0.40, -0.30, -2.356, "MOVE: Leg D" ],
    [ 0.20, -0.30, -3.141, "Move: Leg E" ],
    [ 0.00, -0.10,  2.356, "Move: Leg F" ],
    [ 0.00,  0.10,  1.571, "Move: Leg G" ],
    [ 0.20,  0.30,  0.785, "Move: Leg H" ] 
    ]

    # Two octagon connected on one edge for a figure 8
    figure8octagon = [
    [ 0.40,  0.30,  0.000, "MOVE: Leg A" ],
    [ 0.60,  0.10, -0.785, "MOVE: Leg B" ],
    [ 0.60, -0.10, -1.571, "MOVE: Leg C" ],
    [ 0.40, -0.30, -2.356, "MOVE: Leg D" ],
    [ 0.20, -0.30, -3.141, "Move: Leg E" ],
    [ 0.00, -0.10,  2.356, "Move: Leg F" ],
    [ 0.00,  0.10,  1.571, "Move: Leg G" ],
    [-0.20,  0.30,  2.356, "Move: Leg H" ],
    [-0.40,  0.30,  3.141, "Move: Leg I" ],
    [-0.60,  0.10, -2.356, "Move: Leg J" ],
    [-0.60, -0.10, -1.571, "Move: Leg K" ],
    [-0.40, -0.30, -0.785, "Move: Leg L" ],
    [-0.20, -0.30,  0.000, "Move: Leg M" ],
    [ 0.00, -0.10,  0.785, "Move: Leg N" ],
    [ 0.00,  0.10,  1.571, "Move: Leg O" ],
    [ 0.20,  0.30,  0.785, "Move: Leg P" ] 
    ]


    #######################################################################################

    """
    Constructor for our class
    """
    def __init__(self):

       rospy.init_node('controller')

       # Time per loop for the main control
       self.loop_msec = 50

       # Define waitAtEachVertex as 0 for continual moves or 1 for pause each vertex
       self.waitAtEachVertex = 1

       self.waypointName = 'line'
       self.scaleX  = 1.0
       self.scaleY  = 1.0
       self.offsetX = 0.0
       self.offsetY = 0.0

       # read commandline arguments and place them in array
       # Only the -w option seems to work, something is wrong with getopt usage here
       try:
           opts, args = getopt.getopt(sys.argv[1:], 'hcw:s:x:y:h', \
               ['help','continue', 'waypoints=','scale=','offsetX=','offsetY='])
       except getopt.GetoptError as err:
           # will print something like "option -a not recognized"
           print "Error in recognized options"  
           printUsage()
           sys.exit(2)

       for o, a in opts:
           # evaluate given options
           if o in ("-h", "--help"):
               print ("displaying help")
               printUsage()
               sys.exit(2)
           elif o in ("-c", "--continue"):
               # continuous operation, do not require ENTER at each vertex
               self.waitAtEachVertex = 0
           elif o in ("-w", "--waypoints"):
               # define a waypoint list for waypoints
               waypointName = a
               if (a == "line"):
                   waypointList = self.figureLine
               elif (a == "box"):
                   waypointList = self.figureBox
               elif (a == "figure8"):
                   waypointList = self.figure8octagon
               elif (a == "octagon"):
                   waypointList = self.figureOctagon
               else:
                   print ("Invalid choice of waypoint list")
                   sys.exit(2)
               print "Waypoint list will be '%s'", waypointList
           elif o in ("-s", "--scaleX"):
               self.scaleX = float(a)
               self.scaleY = float(a)
           elif o in ("-x", "--offsetX"):
               self.offsetX = float(a)
           elif o in ("-y", "--offsetY"):
               self.offsetY = float(a)

       print ("WaypointsName: %s scaleX %f scaleY %f offsetX %f offsetY %f" \
             %(self.waypointName,self.scaleX,self.scaleY,self.offsetX,self.offsetY))

       print "A total of %d waypoints is in the list " % (len(waypointList))


    # A publisher for sending commands to the follower node
    # self.moveBaseGoalPub = rospy.Publisher("/move_base/goal", \
    #     move_base_msgs/MoveBaseActionGoal, queue_size=1)

    # Publish the desired waypoint to navigate on top of now
    # For fiducial nav use frame_id of  "map"  
    # For odom nav use frame_id of "odom"
    def publishMoveBaseGoalWaitForReply(self, x, y, yaw, comment):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"     
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x 
        goal.target_pose.pose.position.y = y 

        # to send orientation with a yaw we need quaternion transform
        x , y, z, w = tf.transformations.quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation.x = x
        goal.target_pose.pose.orientation.y = y
        goal.target_pose.pose.orientation.z = z
        goal.target_pose.pose.orientation.w = w
        now = rospy.get_rostime()
        print "[%i.%i]  PubMove:  %s x,y,z,w of %f %f %f %f yaw %f" \
              % (now.secs,now.nsecs,comment,x,y,z,w,yaw)

        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        # publish the goal to the topic
        client.send_goal(goal)

        now = rospy.get_rostime()
        print "[%i.%i]  Waiting for result ..." % (now.secs, now.nsecs)
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            now = rospy.get_rostime()
            print "[%i.%i]  Received result" % (now.secs, now.nsecs)
            return client.get_result()

    """
    Main loop
    """
    def run(self):

       print "ROS publisher publishing goals to move basic"
       waypointList = self.figureLine

       # continue going through waypoints over and over.
       # If you only want to do list once exit after first for loop
       while (True):
           for waypoint in waypointList:
               x,y,yaw,comment = waypoint
               x = (x * self.scaleX) + self.offsetX
               y = (y * self.scaleY) + self.offsetY
               now = rospy.get_rostime()
               print "[%i.%i]  Waypoint: %s X %f Y %f yaw %f  will be published now" \
                     % (now.secs,now.nsecs,comment,x, y, yaw)

               # now publish the waypoint
               moveResult = self.publishMoveBaseGoalWaitForReply( x,  y, yaw, comment)
               if moveResult == True:
                   print "ERROR: MoveBasic Bad Status! for Waypoint: X %f Y %f yaw %f " % (x, y, yaw)
               else:
                   print "[%i.%i]  Waypoint: %s X %f Y %f yaw %f  has been reached" \
                         % (now.secs,now.nsecs,comment,x, y, yaw)

               # optionally wait at each vertex before going to next one
               if (self.waitAtEachVertex == 1):
                   raw_input("Hit ENTER to go to next waypoint ... ")

if __name__ == "__main__":
    # Create an instance of our goal class
    node = Controller()
    # run it
    node.run()
