#!/usr/bin/env python3

import cherrypy
import json
import os
import re
import numpy
import subprocess
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import thread
from time import time

class DictionaryController(object):
    state = 'stopped'
    # this is a controller class, which holds event handlers
    # constructor
    def __init__(self, pub, tbot):
        self.myd = dict()
        self.pub = pub
        self.tbot = tbot

    def get_value(self, key):
        return self.myd[key]

    def add_entry(self, key, value):
        self.myd[key] = value
        
    def delete_entry(self, key):
        del self.myd[key]

    # def read_pgm(self, pgmf):
    #     """Return a raster of integers from a PGM as a list of lists."""
    #     assert pgmf.readline() == 'P5\n'
    #     comment = pgmf.readline()
    #     (width, height) = [int(i) for i in pgmf.readline().split()]
    #     depth = int(pgmf.readline())
    #     assert depth <= 255

    #     raster = []
    #     for y in range(height):
    #         row = []
    #         for y in range(width):
    #             row.append(ord(pgmf.read(1)))
    #         raster.append(row)
    #     return raster

    def read_pgm(self, filename, byteorder='>'):
        """Return image data from a raw PGM file as numpy array.

        Format specification: http://netpbm.sourceforge.net/doc/pgm.html

        """
        with open(filename, 'rb') as f:
            buffer = f.read()
        try:
            header, width, height, maxval = re.search(
                b"(^P5\s(?:\s*#.*[\r\n])*"
                b"(\d+)\s(?:\s*#.*[\r\n])*"
                b"(\d+)\s(?:\s*#.*[\r\n])*"
                b"(\d+)\s(?:\s*#.*[\r\n]\s)*)", buffer).groups()
        except AttributeError:
            raise ValueError("Not a raw PGM file: '%s'" % filename)
        return numpy.frombuffer(buffer,
                                dtype='u1' if int(maxval) < 256 else byteorder+'u2',
                                count=int(width)*int(height),
                                offset=len(header)
                                ).reshape((int(height), int(width)))

    # event handlers for resource requests
    def GET_MAP(self):
        output = {'result' : 'success'}
        if self.state == 'running':
            try:
                os.system('rosrun map_server map_saver -f ~/map')
                pass
            except Error as err:
                output['message'] = str(err)

            try:
                output['message'] = {}
                output['message']['map'] = {}
                output['message']['map']['yaml'] = []
                output['message']['map']['pgm'] = []
                image_path = None

                with open('/home/gazebo/map.yaml', 'r') as f:
                    for line in f.readlines():
                        output['message']['map']['yaml'].append(line.strip())
                        if line.strip():
                            key, value = line.strip().split(': ')
                            if key == 'image':
                                image_path = value

                output['message']['image_path'] = image_path

                if image_path:
                    im = self.read_pgm(image_path, byteorder='<')
                    output['message']['map']['pgm'] = im.tolist()

                    
            except IOError as err:
                output['message'] = str(err)

        return json.dumps(output)
    
    def GET_TEST_CONNECT(self):
        output = {'message': 'success'}

        return json.dumps(output)

    def GET_CURRENT_POSITION(self):
        output = {'message': 'success'}

        with open('/home/gazebo/tf.txt') as f:
            lines = f.readlines()
            latest_time = 0
            latest_time_index = 0
            for i, line in enumerate(lines):
                if "At time" in line:
                    time = float(line.split(" ")[-1])
                    if time > latest_time:
                        latest_time = time
                        latest_time_index = i
            
            wanted_line = lines[latest_time_index + 1]
            translation = wanted_line.split(': [')[-1]
            x = float(translation.split(', ')[0])
            y = float(translation.split(', ')[1])

        output['data'] = {'x': x, 'y': y}
        
        return json.dumps(output)


    def makeSimpleProfile(self, output, input, slop):
        if input > output:
            output = min( input, output + slop )
        elif input < output:
            output = max( input, output - slop )
        else:
            output = input

        return output

    def STOP(self):
        output = {'message': 'success'}
        try:            

            os.system('pkill -9 -f teleop')
            os.system('pkill -9 -f slam')
            os.system('pkill -9 -f base_link')

            target_linear_vel   = 0.0
            control_linear_vel  = 0.0
            target_angular_vel  = 0.0
            control_angular_vel = 0.0

            LIN_VEL_STEP_SIZE = 0.01
            ANG_VEL_STEP_SIZE = 0.1

            twist = Twist()

            control_linear_vel = self.makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
            twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

            control_angular_vel = self.makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

            self.pub.publish(twist)
        except IOError as err:
            output['message'] = str(err)
        self.state = 'stopped'
        return json.dumps(output)

    def START(self):
        output = {'message': 'success'}
        try:
            thread.start_new_thread(os.system, ('roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch',))
            thread.start_new_thread(os.system, ('roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping',))
            thread.start_new_thread(os.system, ('rosrun tf tf_echo /map /base_link > /home/gazebo/tf.txt',))
        except IOError as err:
            output['message'] = str(err)
        self.state = 'running'
        return json.dumps(output)