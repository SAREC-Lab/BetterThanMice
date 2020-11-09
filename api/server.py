#!/usr/bin/env python3

import cherrypy
import routes
from dictionary_controller import DictionaryController # getting our controller class 
from sys import argv
import rospy
from geometry_msgs.msg import Twist

def start_service(ip_address, port_number, pub):
    '''configures and runs the server'''
    dCon = DictionaryController(pub) # object

    dispatcher = cherrypy.dispatch.RoutesDispatcher() # dispatcher object
    # we will use this to connect endpoints to controllers

    # connecting endpoints to resources
    #connect(out_tag, http resource, class object with handler, event handler name, what type of HTTP request to serve)
    dispatcher.connect('dict_get_map', '/map/', controller=dCon, action='GET_MAP', conditions=dict(method=['GET']))
    dispatcher.connect('dict_get_test_connect', '/', controller=dCon, action='GET_TEST_CONNECT', conditions=dict(method=['GET']))
    dispatcher.connect('dict_post_stop', '/stop/', controller=dCon, action='STOP', conditions=dict(method=['POST']))
    dispatcher.connect('dict_post_stop', '/start/', controller=dCon, action='START', conditions=dict(method=['POST']))
    
    # configuration for the server
    conf = {
            'global' : {
                'server.socket_host' : ip_address, # can use 'localhost' instead, then remember to test with localhost
                'server.socket_port' : port_number, # hands off, don't use mine
                },
            '/' : {
                'request.dispatch' : dispatcher, # our dispatcher object
                }
            }
    cherrypy.config.update(conf)
    app = cherrypy.tree.mount(None, config=conf) # creates the app
    cherrypy.quickstart(app)


if __name__ == '__main__':
    ip_address = '10.7.201.15'
    port_number = 5140
    for i, arg in enumerate(argv):
        if arg == '--address':
            ip_address = argv[i + 1]
        if arg == '--port':
            port_number = int(argv[i + 1])

    rospy.init_node('ros_maze_bot')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    start_service(ip_address, port_number, pub)
