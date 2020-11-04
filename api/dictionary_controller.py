#!/usr/bin/env python3

import cherrypy
import json
import os
import re
import numpy

class DictionaryController(object):
    # this is a controller class, which holds event handlers
    # constructor
    def __init__(self):
        self.myd = dict()

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
        try:
            os.system('rosrun map_server map_saver -f ~/map')
        except Error as err:
            output['message'] = str(err)

        try:
            output['message'] = {}
            output['message']['map'] = {}
            output['message']['map']['yaml'] = []
            output['message']['map']['pgm'] = []
            image_path = None

            with open('/home/iritter/map.yaml', 'r') as f:
                for line in f.readlines():
                    output['message']['map']['yaml'].append(line.strip())
                    if line.strip():
                        key, value = line.strip().split(': ')
                        if key == 'image':
                            image_path = value

            output['message']['image_path'] = image_path

            if image_path:
                # f = open(image_path, 'rb')
                im = self.read_pgm(image_path, byteorder='<')
                output['message']['map']['pgm'] = im.tolist()
                # f.close()
                
        except IOError as err:
            output['message'] = str(err)

        return json.dumps(output)
    
    def GET_TEST_CONNECT(self):
        output = {'message': 'success'}

        return json.dumps(output)
