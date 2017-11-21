#!/usr/bin/env python
import yaml
import os
import time

# read yaml
with open("test.yaml", 'r') as stream:
    try:
        novatel_config = yaml.load(stream)
        print novatel_config['ip']
    except yaml.YAMLError as exc:
        print(exc)

x= novatel_config['antenna1']['x']

print x, type(x)

# config novatel




# os.system('screen /dev/ttyUSB0')
# time.sleep(5)
# os.system('unlogall true')
# os.system('SETIMUTOANTOFFSET {} {} {} {} {} {}'.format(x1, y1, z1))