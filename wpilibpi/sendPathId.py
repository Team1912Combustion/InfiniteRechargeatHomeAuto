#!/usr/bin/env python3

import time
import sys

from networktables import NetworkTables
from networktables.util import ntproperty

import logging

logging.basicConfig(level=logging.DEBUG)

class VisionClient(object):
    pathID = ntproperty("/Vision/pathID", 0)

def main():

    if len(sys.argv) != 2:
        print("Error: specify an IP to connect to!")
        exit(0)

    ip = sys.argv[1]

    NetworkTables.initialize(server=ip)

    time.sleep(0.5)

    c = VisionClient()

    i = 0
    path = -1
    while True:

        if i%5 == 0:
            path = path + 1
            if path > 3:
                path = 0

        print("pathID:", path)

        c.pathID = path

        time.sleep(1)
        i += 1

main()
