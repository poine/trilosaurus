#!/usr/bin/env python3

import sys, time

import robot_manager as rm
#import robot_manager_ros as rm

def main(args, freq_hz=30.):
    r = rm.RobotManager()
    r.start()
    r.run(freq_hz)

    
if __name__ == '__main__':
    main(sys.argv)
