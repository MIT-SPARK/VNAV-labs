#!/usr/bin/env python

# ------------------------------------------------------------------------------
# Function : restamp ros bagfile (using header timestamps)
# Project  : IJRR MAV Datasets
# Author   : www.asl.ethz.ch
# Version  : V01  21JAN2016 Initial version.
# Comment  :
# Status   : under review
#
# Usage    : python restamp_bag.py -i inbag.bag -o outbag.bag -c <bz2 OR lz4>
#
#
# This file has been modified to fit the needs of the SparkVIO project.
# All original credit for this work goes to ETHZ.
# ------------------------------------------------------------------------------

import roslib
import rosbag
import rospy
import sys
import getopt
from   std_msgs.msg import String

usage_msg = 'usage: python restamp_bag.py -i <inputfile> -o <outputfile> -c <compression_type> \n \
             options: <compression_type> can be bz2 (smaller) or lz4 (faster), or not specified for uncompressed.'

def main(argv):

    inputfile = ''
    outputfile = ''
    compression = 'none'

    # parse arguments
    try:
        opts, args = getopt.getopt(argv,"hi:o:c:",["ifile=","ofile=","compression="])
    except getopt.GetoptError:
        print usage_msg
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print usage_msg
            sys.exit()
        elif opt in ("-i", "--ifile"):
            inputfile = arg
        elif opt in ("-o", "--ofile"):
            outputfile = arg
        elif opt in ("-c", "--compression"):
            compression = arg

    # print console header
    print ""
    print "restamp_bag"
    print ""
    print 'input file:  ', inputfile
    print 'output file: ', outputfile
    print 'compression: ', compression
    print ""
    print "starting restamping (may take a while)"
    print ""

    outbag = rosbag.Bag(outputfile, 'w', compression=compression)
    messageCounter = 0
    kPrintDotReductionFactor = 1000

    first_clock = None
    tf_static_msg = []

    try:
        for topic, msg, t in rosbag.Bag(inputfile).read_messages():

            if topic == "/clock":
                outbag.write(topic, msg, msg.clock)
                if first_clock == None or first_clock > msg.clock:
                    first_clock = msg.clock
            elif topic == "/tf":
                outbag.write(topic, msg, msg.transforms[0].header.stamp)
            elif topic == "/tf_static":
                tf_static_msg.append((topic, msg))
                # outbag.write(topic, msg, msg.transforms[0].header.stamp)
            else:
                try:
                    # Write message in output bag with input message header stamp
                    outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)
                except:
                    print "a message has no header here. Coming from topic: ", topic

            if (messageCounter % kPrintDotReductionFactor) == 0:
                    sys.stdout.write('.')
                    sys.stdout.flush()
            messageCounter = messageCounter + 1

        # Write static tfs with the earliest timestamp
        for topic, msg in tf_static_msg:
            outbag.write(topic, msg, first_clock)

    # print console footer
    finally:
        print ""
        print ""
        print "finished iterating through input bag"
        print "output bag written"
        print ""
        outbag.close()

if __name__ == "__main__":
   main(sys.argv[1:])
