#!/usr/bin/env python

# Dynamics Identification Toolbox
#
# Copyright (c) 2010 Stanford University. All rights reserved.
#
# This program is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this program.  If not, see
# <http://www.gnu.org/licenses/>

import sys
import roslib
roslib.load_manifest('dynamics_identification')

import rospy
from dynamics_identification.msg import Segment

import matplotlib
matplotlib.use('GTKAgg') # do this before importing pylab
import matplotlib.pyplot as plt

import gobject
import threading
import time

mutex = threading.Lock()
queue = list()
figure = plt.figure()
axes = figure.add_subplot(111)

class Spinner(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.daemon = True
    def run(self):
        rospy.spin()

def g_idle():
    global mutex, queue, figure, axes
    mutex.acquire()
    for segment in queue:
        rospy.loginfo('g_idle(): setup %d, segment %d' % (segment.setup_id, segment.segment_id))
        axes.plot(segment.measured_position)
        figure.suptitle('measured position')
        figure.canvas.draw()
    queue = list()
    mutex.release()
    time.sleep(0.1)

def segment_cb(segment):
    global mutex, queue, figure, axes
    rospy.loginfo('segment_cb(): setup %d, segment %d' % (segment.setup_id, segment.segment_id))
    mutex.acquire()
    queue.append(segment)
    mutex.release()

if __name__ == '__main__':
    rospy.init_node('plot_segments', anonymous = True)
    rospy.Subscriber('/di_analysis/segment', Segment, segment_cb)
    spinner = Spinner()
    spinner.start()
    gobject.idle_add(g_idle)
    plt.show()
