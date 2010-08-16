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
matplotlib.use('GTKAgg')
import matplotlib.pyplot as plt

import gobject
import threading
import time

class Spinner(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.daemon = True
    def run(self):
        rospy.spin()

def g_idle():
    global mutex, queue, figure, axes
    while not rospy.is_shutdown():
        mutex.acquire()
        for segment in queue:
            rospy.loginfo('g_idle(): setup %d, segment %d' % (segment.setup_id, segment.segment_id))
            if segment.valid:
                axes['valid pos'].plot(segment.measured_position)
                axes['valid vel'].plot(segment.measured_velocity)
            else:
                axes['invalid pos'].plot(segment.measured_position)
                axes['invalid vel'].plot(segment.measured_velocity)
            figure.canvas.draw()
        queue = list()
        mutex.release()
        time.sleep(0.5)
    raise SystemExit

def segment_cb(segment):
    global mutex, queue, figure, axes
    rospy.loginfo('segment_cb(): setup %d, segment %d' % (segment.setup_id, segment.segment_id))
    mutex.acquire()
    queue.append(segment)
    mutex.release()

if __name__ == '__main__':
    global mutex, queue, figure, axes
    mutex = threading.Lock()
    queue = list()
    figure = plt.figure()
    axes = dict()
    axes['valid pos'] = figure.add_subplot(221)
    axes['valid pos'].set_title('valid')
    axes['valid pos'].set_ylabel('position')
    axes['valid pos'].set_xlabel('time')
    axes['valid vel'] = figure.add_subplot(222)
    axes['valid vel'].set_title('valid')
    axes['valid vel'].set_ylabel('velocity')
    axes['valid vel'].set_xlabel('time')
    axes['invalid pos'] = figure.add_subplot(223)
    axes['invalid pos'].set_title('invalid')
    axes['invalid pos'].set_ylabel('position')
    axes['invalid pos'].set_xlabel('time')
    axes['invalid vel'] = figure.add_subplot(224)
    axes['invalid vel'].set_title('invalid')
    axes['invalid vel'].set_ylabel('velocity')
    axes['invalid vel'].set_xlabel('time')
    rospy.init_node('plot_segments', anonymous = True)
    rospy.Subscriber('/di_segmentation/segment', Segment, segment_cb)
    spinner = Spinner()
    spinner.start()
    gobject.idle_add(g_idle)
    plt.show()
