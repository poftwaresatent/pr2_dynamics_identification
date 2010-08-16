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
from dynamics_identification.msg import Analysis

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
        for analysis in queue:
            rospy.loginfo('g_idle(): setup %d, segment %d' % (analysis.setup_id, analysis.segment_id))
            axes['measured'].plot(analysis.measured_position)
            axes['estimated'].plot(analysis.estimated_position)
            figure.canvas.draw()
        queue = list()
        mutex.release()
        time.sleep(0.2)
    raise SystemExit

def analysis_cb(analysis):
    global mutex, queue, figure, axes
    rospy.loginfo('analysis_cb(): setup %d, segment %d' % (analysis.setup_id, analysis.segment_id))
    mutex.acquire()
    queue.append(analysis)
    mutex.release()

if __name__ == '__main__':
    global mutex, queue, figure, axes
    mutex = threading.Lock()
    queue = list()
    figure = plt.figure()
    axes = dict()
    axes['measured'] = figure.add_subplot(211)
    axes['measured'].set_ylabel('measured')
    axes['measured'].set_xlabel('time')
    axes['estimated'] = figure.add_subplot(212)
    axes['estimated'].set_ylabel('estimated')
    axes['estimated'].set_xlabel('time')
    rospy.init_node('plot_analysiss', anonymous = True)
    rospy.Subscriber('/di_analysis/analysis', Analysis, analysis_cb)
    spinner = Spinner()
    spinner.start()
    gobject.idle_add(g_idle)
    plt.show()
