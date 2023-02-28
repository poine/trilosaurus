#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import String
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


import numpy as np, matplotlib.pyplot as plt
import pandas as pd

#import 
def decorate(ax, title=None, xlab=None, ylab=None, legend=None, xlim=None, ylim=None, min_yspan=None):
    ax.grid()
    ax.xaxis.grid(color='k', linestyle='-', linewidth=0.2)
    ax.yaxis.grid(color='k', linestyle='-', linewidth=0.2)
    if xlab: ax.xaxis.set_label_text(xlab)
    if ylab: ax.yaxis.set_label_text(ylab)
    if title: ax.set_title(title)#, {'fontsize': 20 })
    if legend is not None:
        if legend == True: ax.legend(loc='best')
        else: ax.legend(legend, loc='best')
    if xlim is not None: ax.set_xlim(xlim[0], xlim[1])
    if ylim is not None: ax.set_ylim(ylim[0], ylim[1])
    if min_yspan is not None: ensure_yspan(ax, min_yspan)


class RecordNode(Node):

    def __init__(self):
        super().__init__('record_node')
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("topic", "/trilosaurus_base_controller/cmd_vel_unstamped")
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.topic = self.get_parameter("topic").get_parameter_value().string_value
        self.publisher_ = self.create_publisher(Twist, self.topic, 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.s1 = self.create_subscription(Twist, '/trilosaurus_base_controller/cmd_vel_unstamped',
                                           self.listener_callback,10)
        self.s2 = self.create_subscription(Odometry, '/trilosaurus_base_controller/odom',
                                           self.listener_callback2,10)
        self.sp_x, self.meas_x = 0., 0.#None, None
        self.sp_z, self.meas_z = 0., 0.#None, None
        self.records = []
 
    def timer_callback(self):
      outMsg = Twist()
      #outMsg.header = Header()
      #outMsg.header.stamp = self.get_clock().now().to_msg()
      #outMsg.header.frame_id = self.frame_id
      outMsg.angular.z = 0.2;
      self.publisher_.publish(outMsg)
      #self.get_logger().info('Publishing: "%s"' % msg.data)
      print(f"lin {self.sp_x:.2f} {self.meas_x:.2f} ang  {self.sp_z:.2f} {self.meas_z:.2f}")
      self.i += 1

    def listener_callback(self, msg):
        #self.get_logger().info('setpoint: "%s"' % msg.angular.z)
        self.sp_x = msg.linear.x
        self.sp_z = msg.angular.z

    def listener_callback2(self, msg):
        #self.get_logger().info('measured: "%s"' % msg.twist.twist.angular.z)
        self.meas_x = msg.twist.twist.linear.x
        self.meas_z = msg.twist.twist.angular.z
        self.records.append([Time.from_msg(msg.header.stamp).nanoseconds/1e9, self.meas_x, self.meas_z, self.sp_x, self.sp_z])
        
        
def record(args=None):
    rclpy.init(args=args)

    rec_node = RecordNode()
    try:
        rclpy.spin(rec_node)
    except KeyboardInterrupt:
        
        #print(rec_node.records)
        _r = np.array(rec_node.records)
        #df = pd.DataFrame(_r[:,1:], index=_r[:,0], columns=list("ABCD"))
        df = pd.DataFrame(_r[:,1:], index=_r[:,0], columns=('mx', 'mz', 'spx', 'spz'))
        print(df)
        df.to_pickle("/tmp/ident_data.pkl")
        print("Bye")

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rec_node.destroy_node()
    rclpy.shutdown()

def plot():
    df = pd.read_pickle("/tmp/ident_data.pkl")
    fig, axes = plt.subplots(figsize=(16, 9), nrows=2, ncols=1, sharex=True, sharey=False)
    #print(df)
    axes[0].plot(df.index, df['mx'])
    axes[0].plot(df.index, df['spx'])
    decorate(axes[0], title="linear", xlab="m/s", ylab=None, legend=True)
    axes[1].plot(df.index, df['mz'])
    axes[1].plot(df.index, df['spz'])
    decorate(axes[1], title="angular", xlab="rad/s", ylab=None, legend=True)
    plt.show()
    
def main(args=None):
    #record()
    plot()
    
    
if __name__ == '__main__':
    main()
