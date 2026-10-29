#!/usr/bin/env python3
"""Optional controller; launching the arena alone leaves the signal red."""
import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String


class TrafficLightController(Node):
    def __init__(self):
        super().__init__('traffic_light_controller', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.declare_parameter('mode', 'auto')
        for key, value in [('red_seconds',8.0),('green_seconds',8.0),('yellow_seconds',2.0)]:
            self.declare_parameter(key,value)
        self.validate({p:self.get_parameter(p).value for p in
                       ['mode','red_seconds','green_seconds','yellow_seconds']})
        self.publisher=self.create_publisher(String,'/traffic_light/command',10)
        self.start=None
        self.last=None
        self.add_on_set_parameters_callback(self.on_parameters)
        self.timer=self.create_timer(0.1,self.tick)
        self.get_logger().info('Waiting for simulation clock; auto sequence RED → GREEN → YELLOW')

    @staticmethod
    def validate(values):
        if 'mode' in values and values['mode'] not in ('auto','red','yellow','green'):
            raise ValueError('mode must be auto/red/yellow/green')
        for key,value in values.items():
            if key.endswith('_seconds') and (not isinstance(value,(float,int)) or not math.isfinite(value) or value<=0):
                raise ValueError(key+' must be finite and positive')

    def on_parameters(self,parameters):
        try:
            self.validate({p.name:p.value for p in parameters})
        except ValueError as error:
            return SetParametersResult(successful=False,reason=str(error))
        self.start=None
        return SetParametersResult(successful=True)

    def tick(self):
        now=self.get_clock().now().nanoseconds/1e9
        if now<=0:return
        if self.start is None or now<self.start:self.start=now
        mode=self.get_parameter('mode').value
        if mode=='auto':
            red=self.get_parameter('red_seconds').value
            green=self.get_parameter('green_seconds').value
            yellow=self.get_parameter('yellow_seconds').value
            elapsed=(now-self.start)%(red+green+yellow)
            state='RED' if elapsed<red else ('GREEN' if elapsed<red+green else 'YELLOW')
        else:state=mode.upper()
        self.publisher.publish(String(data=state))
        if state!=self.last:
            self.get_logger().info('Signal command: '+state)
            self.last=state


def main():
    rclpy.init()
    node=TrafficLightController()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
