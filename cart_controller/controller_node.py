#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
import serial
from toms_msg.srv import CartService
from std_msgs.msg import Int16,Float32

import sys
sys.path.append("/home/hibikinotoms/hibikino_toms_ws/src/cart_controller/cart_controller")
from cart_module import Cart_Unit


"""
@autor yoshida keisuke 
-----------------------------------------
cart service node
カートの指令値を受取り、指定距離カートを移動させる
"""

class Cart_Controller(Node):
    def __init__(self,**args):
        super().__init__('cart_node')
        #service
        self.srv = self.create_service(CartService, "cart_service", self.controller)        
        self.cart_control = Cart_Unit()
        self.movement = 30

    def controller(self,request, response):
        mode = request.command
        if mode == "forword":
            status , distance = self.cart_control.move_cart(self.movement)
        elif mode == "back":
            status , distance = self.cart_control.move_cart(-self.movement)
        
        if status is not None :
            try :
                status_msg = Int16()
                status_msg.data = int(status)
                dist_msg = Float32()
                dist_msg.data = round(float(distance), 2)
                # self.get_logger().info(f"status: {status_msg.data}")
                # self.get_logger().info(f"distanse: {dist_msg.data}")
                response.status = status_msg
                response.distance = dist_msg
            except :
                status_msg = Int16()
                dist_msg = Float32()
                status_msg.data = 0
                dist_msg.data = 0.0
                response.status = status_msg
                response.distance = dist_msg
        else :
            status_msg = Int16()
            dist_msg = Float32()
            status_msg.data = 0
            dist_msg.data = 0.0
            response.status = status_msg
            response.distance = dist_msg
        return response
       
def main():
    try:
        rclpy.init()
        node = Cart_Controller()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Ctrl+Cが入力されました")  
        print("プログラム終了") 
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
 