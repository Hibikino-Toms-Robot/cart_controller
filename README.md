# cart_controller

## 構成
cart_module     : 下位制御用 arduinoコード  
cart_controller : ROS2通信用ノード  


## send_data  
To Arduino  
input:  move_val:  ロボットの移動量[mm]  
        F:         Forward 正転  進行方向  
        R:         Reverse 逆転  逆方向  
output:  
        状態 :      0 動作完了 1 前方のバンパー衝突 2 後方のバンパー衝突 3 測距センサ閾値内  
        現在位置 :  現在位置[]  
