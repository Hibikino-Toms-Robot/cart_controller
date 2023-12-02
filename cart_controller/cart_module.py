import serial
import time

"""
@autor yoshida keisuke , shu tahara 
-----------------------------------------
カート下制御
input:  move_val:  ロボットの移動量[mm]
        F:         Forward 正転  進行方向
        R:         Reverse 逆転  逆方向
output:
        状態 :      0 動作完了 1 前方のバンパー衝突 2 後方のバンパー衝突 3 測距センサ閾値内
        現在位置 :  現在位置[]
入力 : 移動量[mm]
例)move_val = 100 → 正方向へ100mm移動
"""

class Cart_Unit():
   
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyUSB-arduino-cart', 115200)
        time.sleep(2) #serial_port openに2秒ほどいるらしい

    def wait_for_serial_data(self):
        start_time = time.time()
        while True:
            if self.ser.in_waiting > 0: 
                self.ser.timeout = 7
                serial_data = self.ser.readline().decode('utf-8').split("D")
                if len(serial_data) == 2:
                    status = serial_data[0][1:]  # 最初の文字 "M" を削除
                    distance = serial_data[1]
                    return status , distance
                else:
                    #print("無効なデータ:", receive_data)
                    return None , None
            elif time.time() - start_time > 10:  # タイムアウト時間を10秒に設定
                print("Timeout: No data received within 10 seconds.")
                return None,None



    def move_cart(self, move_val):
        if move_val > 0:
            #formatは第2引数の値まで0埋め,move_val=10だったら0010　
            send_data = format(int(round(move_val, 1)*10), '04') 
            send_data = 'F' + send_data
        elif move_val < 0:
            send_data = format(int(round(move_val, 1)*10), '05')
            send_data = 'R' + send_data[1:]
        else:
            send_data = 'F0000'
        if send_data[-1] != ',':
            send_data = send_data + ','
        self.ser.write(send_data.encode(encoding='utf-8'))
        status , distance = self.wait_for_serial_data()
        return status , distance

       
