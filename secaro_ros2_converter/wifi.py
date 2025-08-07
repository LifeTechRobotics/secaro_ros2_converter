import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

import socket

# UDP通信用
UDP_PORT_CONTROL_BROADCAST = 4210
UDP_PORT_CONTROL = 4211

class SecaroWiFiNode(Node):

    def __init__(self):
        super().__init__('secaro_wifi_node')

        # サブスクライバーの初期化
        # geometry_msgs/msg/Twist
        # トピック名： /cmd_vel
        # コールバック： topic_callback
        # QoS： 0
        self.subscriber_ = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.topic_callback,
            0
        )

        self.timer = self.create_timer(0.1, self.timer_callback)

        # デバッグログの有効か無効かのROS2パラメータを取得
        # 初期値はFalseでTrueにすると送信成功時にもログを表示する
        # Falseでもエラーログは表示する
        self.declare_parameter('enable_log', True)
        self.enable_log = self.get_parameter('enable_log').get_parameter_value().bool_value

        self.target_ip = None

        # UDP通信の初期化
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', UDP_PORT_CONTROL_BROADCAST))
        self.sock.setblocking(False)

        # 最終更新時の速度
        self.prev_linear_velocity = 1

        self.get_logger().info("Start %s " %self.get_name())

    def topic_callback(self, msg:Twist):
        # どの条件にも当てはまらなかった場合「stop」を送信する
        send_cmd = 'S\n'

        # linear.y > 0　で前進するコマンド
        if(msg.linear.y > 0.0):
            send_cmd = 'F\n'

        # linear.y < 0　で後退するコマンド
        elif(msg.linear.y < 0.0):
            send_cmd = 'B\n'

        # angular.z > 0　で右旋回
        elif(msg.angular.z > 0.0):
            send_cmd = 'R\n'

        # angular.z < 0　で左旋回
        elif(msg.angular.z < 0.0):
            send_cmd = 'L\n'

        # 並進速度も旋回速度も入力がない場合、ストップするコマンド
        # elif(msg.angular.z == 0.0 & msg.linear.y == 0.0):
        #     send_cmd = 'S\n'

        # UDP送信
        # 制御対象が見つかっていない場合は送信しない
        if(self.target_ip != None):
            try:
                # 送信成功ログ(確認用)
                if(self.enable_log):
                    self.get_logger().info("IP: %s 送信: %s" % self.target_ip % send_cmd)
                
                self.sock.sendto(send_cmd.encode('utf-8'), (self.target_ip, UDP_PORT_CONTROL))
            except Exception as e:
                # エラーログ
                self.get_logger().error('error content :%s' % e)

            # 速度変更コマンドのUDP送信
            if((self.prev_linear_velocity - abs(msg.linear.y)) != 0):
                # 最終更新時の速度を更新
                self.prev_linear_velocity = abs(msg.linear.y)

                # 速度変更コマンドの作成
                left_vel_cmd = 'l' + str(self.prev_linear_velocity) + '\n'
                right_vel_cmd = 'l' + str(self.prev_linear_velocity) + '\n'

                # 左輪速度変更コマンドの送信
                self.sock.sendto(left_vel_cmd.encode('utf-8'), (self.target_ip, UDP_PORT_CONTROL))

                # 右輪速度変更コマンドの送信
                self.sock.sendto(right_vel_cmd.encode('utf-8'), (self.target_ip, UDP_PORT_CONTROL))

    # 0.1sごとに呼び出される。
    def timer_callback(self):
        try:
            data, addr = self.sock.recvfrom(1024)
            msg = data.decode()
            if ":" in msg:
                name, ip = msg.split(":")
                self.target_ip = ip
                self.get_logger().info("Found Device. name:%s ip:%s" %name %ip)
        except BlockingIOError:
            pass

def main(args=None):
    rclpy.init(args=args)

    secaro_wifi_node = SecaroWiFiNode()

    rclpy.spin(secaro_wifi_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    secaro_wifi_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()