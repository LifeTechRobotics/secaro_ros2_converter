import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

import socket

# secaro用モジュール
import secaro

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
        self.prev_linear_velocity = 0

        self.get_logger().info("Start %s " %self.get_name())

    def topic_callback(self, msg:Twist):
        """
        Twist型メッセージを受信した際に呼び出される

        Parameters
        ----------
        msg: Twist
            受信したROS2 Twist型メッセージ
        """
        
        # UDP送信
        # 制御対象が見つかっていない場合(==None)は送信しない
        if(self.target_ip != None):

            self.send_motion_cmd(msg=msg)

            self.send_velocity_cmd(msg=msg)

            
        else:
            self.get_logger().error("No Device found yet.")

    def timer_callback(self):
        """
        0.1秒ごとに呼び出される。
        マイコンからのブロードキャストを受信する
        """

        if(self.target_ip == None):
            self.get_logger().info("Scan Device ...")

        try:
            data, addr = self.sock.recvfrom(1024)
            msg = data.decode()
            if ":" in msg:
                name, ip = msg.split(":")
                self.target_ip = ip
                self.get_logger().info("Found Device. name:%s ip:%s" %name %ip)
        except BlockingIOError:
            pass

    def send_motion_cmd(self, msg:Twist):
        """
        動作コマンドを送信する

        Parameters
        ----------
        msg: Twist
            ROS2 Twist型メッセージ
        """
        # Twistから動作コマンド（前進、後退、右旋回、左旋回）を取得
        send_cmd = secaro.get_motion_cmd(linear_velocity=msg.linear.y, angular_velocity=msg.angular.z)

        self.udp_send(send_cmd=send_cmd)

    def send_velocity_cmd(self, msg:Twist):
        """
        車輪速度コマンドを送信する

        Parameters
        ----------
        msg: Twist
            ROS2 Twist型メッセージ
        """
        # Twistから速度コマンドを取得(1~9)
        velocity_cmd = secaro.get_velocity_cmd(vel=msg.linear.y)
        # 速度変更コマンドのUDP送信
        # 前回の速度と変更がある場合のみ送信
        # prev_linear_velocityは０で初期化されてるので初回は必ず速度コマンドを送信する
        if((self.prev_linear_velocity - velocity_cmd) != 0):
            # 最終更新時の速度を更新
            self.prev_linear_velocity = velocity_cmd
            # 速度変更コマンドの作成
            left_vel_cmd = 'l' + str(self.prev_linear_velocity) + '\n'
            right_vel_cmd = 'r' + str(self.prev_linear_velocity) + '\n'
            # 左輪速度変更コマンドの送信
            self.udp_send(send_cmd=left_vel_cmd)
            # 右輪速度変更コマンドの送信
            self.udp_send(send_cmd=right_vel_cmd)

    def udp_send(self, send_cmd:str):
        """
        UDP送信をする

        Parameters
        ----------
        send_cmd: str
            送信データ
        """
        try:
            # 送信成功ログ(確認用)
            if(self.enable_log):
                self.get_logger().info("IP: %s. send content: %s" % self.target_ip % send_cmd)
            
            self.sock.sendto(send_cmd.encode('utf-8'), (self.target_ip, UDP_PORT_CONTROL))
        except Exception as e:
            # エラーログ
            self.get_logger().error('error content :%s' % e)


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