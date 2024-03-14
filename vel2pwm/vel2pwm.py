import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import Adafruit_PCA9685
import time

# PCA9685初期設定
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(60)
steering_ch_F = 0
steering_ch_R = 1
throttle_ch = 2

class CommandSubscriber(Node):

    def __init__(self):
        super().__init__('vel2pwm_node')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
                
    def listener_callback(self, Twist):

        pwm.set_pwm(steering_ch_F,0,map_pca9685(Twist.angular.z))
        pwm.set_pwm(steering_ch_R,0,map_pca9685(-Twist.angular.z))
        pwm.set_pwm(throttle_ch,0,map_pca9685_th(Twist.linear.x))
        self.get_logger().info(f'並進速度={map_pca9685(Twist.linear.x)}角速度={Twist.angular.z}')


def map_pca9685(val):
    in_min = -1.0
    in_max = 1.0
    out_min = 120
    out_max = 640
    return int((val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def map_pca9685_th(val):
    in_min = -1.0
    in_max = 1.0
    out_min = 340
    out_max = 420
    return int((val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)




def main(args=None):
    rclpy.init(args=args)
    command_subscriber = CommandSubscriber()
    rclpy.spin(command_subscriber)
        
    command_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()