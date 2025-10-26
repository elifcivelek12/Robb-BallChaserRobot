#!/usr/bin/env python3

"""
Ball Chaser Node
Bu node kameradan gelen görüntüleri işler ve beyaz topu takip eder.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np


class BallChaser(Node):
    """
    Beyaz topu takip eden robot node'u
    """
    
    def __init__(self):
        super().__init__('ball_chaser')
        
        # CvBridge: ROS Image <-> OpenCV dönüşümü için
        self.bridge = CvBridge()
        
        # Kırmızı renk tespiti için HSV sınır değerleri
        # Kırmızı HSV'de 0-10 ve 170-180 aralığında (renk çemberinin başı ve sonu)
        # Saturation yüksek, Value orta-yüksek
        self.lower_red = np.array([0, 100, 100])      # Alt sınır: [H, S, V]
        self.upper_red = np.array([10, 255, 255])     # Üst sınır: [H, S, V]
        
        # Subscriber: Kamera görüntülerini dinle
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publisher: Robot hareketlerini yayınla
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.get_logger().info('Ball Chaser Node başlatıldı!')
        self.get_logger().info('Kamera topic: /camera/image_raw')
        self.get_logger().info('Komut topic: /cmd_vel')
        self.get_logger().info(f'Kırmızı renk HSV sınırları: {self.lower_red} - {self.upper_red}')
    
    def image_callback(self, msg):
        """
        Kamera görüntüsü her geldiğinde çağrılır
        
        Args:
            msg (sensor_msgs.msg.Image): Kamera görüntüsü
        """
        try:
            # ROS Image -> OpenCV formatına çevir (BGR formatında)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # ADIM 1: BGR -> HSV renk uzayına çevir
            # HSV, beyaz renk tespiti için RGB/BGR'den daha etkilidir
            # Aydınlatma değişikliklerinden daha az etkilenir
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # ADIM 2: Kırmızı renk için maske oluştur
            # cv2.inRange: Sınırlar içindeki pikseller beyaz (255), dışındakiler siyah (0)
            mask = cv2.inRange(hsv_image, self.lower_red, self.upper_red)
            
            # DEBUG: Maskedeki kırmızı piksel sayısını kontrol et
            red_pixel_count = cv2.countNonZero(mask)
            total_pixels = cv_image.shape[0] * cv_image.shape[1]
            red_percentage = (red_pixel_count / total_pixels) * 100
            
            # ADIM 3: Momentleri kullanarak topun merkezini bul
            # Momentler, maskedeki beyaz piksellerin kütle merkezini hesaplar
            moments = cv2.moments(mask)
            
            # m00: Toplam beyaz piksel sayısı (kütle)
            # Eğer m00 = 0 ise, görüntüde beyaz piksel yok (top görünmüyor)
            if moments['m00'] > 0:
                # Kütle merkezinin koordinatları
                cx = int(moments['m10'] / moments['m00'])  # X koordinatı (yatay)
                cy = int(moments['m01'] / moments['m00'])  # Y koordinatı (dikey)
                
                # Görüntü genişliği
                image_width = cv_image.shape[1]
                
                # ADIM 4: Hareket Kararı Ver
                # Topun pozisyonuna göre robotu hareket ettir
                
                # Sol bölge: Görüntü genişliğinin %40'ından küçük
                if cx < image_width * 0.4:
                    # Top solda → Sola dön
                    self.move_robot(0.0, -0.5)  # linear=0, angular=-0.5 (sola)
                    position = "SOL"
                    action = "Sola dönüyor"
                    
                # Sağ bölge: Görüntü genişliğinin %60'ından büyük
                elif cx > image_width * 0.6:
                    # Top sağda → Sağa dön
                    self.move_robot(0.0, 0.5)  # linear=0, angular=+0.5 (sağa)
                    position = "SAĞ"
                    action = "Sağa dönüyor"
                    
                # Orta bölge: %40 ile %60 arası
                else:
                    # Top ortada → İleri git
                    self.move_robot(0.2, 0.0)  # linear=0.2, angular=0
                    position = "ORTA"
                    action = "İleri gidiyor"
                
                self.get_logger().info(
                    f'Top: {position} ({cx}, {cy}) → {action} | Kırmızı piksel: {red_pixel_count} ({red_percentage:.1f}%)',
                    throttle_duration_sec=1.0
                )
                
            else:
                # Top görünmüyor → Dur
                self.move_robot(0.0, 0.0)
                self.get_logger().info(
                    f'Top görünmüyor! Duruyor... | Kırmızı piksel: {red_pixel_count} ({red_percentage:.1f}%)',
                    throttle_duration_sec=1.0
                )
            
        except Exception as e:
            self.get_logger().error(f'Görüntü işleme hatası: {str(e)}')
    
    def move_robot(self, linear_x, angular_z):
        """
        Robotu hareket ettir
        
        Args:
            linear_x (float): İleri/geri hız (m/s)
            angular_z (float): Dönme hızı (rad/s)
        """
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        
        self.cmd_vel_publisher.publish(twist)
        
        self.get_logger().debug(
            f'Hareket komutu: linear={linear_x:.2f}, angular={angular_z:.2f}'
        )


def main(args=None):
    """
    Node'u başlat
    """
    rclpy.init(args=args)
    
    ball_chaser = BallChaser()
    
    try:
        rclpy.spin(ball_chaser)
    except KeyboardInterrupt:
        pass
    finally:
        ball_chaser.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

