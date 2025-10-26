# CSE412 Robotics Course - Assignment 2: Build Your Own Robot & Ball Chaser

Bu proje, CSE412 Robotik Dersi kapsamında, ROS 2 Humble ve Gazebo Harmonic ortamında sıfırdan bir diferansiyel sürüşlü robot tasarlamayı, simüle etmeyi ve otonom olarak beyaz bir topu takip etmesini sağlamayı amaçlamaktadır.

**YouTube Video Demo:** [Buraya YouTube Videonun Linkini Yapıştır]

![Robot Demo GIF](link_to_your_gif_or_screenshot.gif) 
<!-- Opsiyonel: Projeden güzel bir anın GIF'ini veya ekran görüntüsünü buraya ekleyebilirsin. -->

---

## 📋 İçindekiler

- [Özellikler](#-özellikler)
- [Sistem Gereksinimleri](#-sistem-gereksinimleri)
- [Kurulum](#-kurulum)
- [Kullanım](#-kullanım)
- [Paket Yapısı](#-paket-yapısı)
- [Robot Modeli ve TF Ağacı](#-robot-modeli-ve-tf-ağacı)
- [Karşılaşılan Zorluklar ve Çözümler](#-karşılaşılan-zorluklar-ve-çözümler)
- [Gelecek Geliştirmeler](#-gelecek-geliştirmeler)

---

## ✨ Özellikler

*   **Robot Modeli:** URDF formatında tasarlanmış, iki tahrikli tekerlek, bir destek tekeri ve bir kameradan oluşan diferansiyel sürüşlü robot.
*   **Simülasyon:** Gazebo Harmonic ortamında fiziksel olarak doğru simülasyon.
*   **Hareket Kontrolü:** ROS 2'nin `/cmd_vel` topic'i üzerinden `geometry_msgs/msg/Twist` mesajları ile kontrol.
*   **Sensör Entegrasyonu:** Gazebo kamera sensöründen `/camera/image_raw` topic'ine canlı görüntü akışı.
*   **Otonom Davranış:** OpenCV kullanan bir Python düğümü (`ball_chaser`), kameradan gelen görüntüleri işleyerek beyaz topu algılar ve robota topu takip etmesi için otonom olarak komutlar gönderir.
*   **Tam Entegrasyon:** Tüm sistem (Gazebo, RViz, ROS düğümleri, köprüler) tek bir `ros2 launch` komutu ile başlatılır.

---

## 🛠️ Sistem Gereksinimleri

Bu proje aşağıdaki sistem üzerinde geliştirilmiş ve test edilmiştir:

*   **İşletim Sistemi:** Ubuntu 22.04.5 LTS
*   **ROS 2 Sürümü:** Humble Hawksbill
*   **Gazebo Sürümü:** Gazebo Harmonic (Garden/Ignition)
*   **Gerekli Paketler:** `ros-humble-gazebo-ros-pkgs`, `ros-humble-cv-bridge`, `python3-opencv`, `ros-humble-rqt-image-view`, `gz-harmonic`.

---

## 🚀 Kurulum

1.  **Gerekli ROS 2 ve Gazebo Paketlerini Kurun:**
    ```bash
    sudo apt update
    sudo apt install ros-humble-desktop gz-harmonic ros-humble-gazebo-ros-pkgs ros-humble-cv-bridge python3-opencv
    ```

2.  **ROS 2 Çalışma Alanı (Workspace) Oluşturun:**
    ```bash
    mkdir -p ~/myrobb_ws/src
    cd ~/myrobb_ws/src
    ```

3.  **Bu Repoyu Klonlayın:**
    ```bash
    git clone [Buraya GitHub Repo Linkini Yapıştır] .
    ```

4.  **Bağımlılıkları Yükleyin ve Projeyi Derleyin (Build):**
    ```bash
    cd ~/myrobb_ws
    rosdep install -i --from-path src --rosdistro humble -y
    colcon build
    ```

---

## ▶️ Kullanım

Tüm sistemi tek bir komutla başlatmak için:

1.  **Yeni bir terminal açın ve çalışma alanını `source` edin:**
    ```bash
    source ~/myrobb_ws/install/setup.bash
    ```

2.  **Ana `bringup.launch.py` dosyasını çalıştırın:**
    ```bash
    ros2 launch my_robot_bringup bringup.launch.py
    ```

Bu komut, Gazebo simülasyonunu, RViz görselleştirmesini, ROS-Gazebo köprülerini ve `ball_chaser` otonom kontrol düğümünü başlatacaktır. Robot, Gazebo dünyasına yerleştirilen beyaz topu otomatik olarak algılayıp takip etmeye başlayacaktır.

---

## 📦 Paket Yapısı

Bu çalışma alanı 3 ana paketten oluşmaktadır:

*   **`my_robot_description`**: Robotun URDF modelini, `worlds` ve `rviz` konfigürasyon dosyalarını içerir.
*   **`my_robot_chaser`**: Görüntü işleme ve otonom kontrol mantığını içeren Python tabanlı `ball_chaser` düğümünü barındırır.
*   **`my_robot_bringup`**: Tüm sistemi tek bir komutla başlatan nihai `bringup.launch.py` dosyasını içerir.

---

## 🤖 Robot Modeli ve TF Ağacı

Robot, `base_link`'e bağlı tekerlekler ve bir kameradan oluşur. Sistem, ödev gereksinimlerini karşılayan aşağıdaki TF ağacını oluşturur:

**`world` → `odom` → `base_link` → `{left_wheel_link, right_wheel_link, caster_wheel_link, camera_link}`**

*   **`world` → `odom`:** `static_transform_publisher` tarafından yayınlanan sabit bir dönüşümdür.
*   **`odom` → `base_link`:** Gazebo `DiffDrive` eklentisi tarafından hesaplanan ve `ros_gz_bridge` ile yayınlanan dinamik odometri dönüşümüdür.
*   **`base_link` → Diğerleri:** `robot_state_publisher` tarafından, Gazebo `JointStatePublisher` eklentisinden gelen `/joint_states` verisi kullanılarak yayınlanır.

---

## 🚧 Karşılaşılan Zorluklar ve Çözümler

Bu projenin en öğretici kısmı, sistem seviyesindeki uyumluluk sorunlarının çözülmesiydi. Ana zorluk, çift ekran kartlı (NVIDIA/Intel) bir sistemde Gazebo Harmonic'in sensörlerini (özellikle kamera) çalışır hale getirmekti.

*   **Sorun:** Gazebo, `libEGL` hataları vererek çöküyor veya sensör eklentilerini yükleyemiyordu.
*   **Kök Neden:** Kullanıcının GPU'ya erişim için `video` ve `render` gruplarına üye olmaması, Gazebo dünyasında `Sensors` gibi temel eklentilerin eksik olması ve `ros2 launch`'ın NVIDIA sürücüleriyle uyumsuzluğu.
*   **Çözüm:** Kullanıcıya gerekli izinler verildi, world dosyasına temel sistem eklentileri eklendi ve launch dosyası, Gazebo'yu `ogre` render motoru ve NVIDIA'yı zorlayan ortam değişkenleriyle (`ExecuteProcess` kullanarak) başlatacak şekilde düzenlendi.

---

## 🔮 Gelecek Geliştirmeler

*   **Akıllı Arama Davranışı:** Top kaybolduğunda durmak yerine, robot kendi etrafında dönerek topu aktif olarak arayabilir.
*   **PID Kontrol:** Daha pürüzsüz ve hassas bir takip için basit oransal kontrol yerine tam bir PID (Proportional-Integral-Derivative) kontrolcüsü uygulanabilir.
*   **Engelden Kaçınma:** Robota bir Lidar veya ultrasonik sensörler eklenerek, topu takip ederken engellerden kaçınması sağlanabilir.
