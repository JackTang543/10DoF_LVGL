# v1.0 

2024.08.29inHOME by SIGHTSEER.
使用ICM42688+LIS3MDLTR+LPS22HBTR得到三轴加速度,三轴角速度,三轴磁力计,气压计,组合得到10轴IMU.经过互补滤波得到准确姿态角,但是目前磁力计没有加入AHRS中.
把得到的数据使用LVGL+ST7789V(1.14")显示在屏幕上.
使用LVGL9.1版本

