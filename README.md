## 如何开始运行仿真

运行VI-SLAM



## 如何修改仿真load的数据

在helperDownloadSLAMData函数中，将vioData直接修改为从本地读取



## 如何在原始数据上加入IMU传感器攻击

运行change_imu脚本，该脚本将会在正常数据上引入IMU的攻击数据，并保存，然后利用第二步在仿真中引入受攻击后的数据             