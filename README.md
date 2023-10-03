# simple_vs

A simple video streamer in ROS

# Dependencies

```
sudo apt install ros-noetic-cv-bridge ros-noetic-image-transport ros-noetic-theora-image-transport
```

# Conexion to mavlink

to Ros
python3 readMAV.py -s udpout:51.91.59.240:14555
python3 readMAV.py -s udpout:51.91.59.240:15000 -f GPS
/usr/bin/python3 /usr/local/bin/mavproxy.py --daemon --master=udp:51.91.59.240:14550 --out=udpin:0.0.0.0:14555 --out=udpin:0.0.0.0:15000 --baudrate=57600
mavproxy.py --master=/dev/ttyUSB0,57600 --out=udpin:0.0.0.0:14555
