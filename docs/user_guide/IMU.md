# IMU

We are using BNO055 9-axis absolute orienation sensor, please check more detail on [this datasheet](https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf).

You can check the IMU data and adapt to your code by first go to atdrive-moab-tools/testing_tools, and run a testing script.

`cd atdrive-moab-tools/testing_tools/` 

`python3 host_imu_receiver.py`

It will print out the yaw angle or heading angle of the UGV. For end user, this should be enough, but for a developer you can use a raw value such as quaternion, linear velocity vector, gravity vector, temperature and calibration status to your own purpose. You can check on `IMUPacket.py` to see how to parse packet from MOAB.