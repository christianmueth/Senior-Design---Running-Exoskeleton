Image:
![image](https://github.com/user-attachments/assets/74f07c37-bd42-4705-a625-de2fb45ee9f9)


*By: Christian Mueth, Shuhan Liu*

Title: Ankle Exoskeleton Sensor and Motor Control

File Title: ankle_test_code_withpseudocode_6_6_2024


For: ME Team #1 senior design (2024) Running Exoskeleton

Members: Christian Mueth, DeAndre Thomas, Jaison Dasika Joseph Fleenor, Cole Gala, Shuhan Liu,

Advisor: Dr. Alan Asbeck

**Note: Prior existing ARL Lab code/resources were used to help develop this control code

**Note: a separate code was developed for hip actuation, using a separate microcontroller


Physical function of "Ankle Exoskeleton Sensor and Motor Controls":

This code oscillates rotations for an exoskeleton motor attached to the back of the user, to actuate a supportive ankle exoskeleton

Rotating at a CCW angle actuates a supportive ankle exoskeleton mechanism on the left leg

Rotating at a CW angle actuates a supportive ankle exoskeleton mechanism on the right leg 


Sensor control scheme:

MPU6050 sensors are attached to a user's thighs, and actively read in gyroscope angular velocity data along the x-axis

For active handling of the gyroscope data, the code saves data from the previous loop and data from the current loop

Difference of previous vs. current loop data is taken during each loop iteration, to determine whether there is a negative trend

Once differences are calculated, data is then handled in a state-machine.

If there is a negative difference trend, characteristic angular velocity thresholds from user motion are then used to trigger control of the AK109 motor

Proportional-Derivative control is also used to allow for smooth, proportional actuation. PD control also allows for spring-like compliance upon actuation.


Electronics: MPU6050 IMU/gyroscope sensor, ESP32 microcontroller, CAN bus, AK10-9 motor

Note: in the event of any confusion, please see "Team01_Final_Publication" and the relevant sensor data sheets
