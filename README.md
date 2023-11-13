
# Two-wheeled-self-balancing-robot

This robot is able to stand on two wheels.
This project helps me to learn about sensor fusion, the PID, the MPU6050 module, the IMU sensor, and the complementary and Kalmna filters.
## Demo
Watch the robot video here: 
https://www.youtube.com/shorts/I0qjLEPEBZk

## Documentation

I avoided using any library for both the MPU6050 and the PID algorithm, which is typically utilized in calibration aspects.
after receiving row data from the MPU6050 and manipulating it using a complementary filter to generate adequate and precise data.It is then used to implement the proportional, integral, and derivative functions. The PID values are then used to provide the appropriate motor direction and voltage.



