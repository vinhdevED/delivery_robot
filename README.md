# Delivery Robot work indoor (Factory)  
## 🤖 Problem 
- Big problem: The earphone manufacturing factory is facing challenges, such as increasing demand, rising labor costs, and the need for transporting electronic parts between some sections. A shipping robot that can automate the shipping process would help to address these challenges.
## 🐴 Requirement
+ The robot must be able to navigate autonomously in earphone manufacturing factory, 
+ The robot must be able to identify and avoid workers  
+ The robot not only uses wireless communication, for example, bluetooth or wifi, but also has to connect to the mobile app
+ It can pick up and deliver packages, specifically a headset box under 500g
+ as tracking positon, it can detect coordinate
+ Especially, The weight and costs are under 2.5 kg and under 2 million vnd

## 🗺️ Overview  
+This project showcases a delivery robot system that utilizes the MPU6050 sensor for angle detection and control. By integrating the MPU6050 with the delivery robot, we enhance its ability to maintain a straight trajectory during transportation.  
+This project uses STM32F103C8T6 MCU which connects some peripherals to help robot detect do some basic task of whole robot.  
## 📦 Installation  
Easy clone this project and open in STM32Cube IDE  
## 🪓 Material needed
+Some peripheral   
            **- HC-05 (Bluetooth)**  
            **- HCSR-04 (Ultrasonic sensor)**  
            **- MPU6050 (IMU module)**  
            **- L298N (Driver motor)**  
            **- LCD (Option)**  
            **- Voltage stabilizer circuit (Option)**  
_(Cỉcuit map in folder Circuit)_  
## 🚀How it work
**About MPU6050(IMU)**  
![MPU6050](https://blog.mecsu.vn/wp-content/uploads/2022/02/mo-dun-gia-toc-ke-mpu6050-2.png)  
**Gyroscope and Accelerator Axis**  
![Work](https://www.electronicwings.com/storage/PlatformSection/TopicContent/138/description/2_Oreintation_Polarity_of_Rotation_MPU6050.PNG)


