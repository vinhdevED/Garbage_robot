<h1 align="center">⚡ Garbage Robot ⚡</h1>

# ✋ Overview 
The problem of plastic waste floating in rivers and lakes is increasing. This leads to water pollution or non-biodegradable plastic waste that will clog and cause loss of aesthetics. Therefore, our team's vision is to design and manufacture a robot that ***automatically*** moves along a route on water. It will be compact so it can be wriggled and collected.

Garbage Robot has a name that is Clync. It is a system for cleaning trash on water (rivers, lakes), operating in an autonomous manner that is set up based on where and when the user wants to clean.
![Image Product](https://github.com/vinhdevED/Garbage_robot/blob/main/Images/Final_Product.jpg)

# 💻 Technology
## -Programming Language-
In this project, we develop many different segments with different languages ​​for their specific purposes. Below are the languages ​​used in this project.
<p align="center">
  <a href="https://skillicons.dev">
    <img src="https://skillicons.dev/icons?i=c,javascript,dart,python" />
  </a>
</p>

## -Framework-
We use ***Flutter Framework*** for cross-platform application development for web/android/ios which helps in optimization and better UI development and stability. Express.js simplifies handling HTTP requests and routing management. It is used with ***NodeJS***.
<p align="center">
  <a href="https://skillicons.dev">
    <img src="https://skillicons.dev/icons?i=flutter,expressjs" />
  </a>
</p>

## -Wireless Communication-
This device operates on water (lakes, rivers) so cannot use Bluetooth, Wifi, Zigbee and other short-range wireless communication technologies. Those types consume more energy so they are not suitable for the operating range of this device.

![Image LoRa](https://github.com/vinhdevED/Garbage_robot/blob/main/Images/lorawan_flow.png)

>LoRaWAN is a wireless communication technology used for wide-range IoT (Internet of Things) networks (LPWAN - Low-Power Wide-Area Network). LoRaWAN is one of the technologies that provides IoT device connectivity with low power consumption and can serve long communication distances.

>LoRaWAN technology uses the **LoRa (Long Range)** wireless communication protocol to connect IoT devices to network infrastructure through communication gateways.

We decided to choose ***LoRa*** as the primary wireless communication from End device to Gateway.

# 🌀 System Architecture IOT

![Image LoRa](https://github.com/vinhdevED/Garbage_robot/blob/main/Images/system_architecture.png)

**Overview of System Architecture**

**1. Application Layer**
  + Showing data to Web/App to track position of Robot moving in the river.

**2. Data Processing Layer**
  + Taking data from ***The Thing Network Server*** and sending to ***NodeJS server with MongoDB (Local Storage)*** through **MQTT**

**3. Transport Layer**
  + Transfer data to Gateway Lora through LoRa
  + Data flow like this [End Device] -> [Gateway] -> [The Thing Network] -> [NodeJS Server]
    
**4. Perception Layer**
   
   | Type | LoRa RFM95W | GPS NEO 6M | MPU6050 | HMC5883L | BTS7996(Actuator) | Raspberry Pi 4 |
   | :----: | :----: | :----: | :----: | :----: | :----: | :----: |
   | **Protocol** | SPI | UART | I2C | I2C | GPIO(PWM) | SPI |
   | **Voltage** | 1.3-3.3 | 3.3-5.0 | 3.3-5.0 | 3.3-5.0 | 3.3-5.0 | Empty |


# Overview
It is a system that uses an automatic balancing module to prevent water drift. In addition, robots use AI to identify waste to pick up trash and move it.


