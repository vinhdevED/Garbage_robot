<h1 align="center">⚡ Autonomous Garbage Collecting Robot ⚡</h1>

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

## -Framework and Another-
We use ***Flutter Framework*** for cross-platform application development for web/android/ios which helps in optimization and better UI development and stability. Express.js simplifies handling HTTP requests and routing management. It is used with ***NodeJS***.
<p align="center">
  <a href="https://skillicons.dev">
    <img src="https://skillicons.dev/icons?i=flutter,expressjs,mongodb,nodejs" />
  </a>
</p>

## -MicroController / Embedded Computer-
Our team uses the **STM32F303RET6** microcontroller to process input data, merge values ​​and filter noise of modules. Additionally, the use of **Raspberry Pi 4** for object (garbage) recognition processing using YOLOv5.

## -Wireless Communication-
This device operates on water (lakes, rivers) so cannot use Bluetooth, Wifi, Zigbee and other short-range wireless communication technologies. Those types consume more energy so they are not suitable for the operating range of this device.

![Image LoRa](https://github.com/vinhdevED/Garbage_robot/blob/main/Images/lorawan_flow.png)

>LoRaWAN is a wireless communication technology used for wide-range IoT (Internet of Things) networks (LPWAN - Low-Power Wide-Area Network). LoRaWAN is one of the technologies that provides IoT device connectivity with low power consumption and can serve long communication distances.

>LoRaWAN technology uses the **LoRa (Long Range)** wireless communication protocol to connect IoT devices to network infrastructure through communication gateways.

We decided to choose ***LoRa*** as the primary wireless communication from End device to Gateway.

# 🌀 System Architecture IOT

![Image LoRa](https://github.com/vinhdevED/Garbage_robot/blob/main/Images/system_architecture.png)
<p align="center"><strong>Overview of System Architecture</strong></p>

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

# 📣 Development
## Firmware for Microcontroller STM32F303RET6
Use Interrupts and Asynchronous - Develop an algorithm that follows point coordinates in a point list
## Create Local Server and Database (NodeJS + MongoDB)
**1. Start new project**
   ```
   npm init
   ```
**2. Install dependencies**
   ```
   npm install body-parser, express, mqtt, mongoose, socket.io
   ```
   > **mqtt** - MQTT protocol for trasfering and receiving data from Network Server (The Thing Network - MQTT Broker)
   
   > **socket.io** - Using for Real-time application between Server and Application Web/App

   > **mongoose** - Connecting with Database MongoDB Compass in Desktop
   
   
**3. Connect Local Server to The Thing Network through MQTT** 
  - First, declare mqtt variables for mqtt dependencies
   ```
   var mqtt = require('mqtt')
   ```
  - Second, add the appropriate required configurations
   ```
   const TTN_BROKER = 'mqtts://nam1.cloud.thethings.industries:8883'
   const TTN_USERNAME = 'autonomousrobot@autonomousrobot';
   const TTN_PASSWORD = 'NNSXS.ITEXEECGUFE3TVGKRXNPTRHVP7FMRDXQT3VFDFA.72UEPZWMCLPJOIVBMMKUGJ7UEIZPOH6FWAGI2EWUDUNEAZQUOI2Q';
    
   const APP_ID ='autonomousrobot'
   const DEV_ID ='fuckyoudat'
   ```
  - Option contain USERNAME and PASSWORD requirement 
   ```
    var options = {
    username: TTN_USERNAME,
    password: TTN_PASSWORD,
    keepalive: 60,
    reconnectPeriod: 1000,
    protocolId: 'MQIsdp',
    protocolVersion: 3,
    clean: true,
    encoding: 'utf8'
    }
   ```
  - Connect to MQTT Broker ( The Thing Network )
   ```
   var client = mqtt.connect(TTN_BROKER,options);
   ```
  - Listen event "connect".
   ```
   client.on('connect',function(){
    console.log('-> The Thing Network MQTT Broker Connected <-');
    /* .......
       .......
       TODO: IN HERE
    */
    })
   ```
  - Continue 'TODO: IN HERE' - Subscribe to receive uplink message from TTN.
   ```
    // Subscribe to receive uplink messages
    const uplinkTopic =`v3/${TTN_USERNAME}/devices/${DEV_ID}/up`
    client.subscribe(uplinkTopic, (err) => {
        if (err) {
        console.error('Error subscribing to TTN:', err);
        }
    });
   ```
  - Continue 'TODO: IN HERE' - Subscribe downlink to push from Server to TTN.
  ```
    // Định nghĩa topic downlink
    const downlinkTopic = `v3/${TTN_USERNAME}/devices/${DEV_ID}/down/push`;
    // Tạo payload
    const message = {
      "downlinks": [{
        "f_port": 3, // Chọn FPort thích hợp
        "frm_payload":"AQ==" ,
        "confirmed": true, // Chọn true nếu muốn xác nhận downlink
        "priority": "HIGHEST" // Mức độ ưu tiên của downlink
      }]
    };
  ```
 - Handled event "message" when catching.
  ```
  client.on('message',function(topic,message){
      // Handle the incoming message
      const payload = JSON.parse(message.toString());
      console.log("Data from TTN: ",payload); 
      GPSService.gpsFromTTN(payload)
  
  })
  ```
## Application for User to monitor and control
Developing application using Flutter (Cross-platform for Web/App) and intergrating **Map API of MapBox Service** 
  
   
   



