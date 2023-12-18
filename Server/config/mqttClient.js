var mqtt = require('mqtt')

const GPSService = require('../service/gps.services')

/*-----------------------------MQTT SET UP------------------------------*/
const TTN_BROKER = 'mqtts://nam1.cloud.thethings.industries:8883'
const TTN_USERNAME = 'autonomousrobot@autonomousrobot';
const TTN_PASSWORD = 'NNSXS.ITEXEECGUFE3TVGKRXNPTRHVP7FMRDXQT3VFDFA.72UEPZWMCLPJOIVBMMKUGJ7UEIZPOH6FWAGI2EWUDUNEAZQUOI2Q';

const APP_ID ='autonomousrobot'
const DEV_ID ='fuckyoudat'


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
/*------------------------------------------------------------*/

var client = mqtt.connect(TTN_BROKER,options);



client.on('connect',function(){
    console.log('-> The Thing Network MQTT Broker Connected <-');

     // Subscribe to receive uplink messages
    const uplinkTopic =`v3/${TTN_USERNAME}/devices/${DEV_ID}/up`
    client.subscribe(uplinkTopic, (err) => {
        if (err) {
        console.error('Error subscribing to TTN:', err);
        }
    });

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

    // Gửi tin nhắn downlink
    client.publish(downlinkTopic, JSON.stringify(message), (err) => {
      if (err) {
        console.error('Error sending downlink message:', err);
      } else {
        console.log('Downlink message sent!');
      }
    });
})


client.on('message',function(topic,message){
    // Handle the incoming message
    const payload = JSON.parse(message.toString());
    console.log("Data from TTN: ",payload); 
    GPSService.gpsFromTTN(payload)

})

/*-------------------ERROR-------------------*/
client.on('error', function(err) {
    console.log(err);
});

client.on("close", function() { 
    console.log("Connection closed by client") 
}) 

client.on("reconnect", function() { 
    console.log("Client trying a reconnection") 
}) 

client.on("offline", function() { 
    console.log("Client is currently offline") 
})
/*--------------------------------------------*/

module.exports = client;