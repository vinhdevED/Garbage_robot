
const GPSModel = require('../models/gps.model')
const MarkerModel = require('../models/marker.model')
const db = require('../local/mongodb')
const socket = require('../utils/socket')


class GPSService {

    /*UPLINK FROM THE THING NETWORK*/
    static async gpsFromTTN(newData){
        try {
            const device_id = newData.end_device_ids.device_id;
            const payload = newData.uplink_message.decoded_payload
            const data =  GPSModel(device_id);
            try {

              /*Create new document with insert method*/
              const createNew = {
                data: {
                  latitude: payload['latitude'],
                  longitude: payload['longitude']
                },
                timestamp: new Date() // Set the current timestamp
              }
              /*--------------------------------------*/

          
              /* Save data schema into Database MongoDB */
              await data.insertMany(createNew).then(()=>{
                  console.log('** Created a new GPS entry for device **')
              })
               /*---------------------------------------*/

               /* SocketIO - Realtime emit */
               const io = socket.getIO();
               io.emit('gps-update',newData.uplink_message.decoded_payload);
               console.log('GPS data saved and emitted to Socket.IO');
               /*------------------------*/

            } catch (error) {
                console.error(error);
            }

        } catch (error) {
                console.error('Error saving data to MongoDB:', error);
                throw error;
        }
    }

    static async getDataFromFlutter(newDocument){
      try {
        const data =  MarkerModel(newDocument.device_id);

        const createNew = {
          data:{
            points: newDocument.points
          },
          timestamp: new Date() // Set the current timestamp
        }

        /* Save data schema into Database MongoDB */
        await data.insertMany(createNew).then(()=>{
          console.log('** Created a new List point entry for device **')
      })

      } catch (error) {
        
      }

    }

}

module.exports = GPSService

/*
function decodeUplink(input){
  var data = input.bytes;
  var lat = ((data[0] << 24) + (data[1] << 16) + (data[2] << 8) + data[3]) / 1000000;
  var lon = ((data[4] << 24) + (data[5] << 16) + (data[6] << 8) + data[7]) / 1000000;
  return {
        data: {
      latitude: lat,
      longitude: lon
    },
  }
}*/

//  // Create or update a document in the dynamic collection
                //  const filter = {}; // If you have a specific filter, set it here

                //  const update = {
                //    $set: {
                //      data: newData.uplink_message.decoded_payload,
                //      timestamp: new Date() // Set the current timestamp
                //    }
                //  };
               
                //  // Set the options to upsert
                //  const options = { upsert: true, new: true };

                 //const result = await data.updateOne(filter, update, options);

                 /*if (result.) {
                    console.log('** Created a new GPS entry for device **');
                  } else {
                    console.log(' ** Updated existing GPS entry for device **');
                  }*/