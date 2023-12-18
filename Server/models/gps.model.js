const mongoose = require('mongoose');
const db = require('../local/mongodb')

const { Schema } = mongoose;

  // Define a schema for the dynamic collection
  const gpsSchema = new Schema({
    
    //bytes:Array,
    data: {
      
      latitude: Number,
      longitude: Number
    },
    timestamp: { type: Date, default: Date.now }
  });


module.exports = (device_id)=>{
  //Check if the model has already been compiled
  let model;
  if(mongoose.models[device_id]){
    //If the model already exists, use it
    model =  mongoose.model(device_id);
  }else{
    // If the model does not exist, compile it
    model = db.connectionGPS.model(device_id, gpsSchema); 
  }
  return model;

}


// const GPS = db.connectionGPS.model('GPS',gpsSchema)//dbGPS.model('GPS', gpsSchema);

// module.exports = GPS
