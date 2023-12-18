const mongoose = require('mongoose');
const db = require('../local/mongodb')

const { Schema } = mongoose;

const markerSchema = new Schema({
  data :{
    points:[
      {
        latitude:Number,
        longitude:Number
      }
    ]
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
    model = db.connectionMarker.model(device_id, markerSchema); 
  }
  return model;

}



// const Marker = db.connectionMarker.model('Marker', markerSchema);

// module.exports = Marker
