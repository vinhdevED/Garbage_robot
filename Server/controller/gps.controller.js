const db = require('../local/mongodb')
const GPSController = require('../service/gps.services')

exports.getAllData = async (device_id, res)=> {
    try {
      const data = await db.connectionGPS.collection(`${device_id}`+'s').find({}).toArray(); // Replace with your GPSModel.find() logic
      res.json(data);
    } catch (err) {
      res.status(500).send(err.message);
    }
}

exports.pushFromFlutter = async(device_id,req,res) =>{
  const points = req.body.points;
  console.log(req.body.points);

  if(!Array.isArray(points)){
    return res.status(400).send('Invalid data format. Expecting an array of GPS points.');
  }

  // Create a document to save in the collection
  const routeDocument = {
    points: points,
    // data: {
    //   points: points
    // },
    device_id: device_id
  };

  await GPSController.getDataFromFlutter(routeDocument).then(()=>{
    res.status(200).send("Good");
  });


}


