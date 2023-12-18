const router = require('express').Router()

const GPSController = require('../controller/gps.controller')


//GIVE MORE FUNCTION IN GET METHOD -> TAKE DATA AND SHOW IN APPLICATION
router.get("/gps/:device_id", async(req,res)=>{
    const {device_id} = req.params; // Get the device_id from URL parameters
    await GPSController.getAllData(device_id,res);

})

router.post("/listpoint/:device_id",async(req,res)=>{
    const {device_id} = req.params; // Get the device_id from URL parameters
    await GPSController.pushFromFlutter(device_id,req,res);
})

module.exports = router