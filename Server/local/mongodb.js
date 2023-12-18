const mongoose = require('mongoose');

const url1 = 'mongodb://0.0.0.0:27017/gps';
const url2 = 'mongodb://0.0.0.0:27017/marker';

// Create connections for two databases
const connectionGPS = mongoose.createConnection(url1);

const connectionMarker = mongoose.createConnection(url2);

// Event listeners for connections
connectionGPS.on('open', () => {
  console.log('-> Connected to GPS database <-');
}).on('error', (error) => {
  console.log('Error connecting to GPS database:', error);
});

connectionMarker.on('open', () => {
  console.log('-> Connected to Marker database <-');
}).on('error', (error) => {
  console.log('Error connecting to Marker database:', error);
});

module.exports = {
  connectionGPS,
  connectionMarker
};
