const express = require('express');
const http = require('http');
const mqttClient = require('./config/mqttClient')
const socketUtils = require('./utils/socket');
const gpsRoutes = require('./routers/gps.router');

const app = express();
const bodyParser = require('body-parser');
const server = http.createServer(app);
const io = socketUtils.init(server)

app.use(bodyParser.json()); // Body parser middleware
app.use('/clynctrash', gpsRoutes);

server.listen(3000, () => {
  console.log('-> Server is running on port 3000 <-');
});
