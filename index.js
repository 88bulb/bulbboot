const path = require('path');
const fs = require('fs');
const net = require('net');


const server = net.createServer((socket) => {
    socket.on('data', data => {
        console.log(data.toString())
        const rs = fs.createReadStream('./hello-world.bin')
        rs.pipe(socket)
    });
}).on('error', (err) => {
  // Handle errors here.
  throw err;
});

// Grab an arbitrary unused port.
server.listen({
    host: '10.42.0.1',
    port: 6016,
}, () => {
  console.log('opened server on', server.address());
});
