const path = require('path');
const fs = require('fs');
const net = require('net');


const server = net.createServer((socket) => {
    socket.on('data', data => {
        console.log(data.toString())
        const filepath = './hello-world.bin'
        fs.stat(filepath, (err, stats) => {
            if (err) {
                socket.end();     
            } else {
                let buf = Buffer.alloc(4)
                buf.writeUInt32LE(stats.size)
                console.log(buf)
                socket.write(buf, () => {
                    fs.createReadStream('./hello-world.bin').pipe(socket)
                })
            } 
        })
        
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
