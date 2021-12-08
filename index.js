const fs = require('fs')
const net = require('net')

fs.readFile('hello-world.bin', (err, buf) => {
  if (err) return
  const server = net.createServer(socket =>
    socket.on('data', data =>
      (data.toString() === 'BULBBOOT\n')
        ? socket.end(Buffer.concat([
          Buffer.alloc(4).writeUInt32LE(buf.length),
          buf]))
        : socket.end())
  ).listen({ host: '10.42.0.1', port: 6016 },
    () => console.log(server.address()))
})
