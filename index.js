const fs = require('fs')
const net = require('net')

fs.readFile('assets/hello-world.bin', (err, buf) => {
  if (err) return
  const size = Buffer.alloc(4)
  const server = net.createServer(socket =>
    socket.on('data', data =>
      (data.toString() === 'BULBBOOT\n')
        ? socket.end(Buffer.concat([
          (size.writeUInt32LE(buf.length), size),
          buf]))
        : socket.end())
  ).listen({ host: '10.42.0.1', port: 6016 },
    () => console.log(server.address()))
})
