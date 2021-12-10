const fs = require('fs')
const net = require('net')

fs.readFile('assets/hello-world.bin', (err, buf) => {
  if (err) return
  const size = Buffer.alloc(4)
  size.writeUInt32LE(buf.length)
  const hash = buf.slice(buf.length - 32).toString('hex')
  console.log(`full digest: ${hash}`)
  console.log(`sha80: ${hash.slice(0, 20)}`)
  const server = net.createServer(socket =>
    socket.on('data', data => {
      const text = data.toString()
      console.log(text)
      if (text[text.length - 1] === '\n') {
        const kv = text.trim().split(' ')
        if (kv.length === 2 && kv[0] === 'GET' && kv[1].length === 20 && hash.startsWith(kv[1])) {
          socket.write(Buffer.concat([size, buf]))
        }
      }
      socket.end()
    })
  ).listen({ host: '10.42.0.1', port: 6016 },
    () => console.log(server.address()))
})
