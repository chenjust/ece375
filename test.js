const dth = d => (d < 0 ? (0xFF + d + 1) : d).toString(16)
const pad = (b, s) => new Array((b*2)-s.length).fill().reduce(acc => `0${acc}`, s)
const rand = (min, max) => {
    min = Math.ceil(min);
    max = Math.floor(max);
    return Math.floor(Math.random() * (max - min + 1)) + min;
}

const gen = (pairs) => {
  let sum = 0
  let best = 0
  let same = 0
  let closest = Number.MAX_SAFE_INTEGER
  const res = pairs.map((p, i) => {
    const [x, y] = p
    const b12 = x * x
    const b34 = y * y
    const b56 = b12 + b34
    const b7 = Math.ceil(Math.sqrt(b56))
    sum += b7

    if (b7 < closest) {
      closest = b7
      same = 1
      best = i + 1
    } else if (b7 === closest) {
      same += 1
    }

    return [b12, b34, b56, b7]
  })

  if (same === 2) {
    best = -2
  } else if (same === 3) {
    best = -3
  }

  res.forEach(p => {
    const mp = p.map((x, i) => {
      const m = x.toString(16)
      if (i !== p.length - 1) {
        const h = Buffer.from(pad(2, m), 'hex').readUInt16LE().toString(16)
        return pad(2, h)
      }

      const h = Buffer.from(pad(1, m), 'hex').readUIntLE(0, 1).toString(16)
      return pad(1, h)
    })

    const hs = mp.flatMap(h => h.match(/.{1,2}/g))
    hs.forEach((c, i) => {
      process.stdout.write(`${c} `)
      if (i === hs.length - 1) { console.log() }
    })
  })

  best = pad(1, dth(best))
  const avg = Buffer.from(pad(1, Math.round(sum / 3).toString(16)), 'hex').readUIntLE(0, 1).toString(16)

  console.log(best, avg)
}

const args = new Array(3).fill().map(() => new Array(2).fill().map(() => rand(-128, 127)))
args.map(([b1, b2], i) => `Treasure${i+1}: .DB ${b1}, ${b2}`).forEach(s => console.log(s))
gen(args)
