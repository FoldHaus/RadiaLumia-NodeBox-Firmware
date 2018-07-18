const padw = 4;
const perline = 64;
const elements = 2**12;
const lineprefix = '  ';

const OutMax = elements - 1;
const InMax = elements - 1;

var line = lineprefix;

const f = x => x && Math.round(Math.exp(Math.log(OutMax) / InMax * x));

for (var i = 0; i < elements; i++) {
  line += (f(i) + ',').padStart(padw + 1);
  if (i % perline == perline - 1) {
    console.log(line);
    line = lineprefix;
  }
}