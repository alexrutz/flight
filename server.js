// Static file server for the flight simulator plus a caching tile proxy:
//   /tiles/elev/{z}/{x}/{y}.png  Terrain tiles (Terrarium PNG, AWS Open Data)
//   /tiles/sat/{z}/{x}/{y}.jpg   Sentinel-2 cloudless 2016 by EOX (CC BY 4.0)
// Each upstream tile is fetched once and kept in ./cache.
const http = require('http');
const fs = require('fs');
const path = require('path');
const zlib = require('zlib');

const PORT = Number(process.env.PORT) || 8102;
const CACHE = path.join(__dirname, 'cache');
const ROOTS = {
  '/three/addons/': path.join(__dirname, 'node_modules', 'three', 'examples', 'jsm'),
  '/three/': path.join(__dirname, 'node_modules', 'three', 'build'),
  '/': path.join(__dirname, 'public'),
};
const TYPES = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.svg': 'image/svg+xml',
  '.png': 'image/png',
  '.jpg': 'image/jpeg',
  '.webmanifest': 'application/manifest+json',
  '.json': 'application/json',
  '.wasm': 'application/wasm',
  '.data': 'application/octet-stream',
};
const COMPRESSIBLE = new Set(['.js', '.css', '.json', '.html', '.svg', '.data']);

const TILE_SOURCES = {
  elev: { maxZoom: 15, ext: 'png', url: (z, x, y) => `https://s3.amazonaws.com/elevation-tiles-prod/terrarium/${z}/${x}/${y}.png` },
  sat: { maxZoom: 16, ext: 'jpg', url: (z, x, y) => `https://tiles.maps.eox.at/wmts/1.0.0/s2cloudless_3857/default/g/${z}/${y}/${x}.jpg` },
};

function resolveStatic(urlPath) {
  for (const [prefix, dir] of Object.entries(ROOTS)) {
    if (!urlPath.startsWith(prefix)) continue;
    const file = path.join(dir, path.normalize('/' + urlPath.slice(prefix.length)));
    return file.startsWith(dir + path.sep) ? file : null;
  }
  return null;
}

function sendFile(req, res, file, cacheControl) {
  fs.stat(file, (err, stat) => {
    if (err || !stat.isFile()) {
      res.writeHead(404).end('Not found');
      return;
    }
    const ext = path.extname(file);
    const headers = { 'Content-Type': TYPES[ext] || 'application/octet-stream', 'Cache-Control': cacheControl };
    const gzip = COMPRESSIBLE.has(ext) && /\bgzip\b/.test(req.headers['accept-encoding'] || '') && stat.size > 1024;
    if (gzip) {
      headers['Content-Encoding'] = 'gzip';
      headers.Vary = 'Accept-Encoding';
      res.writeHead(200, headers);
      fs.createReadStream(file).pipe(zlib.createGzip({ level: 6 })).pipe(res);
    } else {
      headers['Content-Length'] = stat.size;
      res.writeHead(200, headers);
      fs.createReadStream(file).pipe(res);
    }
  });
}

// Upstream fetches are de-duplicated and limited in number.
const inflight = new Map();
let active = 0;
const queue = [];
const MAX_UPSTREAM = 8;

function schedule(job) {
  return new Promise((resolve, reject) => {
    queue.push({ job, resolve, reject });
    pump();
  });
}
function pump() {
  while (active < MAX_UPSTREAM && queue.length) {
    const { job, resolve, reject } = queue.shift();
    active++;
    job().then(resolve, reject).finally(() => {
      active--;
      pump();
    });
  }
}

async function fetchTile(kind, z, x, y, file) {
  const src = TILE_SOURCES[kind];
  const res = await fetch(src.url(z, x, y), { headers: { 'User-Agent': 'SkyHopper-FlightSim/1.0 (tile cache)' } });
  if (!res.ok) throw Object.assign(new Error(`upstream ${res.status}`), { status: res.status });
  const buf = Buffer.from(await res.arrayBuffer());
  await fs.promises.mkdir(path.dirname(file), { recursive: true });
  const tmp = `${file}.${process.pid}.tmp`;
  await fs.promises.writeFile(tmp, buf);
  await fs.promises.rename(tmp, file);
}

function handleTile(req, res, match) {
  const [, kind, zs, xs, ys] = match;
  const src = TILE_SOURCES[kind];
  const z = Number(zs), x = Number(xs), y = Number(ys);
  const n = 2 ** z;
  if (!src || z > src.maxZoom || x >= n || y >= n) {
    res.writeHead(404).end('Not found');
    return;
  }
  const file = path.join(CACHE, kind, String(z), String(x), `${y}.${src.ext}`);
  const cacheControl = 'public, max-age=2592000, immutable';
  if (fs.existsSync(file)) {
    sendFile(req, res, file, cacheControl);
    return;
  }
  const key = `${kind}/${z}/${x}/${y}`;
  let p = inflight.get(key);
  if (!p) {
    p = schedule(() => fetchTile(kind, z, x, y, file)).finally(() => inflight.delete(key));
    inflight.set(key, p);
  }
  p.then(
    () => sendFile(req, res, file, cacheControl),
    (err) => {
      res.writeHead(err.status === 404 ? 404 : 502).end('Tile unavailable');
    }
  );
}

http
  .createServer((req, res) => {
    let urlPath;
    try {
      urlPath = decodeURIComponent(req.url.split('?')[0]);
    } catch {
      res.writeHead(400).end('Bad request');
      return;
    }
    const tile = urlPath.match(/^\/tiles\/(elev|sat)\/(\d{1,2})\/(\d{1,6})\/(\d{1,6})\.(png|jpg)$/);
    if (tile) {
      handleTile(req, res, tile);
      return;
    }
    if (urlPath === '/') urlPath = '/index.html';
    const file = resolveStatic(urlPath);
    if (!file) {
      res.writeHead(403).end('Forbidden');
      return;
    }
    sendFile(req, res, file, 'no-cache');
  })
  .listen(PORT, '0.0.0.0', () => console.log(`Flight sim running on http://0.0.0.0:${PORT}`));
