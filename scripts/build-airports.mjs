// Builds public/data/airports.json from OurAirports (public domain data,
// https://ourairports.com/data/). Keeps airports with at least one usable
// paved runway of 800 m or more.
//
//   node scripts/build-airports.mjs
import fs from 'node:fs/promises';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const root = path.join(path.dirname(fileURLToPath(import.meta.url)), '..');
const cacheDir = path.join(root, 'cache', 'ourairports');
const BASE = 'https://davidmegginson.github.io/ourairports-data';

async function download(name) {
  const file = path.join(cacheDir, name);
  try {
    return await fs.readFile(file, 'utf8');
  } catch {}
  const res = await fetch(`${BASE}/${name}`);
  if (!res.ok) throw new Error(`${name}: ${res.status}`);
  const text = await res.text();
  await fs.mkdir(cacheDir, { recursive: true });
  await fs.writeFile(file, text);
  return text;
}

function parseCsv(text) {
  const rows = [];
  let row = [], field = '', quoted = false;
  for (let i = 0; i < text.length; i++) {
    const c = text[i];
    if (quoted) {
      if (c === '"') {
        if (text[i + 1] === '"') { field += '"'; i++; } else quoted = false;
      } else field += c;
    } else if (c === '"') quoted = true;
    else if (c === ',') { row.push(field); field = ''; }
    else if (c === '\n') { row.push(field); rows.push(row); row = []; field = ''; }
    else if (c !== '\r') field += c;
  }
  if (field || row.length) { row.push(field); rows.push(row); }
  const [head, ...data] = rows;
  return data.map((r) => Object.fromEntries(head.map((h, i) => [h, r[i] ?? ''])));
}

const PAVED = /asp|con|pem|bit|tar|paved|macadam|bet|tarmac|cop|asf/i;
const num = (s) => (s === '' ? null : Number(s));
const round = (v, d) => (v === null ? null : Math.round(v * 10 ** d) / 10 ** d);

const airports = parseCsv(await download('airports.csv'));
const runways = parseCsv(await download('runways.csv'));

const byAirport = new Map();
for (const r of runways) {
  if (r.closed === '1' || !PAVED.test(r.surface)) continue;
  const len = num(r.length_ft);
  const coords = [r.le_latitude_deg, r.le_longitude_deg, r.he_latitude_deg, r.he_longitude_deg].map(num);
  if (!len || len < 2600 || coords.some((c) => c === null)) continue;
  if (!r.le_ident || !r.he_ident) continue;
  const list = byAirport.get(r.airport_ref) || [];
  list.push([
    r.le_ident, r.he_ident,
    round(coords[0], 6), round(coords[1], 6), round(coords[2], 6), round(coords[3], 6),
    num(r.le_elevation_ft), num(r.he_elevation_ft),
    num(r.width_ft) || 150, len,
    num(r.le_displaced_threshold_ft) || 0, num(r.he_displaced_threshold_ft) || 0,
    r.lighted === '1' ? 1 : 0,
  ]);
  byAirport.set(r.airport_ref, list);
}

const out = [];
for (const a of airports) {
  if (!['large_airport', 'medium_airport', 'small_airport'].includes(a.type)) continue;
  const rw = byAirport.get(a.id);
  if (!rw) continue;
  out.push([
    a.ident, a.iata_code || '', a.name, round(num(a.latitude_deg), 5), round(num(a.longitude_deg), 5),
    num(a.elevation_ft) ?? 0, a.iso_country, a.municipality || '', a.type === 'large_airport' ? 2 : a.type === 'medium_airport' ? 1 : 0, rw,
  ]);
}
out.sort((p, q) => q[8] - p[8] || p[0].localeCompare(q[0]));

const fields = ['ident', 'iata', 'name', 'lat', 'lon', 'elevFt', 'country', 'city', 'size', 'runways'];
const runwayFields = ['leIdent', 'heIdent', 'leLat', 'leLon', 'heLat', 'heLon', 'leElevFt', 'heElevFt', 'widthFt', 'lengthFt', 'leDispFt', 'heDispFt', 'lighted'];
await fs.mkdir(path.join(root, 'public', 'data'), { recursive: true });
await fs.writeFile(path.join(root, 'public', 'data', 'airports.json'), JSON.stringify({ source: 'OurAirports (public domain)', fields, runwayFields, airports: out }));
console.log(`${out.length} airports, ${out.reduce((n, a) => n + a[9].length, 0)} runways`);
