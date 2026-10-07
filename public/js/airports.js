// Airport and runway database (OurAirports, public domain), with runway
// geometry, synthetic ILS data and a spatial index.
import { DEG, FT, metersPerDeg, distance, bearing, destination, angleDiff } from './geo.js';

export const airports = [];
const grid = new Map(); // "lat,lon" (1 degree cells) -> runways

const cellKey = (lat, lon) => `${Math.floor(lat)},${Math.floor(lon)}`;

export async function loadAirports() {
  const res = await fetch('data/airports.json');
  const data = await res.json();
  const f = Object.fromEntries(data.fields.map((n, i) => [n, i]));
  for (const row of data.airports) {
    const apt = {
      ident: row[f.ident], iata: row[f.iata], name: row[f.name], lat: row[f.lat], lon: row[f.lon],
      elevFt: row[f.elevFt], country: row[f.country], city: row[f.city], size: row[f.size], runways: [],
    };
    for (const r of row[f.runways]) apt.runways.push(makeRunway(apt, r));
    airports.push(apt);
    for (const rw of apt.runways) {
      for (const [la, lo] of [[rw.le.lat, rw.le.lon], [rw.he.lat, rw.he.lon]]) {
        const k = cellKey(la, lo);
        const list = grid.get(k) || [];
        if (!list.includes(rw)) list.push(rw);
        grid.set(k, list);
      }
    }
  }
  return airports;
}

function makeRunway(apt, r) {
  const [leIdent, heIdent, leLat, leLon, heLat, heLon, leElevFt, heElevFt, widthFt, lengthFt, leDispFt, heDispFt, lighted] = r;
  const lengthM = distance(leLat, leLon, heLat, heLon);
  const rw = {
    airport: apt,
    widthM: Math.max(18, widthFt * FT),
    lengthM,
    lighted: !!lighted,
    centerLat: (leLat + heLat) / 2,
    centerLon: (leLon + heLon) / 2,
    profile: null, // terrain-fitted surface elevation, set by elevation.js
  };
  const hdgLe = bearing(leLat, leLon, heLat, heLon);
  rw.le = { runway: rw, ident: leIdent, lat: leLat, lon: leLon, elevFt: leElevFt ?? apt.elevFt, dispM: leDispFt * FT, heading: hdgLe, s: 0 };
  rw.he = { runway: rw, ident: heIdent, lat: heLat, lon: heLon, elevFt: heElevFt ?? apt.elevFt, dispM: heDispFt * FT, heading: (hdgLe + 180) % 360, s: lengthM };
  rw.le.opposite = rw.he;
  rw.he.opposite = rw.le;
  // local metric frame around the runway center (accurate over a few km)
  const m = metersPerDeg(rw.centerLat);
  rw.mLat = m.lat;
  rw.mLon = m.lon;
  const dx = (heLon - leLon) * m.lon, dy = (heLat - leLat) * m.lat;
  const len = Math.hypot(dx, dy);
  rw.ux = dx / len; // unit vector le -> he (east, north)
  rw.uy = dy / len;
  for (const end of [rw.le, rw.he]) {
    // landing threshold (displaced) and ILS geometry
    const dir = end === rw.le ? 1 : -1;
    end.thresholdS = end.s + dir * end.dispM;
    const [tl, tn] = runwayToLatLon(rw, end.thresholdS, 0);
    end.thrLat = tl;
    end.thrLon = tn;
    end.ils = { course: end.heading, gs: 3.0, gpiS: end.thresholdS + dir * 300, tchFt: 50 };
  }
  return rw;
}

// Runway coordinates of a point: s along the runway from the le end (m), c to the right of le->he (m).
export function runwayCoords(rw, lat, lon) {
  const dx = (lon - rw.le.lon) * rw.mLon, dy = (lat - rw.le.lat) * rw.mLat;
  return { s: dx * rw.ux + dy * rw.uy, c: dx * rw.uy - dy * rw.ux };
}
export function runwayToLatLon(rw, s, c) {
  const e = s * rw.ux + c * rw.uy, n = s * rw.uy - c * rw.ux;
  return [rw.le.lat + n / rw.mLat, rw.le.lon + e / rw.mLon];
}

export function runwaysNear(lat, lon, radiusM) {
  const out = [];
  const dLat = radiusM / 111000 + 1, dLon = radiusM / (111000 * Math.max(0.1, Math.cos(lat * DEG))) + 1;
  for (let a = Math.floor(lat - dLat); a <= Math.floor(lat + dLat); a++) {
    for (let o = Math.floor(lon - dLon); o <= Math.floor(lon + dLon); o++) {
      for (const rw of grid.get(`${a},${o}`) || []) {
        if (!out.includes(rw) && distance(lat, lon, rw.centerLat, rw.centerLon) < radiusM + rw.lengthM / 2) out.push(rw);
      }
    }
  }
  return out;
}

export function findAirport(ident) {
  const q = ident.trim().toUpperCase();
  return airports.find((a) => a.ident === q) || airports.find((a) => a.iata === q) || null;
}

export function searchAirports(query, limit = 12) {
  const q = query.trim().toUpperCase();
  if (!q) return airports.filter((a) => a.size === 2).slice(0, limit);
  const scored = [];
  for (const a of airports) {
    let score = 0;
    if (a.ident === q || a.iata === q) score = 100;
    else if (a.ident.startsWith(q)) score = 60;
    else if (a.iata && a.iata.startsWith(q)) score = 50;
    else if (a.name.toUpperCase().includes(q)) score = 30;
    else if (a.city.toUpperCase().includes(q)) score = 25;
    if (score) scored.push([score + a.size * 5, a]);
  }
  return scored.sort((x, y) => y[0] - x[0]).slice(0, limit).map((x) => x[1]);
}

export function runwayEnds(apt) {
  return apt.runways.flatMap((rw) => [rw.le, rw.he]).sort((a, b) => a.ident.localeCompare(b.ident, 'en', { numeric: true }));
}

// ILS deviations for a runway end from an aircraft position.
// Returns angular deviations in "dots" (LOC 1 dot = 0.8 deg, GS 1 dot = 0.35 deg) plus distances.
export function ilsDeviation(end, lat, lon, altFt) {
  const rw = end.runway;
  const dir = end === rw.le ? 1 : -1;
  const { s, c } = runwayCoords(rw, lat, lon);
  // distance before the threshold along the approach (positive on the approach side)
  const along = (end.thresholdS - s) * dir;
  const cross = c * dir; // positive = right of the extended centerline when facing the runway
  // localizer antenna beyond the far end
  const locS = dir === 1 ? rw.lengthM + 300 : -300;
  const toLoc = (locS - s) * dir;
  const locDevDeg = Math.atan2(-cross, toLoc) / DEG; // positive = runway is to the right -> fly right
  const gpiAlong = (end.ils.gpiS - s) * dir;
  const surfFt = (rw.profile ? rw.profile(end.ils.gpiS) / FT : end.elevFt);
  const heightFt = altFt - surfFt;
  const gsAngle = Math.atan2(heightFt * FT, Math.max(1, gpiAlong)) / DEG;
  return {
    locDots: Math.max(-2.5, Math.min(2.5, locDevDeg / 0.8)),
    gsDots: Math.max(-2.5, Math.min(2.5, (end.ils.gs - gsAngle) / 0.35)), // positive = glideslope above -> fly up
    locDevDeg,
    gsAngle,
    distM: Math.hypot(along, cross),
    along,
    cross,
    valid: along > -500 && Math.abs(locDevDeg) < 35 && toLoc > 0,
    gsValid: gpiAlong > 0 && Math.abs(locDevDeg) < 10,
  };
}

// Heading/course helpers for the approach start
export function finalApproachPoint(end, distM, heightFt) {
  const [lat, lon] = destination(end.thrLat, end.thrLon, (end.heading + 180) % 360, distM);
  return { lat, lon, heading: end.heading, heightFt };
}

export { angleDiff };
