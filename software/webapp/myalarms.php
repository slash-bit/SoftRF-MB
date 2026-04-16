<?php
/* ---- Aircraft label lookup ---- */
if (isset($_GET['lookup'])) {
    header('Content-Type: application/json');
    $ids = array_filter(array_map('trim', explode(',', $_GET['lookup'])));
    if (empty($ids)) { echo '{}'; exit; }

    /* Try production path first, then local */
    $dbPath = __DIR__ . '/../ogn_tools/db/aircraft_labels.db';
    if (!file_exists($dbPath)) {
        $dbPath = __DIR__ . '/aircraft_labels.db';
    }
    if (!file_exists($dbPath)) { echo '{}'; exit; }

    $db = new SQLite3($dbPath, SQLITE3_OPEN_READONLY);
    $placeholders = implode(',', array_fill(0, count($ids), '?'));
    $stmt = $db->prepare("SELECT hex, label FROM labels WHERE hex IN ($placeholders)");
    foreach (array_values($ids) as $i => $id) {
        $stmt->bindValue($i + 1, strtoupper($id), SQLITE3_TEXT);
    }
    $result = $stmt->execute();
    $map = [];
    while ($row = $result->fetchArray(SQLITE3_ASSOC)) {
        $map[$row['hex']] = $row['label'];
    }
    $db->close();
    echo json_encode($map);
    exit;
}
?>
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>SoftRF Alarm Log Viewer</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
<style>
:root {
  --bg: #f0f2f5;
  --bg-card: #ffffff;
  --bg-header: #1a1a2e;
  --text: #1a1a2e;
  --text-secondary: #6b7280;
  --text-header: #ffffff;
  --border: #e5e7eb;
  --primary: #4361ee;
  --primary-hover: #3651d4;
  --danger: #ef4444;
  --warning: #f59e0b;
  --success: #10b981;
  --radius: 12px;
  --radius-sm: 8px;
  --shadow: 0 1px 3px rgba(0,0,0,0.1), 0 1px 2px rgba(0,0,0,0.06);
  --shadow-lg: 0 4px 12px rgba(0,0,0,0.1);
}
@media (prefers-color-scheme: dark) {
  :root {
    --bg: #0f0f1a;
    --bg-card: #1a1a2e;
    --bg-header: #0d0d1a;
    --text: #e2e8f0;
    --text-secondary: #94a3b8;
    --border: #2d2d44;
    --primary: #6381ff;
    --primary-hover: #4361ee;
  }
}
* { margin: 0; padding: 0; box-sizing: border-box; }
body {
  font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
  background: var(--bg);
  color: var(--text);
  min-height: 100vh;
}
header {
  background: var(--bg-header);
  color: var(--text-header);
  padding: 16px 20px;
  text-align: center;
  font-size: 18px;
  font-weight: 700;
  letter-spacing: 0.5px;
}
.container {
  max-width: 1200px;
  margin: 0 auto;
  padding: 16px;
}

/* Upload areas */
.upload-row {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 12px;
  margin-bottom: 16px;
}
.upload-area {
  background: var(--bg-card);
  border: 2px dashed var(--border);
  border-radius: var(--radius);
  padding: 30px 16px;
  text-align: center;
  cursor: pointer;
  transition: all 0.2s;
}
.upload-area:hover, .upload-area.dragover {
  border-color: var(--primary);
  background: color-mix(in srgb, var(--primary) 5%, var(--bg-card));
}
.upload-area.loaded {
  border-color: var(--success);
  border-style: solid;
}
.upload-area input { display: none; }
.upload-icon { font-size: 36px; margin-bottom: 8px; }
.upload-text { font-size: 14px; color: var(--text-secondary); }
.upload-text strong { color: var(--primary); }
.upload-filename {
  font-size: 12px; color: var(--success); margin-top: 6px; font-weight: 600;
}

/* Summary */
.summary {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(140px, 1fr));
  gap: 12px;
  margin-bottom: 16px;
}
.stat-card {
  background: var(--bg-card);
  border-radius: var(--radius-sm);
  padding: 16px;
  text-align: center;
  box-shadow: var(--shadow);
}
.stat-value { font-size: 28px; font-weight: 700; }
.stat-label { font-size: 12px; color: var(--text-secondary); margin-top: 4px; }
.stat-level1 .stat-value { color: var(--warning); }
.stat-level2 .stat-value { color: #ff6b35; }
.stat-level3 .stat-value { color: var(--danger); }
.stat-total .stat-value { color: var(--primary); }
.stat-duration .stat-value { color: var(--success); font-size: 20px; }

/* Map */
.map-container {
  background: var(--bg-card);
  border-radius: var(--radius);
  overflow: hidden;
  box-shadow: var(--shadow);
  margin-bottom: 16px;
}
#map { height: 450px; }

/* Table */
.table-container {
  background: var(--bg-card);
  border-radius: var(--radius);
  overflow: hidden;
  box-shadow: var(--shadow);
}
.table-header {
  padding: 16px 20px;
  font-weight: 700;
  font-size: 15px;
  border-bottom: 1px solid var(--border);
}
table {
  width: 100%;
  border-collapse: collapse;
  font-size: 13px;
}
th {
  text-align: left;
  padding: 10px 12px;
  background: color-mix(in srgb, var(--primary) 8%, var(--bg-card));
  font-weight: 600;
  font-size: 11px;
  text-transform: uppercase;
  letter-spacing: 0.5px;
  color: var(--text-secondary);
  position: sticky;
  top: 0;
}
td { padding: 10px 12px; border-bottom: 1px solid var(--border); }
tr:hover td { background: color-mix(in srgb, var(--primary) 4%, var(--bg-card)); }
.scrollable { max-height: 400px; overflow-y: auto; }

/* Level badges */
.level-badge {
  display: inline-block;
  padding: 2px 10px;
  border-radius: 12px;
  font-weight: 600;
  font-size: 12px;
}
.level-1 { background: rgba(245,158,11,0.15); color: #b45309; }
.level-2 { background: rgba(255,107,53,0.15); color: #c2410c; }
.level-3 { background: rgba(239,68,68,0.18); color: #b91c1c; }

/* Legend */
.legend {
  display: flex;
  gap: 16px;
  padding: 12px 20px;
  font-size: 12px;
  color: var(--text-secondary);
  border-top: 1px solid var(--border);
  flex-wrap: wrap;
}
.legend-item { display: flex; align-items: center; gap: 6px; }
.legend-dot {
  width: 12px; height: 12px;
  border-radius: 50%;
  border: 2px solid;
}
.legend-dot.l1 { background: rgba(245,158,11,0.3); border-color: #f59e0b; }
.legend-dot.l2 { background: rgba(255,107,53,0.3); border-color: #ff6b35; }
.legend-dot.l3 { background: rgba(239,68,68,0.3); border-color: #ef4444; }
.legend-dot.takeoff { background: rgba(16,185,129,0.3); border-color: #10b981; }
.legend-line {
  width: 20px; height: 0;
  border-top: 3px solid;
}
.legend-line.igc { border-color: #2196F3; }
.legend-line.alarm-path { border-color: #4361ee; border-style: dashed; }

/* ID colors for multi-aircraft */
.id-badge {
  font-family: monospace;
  font-size: 12px;
  padding: 1px 6px;
  border-radius: 4px;
  background: color-mix(in srgb, var(--primary) 10%, var(--bg-card));
}

.hidden { display: none; }

@media (max-width: 600px) {
  .upload-row { grid-template-columns: 1fr; }
  #map { height: 320px; }
  .summary { grid-template-columns: repeat(3, 1fr); }
  table { font-size: 11px; }
  th, td { padding: 8px 6px; }
}
</style>
</head>
<body>

<header>SoftRF Alarm Log Viewer</header>

<div class="container">
  <div class="upload-row">
    <div class="upload-area" id="uploadAlarm">
      <div class="upload-icon">&#128203;</div>
      <div class="upload-text">
        Drop <strong>alarmlog.txt</strong> here or click
      </div>
      <div class="upload-filename" id="alarmFileName"></div>
      <input type="file" id="alarmInput" accept=".txt,.log,.csv">
    </div>
    <div class="upload-area" id="uploadIGC">
      <div class="upload-icon">&#9992;</div>
      <div class="upload-text">
        Drop <strong>.IGC flight log</strong> here or click (optional)
      </div>
      <div class="upload-filename" id="igcFileName"></div>
      <input type="file" id="igcInput" accept=".igc,.IGC">
    </div>
  </div>

  <div id="results" class="hidden">
    <div class="summary" id="summary"></div>
    <div class="map-container">
      <div id="map"></div>
      <div class="legend" id="legend">
        <div class="legend-item"><span class="legend-dot takeoff"></span> Takeoff</div>
        <div class="legend-item"><span class="legend-dot l1"></span> Level 1 &ndash; Low</div>
        <div class="legend-item"><span class="legend-dot l2"></span> Level 2 &ndash; Important</div>
        <div class="legend-item"><span class="legend-dot l3"></span> Level 3 &ndash; Urgent</div>
        <div class="legend-item"><span style="display:inline-block;width:12px;height:12px;border-left:2px solid #333;"></span> Clock bearing</div>
      </div>
    </div>
    <div class="table-container">
      <div class="table-header">Alarm Events</div>
      <div class="scrollable">
        <table>
          <thead><tr>
            <th>#</th><th>Time (UTC)</th><th>Level</th><th>ID</th>
            <th>Direction</th><th>H Dist (m)</th><th>V Dist (m)</th>
            <th>Lat</th><th>Lon</th>
          </tr></thead>
          <tbody id="tableBody"></tbody>
        </table>
      </div>
    </div>
  </div>
</div>

<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script>
const App = {
  map: null,
  alarmData: null,   /* { takeoff, alarms } */
  igcTrack: null,    /* [ {timeSec, lat, lon, altGPS} ] */
  labelCache: {},    /* hex -> name */

  init() {
    this.setupUpload('uploadAlarm', 'alarmInput', 'alarmFileName', file => {
      const reader = new FileReader();
      reader.onload = e => {
        this.alarmData = this.parseAlarmLog(e.target.result);
        if (this.alarmData) this.fetchLabelsAndRefresh();
      };
      reader.readAsText(file);
    });
    this.setupUpload('uploadIGC', 'igcInput', 'igcFileName', file => {
      const reader = new FileReader();
      reader.onload = e => {
        this.igcTrack = this.parseIGC(e.target.result);
        if (this.alarmData) this.fetchLabelsAndRefresh();
      };
      reader.readAsText(file);
    });
  },

  /* Fetch aircraft labels from DB, then refresh */
  fetchLabelsAndRefresh() {
    const ids = [...new Set(this.alarmData.alarms.map(a => a.id))];
    const uncached = ids.filter(id => !(id in this.labelCache));
    if (uncached.length === 0) { this.refresh(); return; }
    fetch('?lookup=' + encodeURIComponent(uncached.join(',')))
      .then(r => r.json())
      .then(map => {
        Object.assign(this.labelCache, map);
        /* Mark missing IDs so we don't re-fetch */
        uncached.forEach(id => { if (!(id in this.labelCache)) this.labelCache[id] = null; });
        this.refresh();
      })
      .catch(() => this.refresh());  /* on error, just show IDs */
  },

  /* Display name: label if found, otherwise hex ID */
  displayName(id) {
    const label = this.labelCache[id];
    return label ? `${label} (${id})` : id;
  },

  setupUpload(areaId, inputId, nameId, handler) {
    const area = document.getElementById(areaId);
    const input = document.getElementById(inputId);
    const nameEl = document.getElementById(nameId);
    area.addEventListener('click', () => input.click());
    area.addEventListener('dragover', e => { e.preventDefault(); area.classList.add('dragover'); });
    area.addEventListener('dragleave', () => area.classList.remove('dragover'));
    area.addEventListener('drop', e => {
      e.preventDefault();
      area.classList.remove('dragover');
      if (e.dataTransfer.files.length) {
        nameEl.textContent = e.dataTransfer.files[0].name;
        area.classList.add('loaded');
        handler(e.dataTransfer.files[0]);
      }
    });
    input.addEventListener('change', () => {
      if (input.files.length) {
        nameEl.textContent = input.files[0].name;
        area.classList.add('loaded');
        handler(input.files[0]);
      }
    });
  },

  /* Convert NMEA DDMM.MMMM,N/S to decimal degrees */
  nmeaToDecimal(raw, hemisphere) {
    const dot = raw.indexOf('.');
    const degLen = dot - 2;
    const deg = parseInt(raw.substring(0, degLen));
    const min = parseFloat(raw.substring(degLen));
    let dd = deg + min / 60.0;
    if (hemisphere === 'S' || hemisphere === 'W') dd = -dd;
    return dd;
  },

  formatTime(dateStr, timeStr) {
    const hh = timeStr.substring(0, 2);
    const mm = timeStr.substring(2, 4);
    const ss = timeStr.substring(4, 6);
    return `${hh}:${mm}:${ss}`;
  },

  formatDate(dateStr) {
    const dd = dateStr.substring(0, 2);
    const mm = dateStr.substring(2, 4);
    const yy = dateStr.substring(4, 6);
    return `20${yy}-${mm}-${dd}`;
  },

  timeToSeconds(timeStr) {
    const hh = parseInt(timeStr.substring(0, 2));
    const mm = parseInt(timeStr.substring(2, 4));
    const ss = parseFloat(timeStr.substring(4));
    return hh * 3600 + mm * 60 + ss;
  },

  bearingToClock(deg) {
    let h = Math.round(((deg % 360) + 360) % 360 / 30);
    if (h === 0) h = 12;
    return h + " o'clock";
  },

  /* ---------- Alarm log parser ---------- */
  parseAlarmLog(text) {
    const lines = text.trim().split('\n');
    let takeoff = null;
    const alarms = [];

    for (const line of lines) {
      const trimmed = line.trim();
      if (!trimmed || trimmed.startsWith('date,')) continue;

      if (trimmed.startsWith('takeoff:')) {
        const m = trimmed.match(/takeoff:\s*(\S+)\s+(\S+)\s+([-\d.]+),([-\d.]+)/);
        if (m) {
          takeoff = {
            date: m[1], time: m[2],
            lat: parseFloat(m[3]), lon: parseFloat(m[4])
          };
        }
        continue;
      }

      const f = trimmed.split(',');
      if (f.length < 12) continue;
      const level = parseInt(f[6]);
      if (isNaN(level) || level < 1) continue;

      alarms.push({
        date: f[0],
        time: f[1],
        lat: this.nmeaToDecimal(f[2], f[3]),
        lon: this.nmeaToDecimal(f[4], f[5]),
        level: level,
        count: parseInt(f[7]),
        id: f[8],
        relBearing: parseInt(f[9]),
        hDist: parseInt(f[10]),
        vDist: parseInt(f[11])
      });
    }

    if (alarms.length === 0) {
      alert('No alarm entries found in file.');
      return null;
    }
    return { takeoff, alarms };
  },

  /* ---------- IGC B-record parser ---------- */
  parseIGC(text) {
    const lines = text.split('\n');
    const track = [];

    for (const line of lines) {
      const t = line.trim();
      if (t.length < 35 || t[0] !== 'B') continue;

      /* B HHMMSS DDMMmmmN DDDMMmmmE A ppppp ggggg */
      const hh = parseInt(t.substring(1, 3));
      const mm = parseInt(t.substring(3, 5));
      const ss = parseInt(t.substring(5, 7));
      if (isNaN(hh) || isNaN(mm) || isNaN(ss)) continue;
      const timeSec = hh * 3600 + mm * 60 + ss;

      /* Latitude: DDMMmmm N/S  (positions 7-14) */
      const latDeg = parseInt(t.substring(7, 9));
      const latMin = parseInt(t.substring(9, 14)) / 1000.0;
      const latHem = t[14];
      let lat = latDeg + latMin / 60.0;
      if (latHem === 'S') lat = -lat;

      /* Longitude: DDDMMmmm E/W  (positions 15-23) */
      const lonDeg = parseInt(t.substring(15, 18));
      const lonMin = parseInt(t.substring(18, 23)) / 1000.0;
      const lonHem = t[23];
      let lon = lonDeg + lonMin / 60.0;
      if (lonHem === 'W') lon = -lon;

      /* GPS altitude (positions 30-34) */
      const altGPS = parseInt(t.substring(30, 35)) || 0;

      if (isNaN(lat) || isNaN(lon)) continue;
      track.push({ timeSec, lat, lon, altGPS });
    }

    if (track.length === 0) {
      alert('No B records found in IGC file.');
      return null;
    }
    return track;
  },

  /* ---------- Render everything ---------- */
  refresh() {
    const { takeoff, alarms } = this.alarmData;
    document.getElementById('results').classList.remove('hidden');

    /* Summary stats */
    const counts = [0, 0, 0];
    const ids = new Set();
    alarms.forEach(a => { counts[a.level - 1]++; ids.add(a.id); });
    const firstTime = this.timeToSeconds(alarms[0].time);
    const lastTime = this.timeToSeconds(alarms[alarms.length - 1].time);
    const durationSec = Math.round(lastTime - firstTime);
    const durationStr = `${Math.floor(durationSec / 60)}m ${durationSec % 60}s`;
    const dateStr = this.formatDate(alarms[0].date);

    document.getElementById('summary').innerHTML = `
      <div class="stat-card stat-total">
        <div class="stat-value">${alarms.length}</div>
        <div class="stat-label">Total Alarms</div>
      </div>
      <div class="stat-card stat-level1">
        <div class="stat-value">${counts[0]}</div>
        <div class="stat-label">Level 1 &ndash; Low</div>
      </div>
      <div class="stat-card stat-level2">
        <div class="stat-value">${counts[1]}</div>
        <div class="stat-label">Level 2 &ndash; Important</div>
      </div>
      <div class="stat-card stat-level3">
        <div class="stat-value">${counts[2]}</div>
        <div class="stat-label">Level 3 &ndash; Urgent</div>
      </div>
      <div class="stat-card stat-duration">
        <div class="stat-value">${durationStr}</div>
        <div class="stat-label">Span (${dateStr})</div>
      </div>
      <div class="stat-card stat-total">
        <div class="stat-value">${ids.size}</div>
        <div class="stat-label">Aircraft</div>
      </div>
    `;

    /* Table */
    const tbody = document.getElementById('tableBody');
    tbody.innerHTML = '';
    alarms.forEach((a, i) => {
      const tr = document.createElement('tr');
      tr.innerHTML = `
        <td>${i + 1}</td>
        <td>${this.formatTime(a.date, a.time)}</td>
        <td><span class="level-badge level-${a.level}">${
          a.level === 1 ? 'Low' : a.level === 2 ? 'Important' : 'Urgent'
        }</span></td>
        <td><span class="id-badge">${this.displayName(a.id)}</span></td>
        <td>${this.bearingToClock(a.relBearing)}</td>
        <td>${a.hDist}</td>
        <td>${a.vDist >= 0 ? '+' : ''}${a.vDist}</td>
        <td>${a.lat.toFixed(5)}</td>
        <td>${a.lon.toFixed(5)}</td>
      `;
      tr.style.cursor = 'pointer';
      tr.addEventListener('click', () => {
        this.map.setView([a.lat, a.lon], 16);
      });
      tbody.appendChild(tr);
    });

    this.renderMap(takeoff, alarms);
  },

  renderMap(takeoff, alarms) {
    if (this.map) {
      this.map.remove();
      this.map = null;
    }

    this.map = L.map('map');
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      attribution: '&copy; OpenStreetMap contributors',
      maxZoom: 19
    }).addTo(this.map);

    const bounds = L.latLngBounds();
    const levelColors = { 1: '#f59e0b', 2: '#ff6b35', 3: '#ef4444' };
    const levelNames = { 1: 'Low', 2: 'Important', 3: 'Urgent' };
    const levelRadius = { 1: 6, 2: 8, 3: 10 };

    /* IGC flight track (draw first so it's underneath alarm markers) */
    const hasIGC = this.igcTrack && this.igcTrack.length > 0;
    if (hasIGC) {
      const igcCoords = this.igcTrack.map(p => [p.lat, p.lon]);
      L.polyline(igcCoords, {
        color: '#2196F3', weight: 3, opacity: 0.7
      }).addTo(this.map);
      igcCoords.forEach(c => bounds.extend(c));
    }

    /* Takeoff marker */
    if (takeoff) {
      L.circleMarker([takeoff.lat, takeoff.lon], {
        radius: 10, color: '#10b981', fillColor: '#10b981',
        fillOpacity: 0.4, weight: 2
      }).addTo(this.map).bindPopup(
        `<b>Takeoff</b><br>${takeoff.date} ${takeoff.time} UTC`
      );
      bounds.extend([takeoff.lat, takeoff.lon]);
    }

    /* Alarm-point path (dashed line connecting alarm positions) */
    if (!hasIGC) {
      const pathCoords = alarms.map(a => [a.lat, a.lon]);
      L.polyline(pathCoords, {
        color: '#4361ee', weight: 2, opacity: 0.4, dashArray: '6,6'
      }).addTo(this.map);
    }

    /* Alarm markers */
    alarms.forEach((a, i) => {
      const color = levelColors[a.level] || '#888';
      const marker = L.circleMarker([a.lat, a.lon], {
        radius: levelRadius[a.level] || 6,
        color: color,
        fillColor: color,
        fillOpacity: 0.5,
        weight: 2
      }).addTo(this.map);

      const clockStr = this.bearingToClock(a.relBearing);

      /* If IGC loaded, show altitude at alarm time */
      let altInfo = '';
      if (hasIGC) {
        const alarmSec = this.timeToSeconds(a.time);
        const igcPt = this.findIGCPointAt(alarmSec);
        if (igcPt) {
          altInfo = `<b>GPS Alt:</b> ${igcPt.altGPS} m<br>`;
        }
      }

      marker.bindPopup(`
        <div style="font-size:13px; line-height:1.6">
          <b>Alarm #${i + 1}</b> &mdash;
          <span style="color:${color}; font-weight:700">
            Level ${a.level} (${levelNames[a.level]})
          </span><br>
          <b>Time:</b> ${this.formatTime(a.date, a.time)} UTC<br>
          <b>Aircraft:</b> ${this.displayName(a.id)}<br>
          <b>Target:</b> ${clockStr} (${a.relBearing}&deg;)<br>
          <b>H Distance:</b> ${a.hDist} m<br>
          <b>V Distance:</b> ${a.vDist >= 0 ? '+' : ''}${a.vDist} m<br>
          ${altInfo}
          <b>Position:</b> ${a.lat.toFixed(5)}, ${a.lon.toFixed(5)}
        </div>
      `);

      /* Clock-hand needle */
      const needleLen = (levelRadius[a.level] || 6) + 6;
      const angleDeg = a.relBearing;
      L.marker([a.lat, a.lon], {
        icon: L.divIcon({
          className: '',
          html: `<div style="
            position:absolute; left:50%; top:50%;
            width:2px; height:${needleLen}px;
            background:#333; opacity:0.8;
            transform-origin:bottom center;
            transform:translate(-50%,-100%) rotate(${angleDeg}deg);
          "></div>`,
          iconSize: [0, 0]
        }),
        interactive: false
      }).addTo(this.map);

      /* Show alarm number label for level 3 */
      if (a.level >= 3) {
        L.marker([a.lat, a.lon], {
          icon: L.divIcon({
            className: '',
            html: `<div style="
              color:${color}; font-weight:700; font-size:11px;
              text-shadow: 0 0 3px #fff, 0 0 3px #fff;
              white-space:nowrap; position:relative; top:-18px; left:12px;
            ">#${i + 1}</div>`,
            iconSize: [0, 0]
          })
        }).addTo(this.map);
      }

      bounds.extend([a.lat, a.lon]);
    });

    /* Update legend */
    const legendEl = document.getElementById('legend');
    const igcLegend = document.getElementById('igcLegend');
    const pathLegend = document.getElementById('pathLegend');
    if (hasIGC) {
      if (!igcLegend) {
        const item = document.createElement('div');
        item.className = 'legend-item';
        item.id = 'igcLegend';
        item.innerHTML = '<span class="legend-line igc"></span> IGC flight track';
        legendEl.appendChild(item);
      }
      if (pathLegend) pathLegend.style.display = 'none';
    } else {
      if (!pathLegend) {
        const item = document.createElement('div');
        item.className = 'legend-item';
        item.id = 'pathLegend';
        item.innerHTML = '<span class="legend-line alarm-path"></span> Alarm path';
        legendEl.appendChild(item);
      }
      if (igcLegend) igcLegend.style.display = 'none';
      if (pathLegend) pathLegend.style.display = '';
    }

    this.map.fitBounds(bounds, { padding: [40, 40] });
  },

  /* Find closest IGC track point for a given time (seconds of day) */
  findIGCPointAt(timeSec) {
    if (!this.igcTrack || this.igcTrack.length === 0) return null;
    let best = null;
    let bestDiff = Infinity;
    for (const pt of this.igcTrack) {
      const diff = Math.abs(pt.timeSec - timeSec);
      if (diff < bestDiff) {
        bestDiff = diff;
        best = pt;
      }
      if (pt.timeSec > timeSec && diff > bestDiff) break;
    }
    return (bestDiff <= 10) ? best : null;  /* within 10 seconds */
  }
};

document.addEventListener('DOMContentLoaded', () => App.init());
</script>
</body>
</html>
