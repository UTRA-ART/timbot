// Waypoints
// Curently static and constant, could change in future to dynamic version
const WAYPOINTS = {
  A: { lat: 43.6610182, lon: -79.3948450, alt: 73.0, color: '#ff6b35' },
  B: { lat: 43.661900,  lon: -79.394179,  alt: 70.0, color: '#ff6b35' },
  C: { lat: 43.661505,  lon: -79.396064,  alt: 70.0, color: '#7fff6e' },
};

// Track colors for multi-file display (Add more if needed, else wraps around)
const TRACK_COLORS = ['#00d4ff', '#ff6b35', '#c77dff', '#ffbe0b', '#7fff6e'];

let loadedFiles = [];
let mapInstance = null;
let drawnLayers = [];
let waypointMarkers = [];

let currentLayer = 'stadia';
let stadiaKey = '';
let baseTileLayer = null;
let labelLayer = null;

function applyLayer() {
  if (!mapInstance) return;

  if (baseTileLayer) { mapInstance.removeLayer(baseTileLayer); baseTileLayer = null; }
  if (labelLayer)    { mapInstance.removeLayer(labelLayer);    labelLayer = null; }

  baseTileLayer = L.tileLayer(
    `https://tiles.stadiamaps.com/tiles/alidade_satellite/{z}/{x}/{y}.jpg?api_key=${stadiaKey}`
  ).addTo(mapInstance);
  log('ok', 'Layer: Stadia Alidade Satellite (2025 refresh)');
}

// Leaflet Map Init
function initMap() {
  mapInstance = L.map('gmap', {
    center: [43.6614, -79.3950],
    zoom: 17,
    zoomControl: true,
  });

  applyLayer();
  drawWaypoints();
}

function drawWaypoints() {
  const wpColors = { A: '#ff6b35', B: '#2670d7', C: '#58c549' };
  Object.entries(WAYPOINTS).forEach(([label, wp]) => {
    const icon = L.divIcon({    
      html: `<div style="
        width:26px;height:26px;border-radius:50%;
        background:${wpColors[label]};
        border:2px solid white;
        display:flex;align-items:center;justify-content:center;
        font-family:'Share Tech Mono',monospace;font-size:11px;font-weight:bold;color:white;
        box-shadow:0 0 10px ${wpColors[label]}88;
      ">${label}</div>`,
      className: '',
      iconSize: [26, 26],
      iconAnchor: [13, 13],
    });
    const marker = L.marker([wp.lat, wp.lon], { icon })
      .addTo(mapInstance)
      .bindPopup(`
        <div style="font-family:'Share Tech Mono',monospace;font-size:11px;color:#080c10">
          <b>Waypoint ${label}</b><br>
          Lat: ${wp.lat.toFixed(7)}<br>
          Lon: ${wp.lon.toFixed(7)}<br>
          Alt: ${wp.alt}m
        </div>
      `);
    waypointMarkers.push(marker);
  });
}

// CSV Loading 
document.getElementById('file-input').addEventListener('change', e => {
  handleFiles([...e.target.files]);
  e.target.value = '';
});

const dropZone = document.getElementById('drop-zone');
dropZone.addEventListener('dragover', e => { e.preventDefault(); dropZone.classList.add('drag-over'); });
dropZone.addEventListener('dragleave', () => dropZone.classList.remove('drag-over'));
dropZone.addEventListener('drop', e => {
  e.preventDefault();
  dropZone.classList.remove('drag-over');
  handleFiles([...e.dataTransfer.files].filter(f => f.name.endsWith('.csv')));
});

function handleFiles(files) {
  files.forEach(file => {
    Papa.parse(file, {
      header: true,
      dynamicTyping: true,
      skipEmptyLines: true,
      complete(results) {
        const rows = results.data;
        // Detect columns flexibly
        const cols = Object.keys(rows[0] || {}).map(c => c.trim().toLowerCase());
        const hasCols = ['latitude','longitude'].every(c => cols.includes(c));
        if (!hasCols) {
          log('warn', `${file.name}: missing latitude/longitude columns`);
          return;
        }

        // Normalise column names
        const norm = rows.map(r => {
          const out = {};
          Object.entries(r).forEach(([k, v]) => { out[k.trim().toLowerCase()] = v; });
          return out;
        }).filter(r => r.latitude != null && r.longitude != null);

        loadedFiles.push({ name: file.name, data: norm });
        updateFileList();
        log('ok', `Loaded ${file.name} — ${norm.length} points`);
        document.getElementById('btn-plot').disabled = false;
      },
      error(err) { log('warn', `Parse error: ${err.message}`); }
    });
  });
}

function updateFileList() {
  const container = document.getElementById('file-list');
  const legend = document.getElementById('file-legend');
  container.innerHTML = '';
  legend.innerHTML = '';

  loadedFiles.forEach((f, i) => {
    const color = TRACK_COLORS[i % TRACK_COLORS.length];
    // File list item
    const item = document.createElement('div');
    item.className = 'file-item';
    item.innerHTML = `
      <div class="file-dot" style="background:${color};box-shadow:0 0 6px ${color}66"></div>
      <span class="file-name">${f.name}</span>
      <span class="file-pts">${f.data.length}pts</span>
    `;
    container.appendChild(item);
    // Legend entry
    const li = document.createElement('div');
    li.className = 'legend-item';
    li.innerHTML = `
      <div class="legend-color" style="background:linear-gradient(to right,${color}44,${color})"></div>
      <span style="font-family:var(--mono);font-size:11px">${f.name.replace('.csv','')}</span>
    `;
    legend.appendChild(li);
  });

  document.getElementById('hdr-files').textContent = loadedFiles.length;
}

// Plot 
document.getElementById('btn-plot').addEventListener('click', plotAll);
document.getElementById('btn-clear').addEventListener('click', clearAll);

function applyAPI() {
  const key = document.getElementById('apikey-input').value.trim();
  stadiaKey = key;
  log('info', 'Applied API Key');
  
  applyLayer();
  drawWaypoints();
}

function clearAll() {
  drawnLayers.forEach(l => mapInstance.removeLayer(l));
  drawnLayers = [];
  loadedFiles = [];
  updateFileList();
  document.getElementById('btn-plot').disabled = true;
  document.getElementById('hdr-pts').textContent = '—';
  document.getElementById('hdr-dist').textContent = '—';
  document.getElementById('hdr-alt').textContent = '—';
  log('info', 'Cleared all trajectories');
}

function plotAll() {
  // Remove previous drawn layers (but keep waypoints)
  drawnLayers.forEach(l => mapInstance.removeLayer(l));
  drawnLayers = [];

  let totalPts = 0;
  let allLats = [], allLons = [];

  loadedFiles.forEach((f, fi) => {
    const color = TRACK_COLORS[fi % TRACK_COLORS.length];
    const pts = f.data;
    if (pts.length < 2) return;

    totalPts += pts.length;

    const latLngs = pts.map(p => [p.latitude, p.longitude]);
    allLats.push(...pts.map(p => p.latitude));
    allLons.push(...pts.map(p => p.longitude));

    // Compute timestamps for gradient
    const times = pts.map(p => parseFloat(p.timestamp ?? 0));
    const tMin = Math.min(...times), tMax = Math.max(...times);

    // Draw gradient polyline using segments
    for (let i = 0; i < latLngs.length - 1; i++) {
      const t = tMax > tMin ? (times[i] - tMin) / (tMax - tMin) : i / (latLngs.length - 1);
      const segColor = plasmaColor(t);
      const seg = L.polyline([latLngs[i], latLngs[i+1]], {
        color: segColor,
        weight: 3,
        opacity: 0.9,
      }).addTo(mapInstance);
      drawnLayers.push(seg);
    }

    // Start marker (diamond-ish)
    const startIcon = L.divIcon({
      html: `<div style="width:12px;height:12px;background:${color};border:2px solid white;transform:rotate(45deg);box-shadow:0 0 8px ${color}"></div>`,
      className: '', iconSize: [12, 12], iconAnchor: [6, 6]
    });
    const sm = L.marker(latLngs[0], { icon: startIcon })
      .addTo(mapInstance)
      .bindPopup(`<div style="font-family:'Share Tech Mono',monospace;font-size:11px"><b>Start</b><br>${f.name}<br>Lat: ${pts[0].latitude.toFixed(7)}<br>Lon: ${pts[0].longitude.toFixed(7)}</div>`);
    drawnLayers.push(sm);

    // End marker (square)
    const endIcon = L.divIcon({
      html: `<div style="width:12px;height:12px;background:#f85149;border:2px solid white;box-shadow:0 0 8px #f85149"></div>`,
      className: '', iconSize: [12, 12], iconAnchor: [6, 6]
    });
    const em = L.marker(latLngs[latLngs.length - 1], { icon: endIcon })
      .addTo(mapInstance)
      .bindPopup(`<div style="font-family:'Share Tech Mono',monospace;font-size:11px"><b>End</b><br>${f.name}</div>`);
    drawnLayers.push(em);
  });

  // Fit map to data
  if (allLats.length > 0) {
    const bounds = L.latLngBounds(allLats.map((lat, i) => [lat, allLons[i]]));
    mapInstance.fitBounds(bounds.pad(0.15));
  }

  // Update header stats
  document.getElementById('hdr-pts').textContent = totalPts.toLocaleString();
  
  log('ok', `Plotted ${loadedFiles.length} track(s), ${totalPts} pts`);
}

// Colormap function 
function plasmaColor(t) {
  t = Math.max(0, Math.min(1, t));
  // Approximate plasma colormap keypoints
  const stops = [
    [13,  8, 135],
    [84,  2, 163],
    [139, 10, 165],
    [185, 50, 137],
    [219, 92, 104],
    [244,136,  73],
    [254,188,  43],
    [240,249,  33],
  ];
  const idx = t * (stops.length - 1);
  const lo = Math.floor(idx), hi = Math.min(lo + 1, stops.length - 1);
  const frac = idx - lo;
  const r = Math.round(stops[lo][0] + frac * (stops[hi][0] - stops[lo][0]));
  const g = Math.round(stops[lo][1] + frac * (stops[hi][1] - stops[lo][1]));
  const b = Math.round(stops[lo][2] + frac * (stops[hi][2] - stops[lo][2]));
  return `rgb(${r},${g},${b})`;
}

// Logging (Helps to communcate erros with file inputs or etc)
const logEl = document.getElementById('log');
function log(type, msg) {
  const cls = type === 'ok' ? 'ok' : type === 'warn' ? '25' : 'info';
  const ts = new Date().toISOString().slice(11,19);
  const line = document.createElement('div');
  line.className = 'log-line';
  line.innerHTML = `<span class="ts">[${ts}]</span><span class="${cls}">${msg}</span>`;
  logEl.appendChild(line);
  logEl.scrollTop = logEl.scrollHeight;
}

// Init function call
initMap();