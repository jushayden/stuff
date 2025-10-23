// ---- Globals ----
let map, directionsService, directionsRenderer;
let picking = null;           // "start" | "end" | "wp" | null
let startMarker = null, endMarker = null, waypoints = [];
let robotMarker = null;

const el = id => document.getElementById(id);
const msg = (t) => el("msg").textContent = t || "";

// ---- Init Map ----
function initMap() {
  map = new google.maps.Map(document.getElementById("map"), {
    center: { lat: 37.4219999, lng: -122.0840575 },
    zoom: 15,
    mapId: "DEMO_MAP",
    streetViewControl: false
  });

  directionsService = new google.maps.DirectionsService();
  directionsRenderer = new google.maps.DirectionsRenderer({
    map,
    suppressMarkers: true,
    polylineOptions: { strokeColor: "#2a9df4", strokeWeight: 5 }
  });

  map.addListener("click", (e) => onMapClick(e.latLng));

  el("btnPickStart").onclick = () => { picking = "start"; msg("Click on map for START"); };
  el("btnPickEnd").onclick   = () => { picking = "end";   msg("Click on map for END"); };
  el("btnAddWp").onclick     = () => { picking = "wp";    msg("Click on map for WAYPOINT"); };
  el("btnRoute").onclick     = routeAndShow;
  el("btnStartMission").onclick = sendMission;

  msg("");
  setupSocket();
}

// ---- Picking points ----
function onMapClick(latlng) {
  if (!picking) return;
  if (picking === "start") {
    if (startMarker) startMarker.setMap(null);
    startMarker = new google.maps.Marker({ position: latlng, map, label: "S" });
  } else if (picking === "end") {
    if (endMarker) endMarker.setMap(null);
    endMarker = new google.maps.Marker({ position: latlng, map, label: "E" });
  } else if (picking === "wp") {
    const m = new google.maps.Marker({ position: latlng, map, icon: {
      path: google.maps.SymbolPath.CIRCLE, scale: 5, fillColor: "#f80", fillOpacity: 1, strokeWeight: 1, strokeColor: "#a50"
    }});
    waypoints.push({ marker: m, latlng: latlng });
  }
  picking = null;
  msg("");
}

// ---- Route with Google Directions in browser ----
function routeAndShow() {
  const start = startMarker?.getPosition();
  const end   = endMarker?.getPosition();

  if (!start || !end) {
    msg("Pick start and end (or type them and press Enter to place).");
    return;
  }

  const wps = waypoints.map(w => ({ location: w.latlng, stopover: false }));
  const mode = el("mode").value;

  msg("Fetching route...");
  directionsService.route({
    origin: start,
    destination: end,
    waypoints: wps,
    travelMode: google.maps.TravelMode[mode],
    provideRouteAlternatives: false,
  }, (res, status) => {
    if (status !== "OK" || !res.routes.length) {
      msg("Directions error: " + status);
      return;
    }
    directionsRenderer.setDirections(res);

    // start/end display markers
    new google.maps.Marker({ position: res.routes[0].overview_path[0], map, icon: {
      path: google.maps.SymbolPath.CIRCLE, scale: 6, fillColor: "#0a0", fillOpacity: 1, strokeWeight: 1, strokeColor: "#0a0"
    }});
    const last = res.routes[0].overview_path.at(-1);
    new google.maps.Marker({ position: last, map, icon: {
      path: google.maps.SymbolPath.CIRCLE, scale: 6, fillColor: "#000", fillOpacity: 1, strokeWeight: 1, strokeColor: "#000"
    }});

    // Fit bounds
    const b = new google.maps.LatLngBounds();
    res.routes[0].overview_path.forEach(p => b.extend(p));
    map.fitBounds(b, 30);

    msg("");
  });
}

// ---- Socket.IO wiring ----
let socket;
function setupSocket() {
  socket = io(); // connects to same host:port

  socket.on("connect", () => msg("Connected to backend"));
  socket.on("disconnect", () => msg("Disconnected from backend"));

  // incoming telemetry -> update UI + marker
  socket.on("telemetry", t => {
    el("tBattery").textContent = `${Math.round(t.battery)}%`;
    el("tMode").textContent = t.mode;
    el("tSpeed").textContent = `${(t.speed_mps || 0).toFixed(2)} m/s`;
    el("tPos").textContent = `${t.lat.toFixed(6)}, ${t.lng.toFixed(6)}`;

    const pos = new google.maps.LatLng(t.lat, t.lng);
    if (!robotMarker) {
      robotMarker = new google.maps.Marker({
        position: pos, map, title: "Robot",
        icon: { path: google.maps.SymbolPath.CIRCLE, scale: 5, fillColor: "#e33", fillOpacity: 1, strokeWeight: 1, strokeColor: "#a00" }
      });
    } else {
      robotMarker.setPosition(pos);
    }
  });

  socket.on("mission_ack", m => {
    msg("Mission accepted by backend");
  });
}

// ---- Send mission (start/end/waypoints) to Python ----
function sendMission() {
  const start = startMarker?.getPosition()?.toJSON() || null;
  const end   = endMarker?.getPosition()?.toJSON() || null;
  const wps   = waypoints.map(w => w.latlng.toJSON());
  const mode  = el("mode").value;

  if (!start || !end) {
    msg("Pick start and end first.");
    return;
  }
  socket.emit("mission", { start, end, waypoints: wps, mode });
  msg("Mission sent...");
}
