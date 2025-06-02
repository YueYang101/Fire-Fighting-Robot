/* ROS-2 Motor Dashboard – robust host handling */

/* ───── Determine backend base URL ─────────────────────────── */
const API_PORT = 8000;
const PAGE_PROTO = location.protocol;          // "http:" or "https:"
const WS_PROTO   = PAGE_PROTO === 'https:' ? 'wss' : 'ws';

const RAW = location.hostname || 'localhost';

// Map IPv6 loopback / "0.0.0.0" → 127.0.0.1 because uvicorn --host 0.0.0.0 binds IPv4:
const HOST = (RAW === '::1' || RAW === 'localhost') ? '127.0.0.1'
           :  (RAW === '0.0.0.0')                   ? '127.0.0.1'
           :  RAW;

const BACKEND = `${PAGE_PROTO}//${HOST}:${API_PORT}`;
const WS_STATE = `${WS_PROTO}://${HOST}:${API_PORT}/ws/motor_state`;
const WS_LOGS  = `${WS_PROTO}://${HOST}:${API_PORT}/ws/logs`;

/* ───── UI helper ───────────────────────────────────────────── */
function logTo(el, m) {
  el.textContent += m + '\n';
  el.scrollTop = el.scrollHeight;
}

/* ───── Power toggle → /start /stop ─────────────────────────── */
const t = document.getElementById('powerToggle');
const L = document.getElementById('powerLabel');
const C = document.getElementById('motorControls');

t.addEventListener('change', () => {
  const on = t.checked;
  L.textContent = on ? 'ON' : 'OFF';
  C.style.opacity = on ? '1' : '0.5';
  C.style.pointerEvents = on ? 'auto' : 'none';
  backendCmd(on ? 'start' : 'stop');
});

/* ───── Generic POST helper ─────────────────────────────────── */
async function backendCmd(ep, body = null) {
  const logEl = document.getElementById('commandLog');
  try {
    const r = await fetch(`${BACKEND}/${ep}`, {
      method: 'POST',
      headers: body ? { 'Content-Type': 'application/json' } : undefined,
      body: body ? JSON.stringify(body) : undefined
    });
    if (!r.ok) {
      logTo(logEl, `❌ ${ep} → HTTP ${r.status}`);
    } else {
      let txt = await r.text();
      logTo(logEl, txt);
    }
  } catch (e) {
    logTo(logEl, 'Backend error: ' + e);
  }
}

/* ───── Send /set_motor ─────────────────────────────────────── */
document.getElementById('sendBtn').addEventListener('click', () => {
  backendCmd('set_motor', {
    motor_id: Number(document.getElementById('motorSelect').value),
    direction: document.getElementById('directionSelect').value,
    speed: Number(document.getElementById('speedInput').value)
  });
});

/* ───── Live motor_state feed ───────────────────────────────── */
(function connectState() {
  const stateEl = document.getElementById('statusContainer');
  const ws = new WebSocket(WS_STATE);

  ws.onopen = () => {
    logTo(document.getElementById('commandLog'), '📡 WS connected (motor_state)');
  };
  ws.onerror = e => {
    logTo(document.getElementById('commandLog'), '⚠️ WS error: ' + e);
  };
  ws.onclose = () => {
    logTo(document.getElementById('commandLog'), '🔌 WS closed – reconnect in 1 s');
    setTimeout(connectState, 1000);
  };
  ws.onmessage = e => {
    const m = JSON.parse(e.data);
    const id = m.motor_id ?? 0;
    const rowId = `motor${id}Row`;
    let row = document.getElementById(rowId);
    if (!row) {
      row = document.createElement('div');
      row.id = rowId;
      row.className = 'p-2 bg-gray-50 rounded';
      stateEl.appendChild(row);
    }
    row.textContent = `Motor ${id}: dir=${m.direction}  speed=${m.speed}`;
  };
})();

/* ───── Live pi_backend logs feed ───────────────────────────── */
(function connectLogs() {
  const logPane = document.getElementById('logContainer');
  const ws = new WebSocket(WS_LOGS);

  ws.onopen = () => {
    logTo(logPane, '📡 WS connected (backend logs)');
  };
  ws.onerror = e => {
    logTo(logPane, '⚠️ WS error: ' + e);
  };
  ws.onclose = () => {
    logTo(logPane, '🔌 WS closed (logs) – reconnect in 1 s');
    setTimeout(connectLogs, 1000);
  };
  ws.onmessage = e => {
    logTo(logPane, e.data);
  };
})();