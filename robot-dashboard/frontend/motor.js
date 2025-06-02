/*  motor.js – host-side front-end logic  */

/* ───── Configuration ───────────────────────────────────────── */
const API_PORT = 8000;
const PAGE_PROTO = location.protocol;          // "http:" or "https:"
const WS_PROTO   = PAGE_PROTO === 'https:' ? 'wss' : 'ws';

const RAW  = location.hostname || 'localhost';
const HOST = (RAW === '::1' || RAW === 'localhost' || RAW === '0.0.0.0')
           ? '127.0.0.1' : RAW;

const BACKEND = `${PAGE_PROTO}//${HOST}:${API_PORT}`;
const WS_URL  = `${WS_PROTO}://${HOST}:${API_PORT}/ws/motor_state`;

/* ───── Simple logger helpers ───────────────────────────────── */
function log(line) {
  const box = document.getElementById('commandLog');
  box.textContent += line + '\n';
  box.scrollTop = box.scrollHeight;

  logBackend(line);                 /* ⬅︎ mirror into right column */
}

/* ⬅︎ new helper – right-hand “Backend logs” panel */
function logBackend(line) {
  const box = document.getElementById('logContainer');
  if (!box) return;                 // failsafe if HTML missing
  box.textContent += line + '\n';
  box.scrollTop = box.scrollHeight;
}

/* ───── Power toggle => /start /stop ────────────────────────── */
const toggle = document.getElementById('powerToggle');
const label  = document.getElementById('powerLabel');
const panel  = document.getElementById('motorControls');

toggle.addEventListener('change', () => {
  const on = toggle.checked;
  label.textContent = on ? 'ON' : 'OFF';
  panel.style.opacity        = on ? '1'   : '0.5';
  panel.style.pointerEvents  = on ? 'auto': 'none';
  backendCmd(on ? 'start' : 'stop');
});

/* ───── Generic POST wrapper ───────────────────────────────── */
async function backendCmd(endpoint, body = null) {
  try {
    const res = await fetch(`${BACKEND}/${endpoint}`, {
      method : 'POST',
      headers: body ? {'Content-Type':'application/json'} : undefined,
      body   : body ? JSON.stringify(body)                : undefined,
    });
    log(await res.text());
  } catch (err) {
    log('Backend error: ' + err);
  }
}

/* ───── /set_motor sender ──────────────────────────────────── */
document.getElementById('sendBtn').addEventListener('click', () => {
  backendCmd('set_motor', {
    motor_id : Number(document.getElementById('motorSelect').value),
    direction: document.getElementById('directionSelect').value,
    speed    : Number(document.getElementById('speedInput').value),
  });
});

/* ───── Live /motor_X/state feed ───────────────────────────── */
(function connect() {
  const ws = new WebSocket(WS_URL);

  ws.onopen   = () => log('📡 WS connected');
  ws.onerror  = e  => log('WebSocket error: ' + e);
  ws.onclose  = () => { log('🔌 WS closed – reconnect in 1 s');
                        setTimeout(connect, 1000); };
  ws.onmessage = e => {
    const m  = JSON.parse(e.data);
    const id = m.motor_id ?? 0;
    const rowId = `motor${id}Row`;

    let row = document.getElementById(rowId);
    if (!row) {
      row = document.createElement('div');
      row.id = rowId;
      row.className = 'p-2 bg-gray-50 rounded';
      document.getElementById('statusContainer').appendChild(row);
    }
    row.textContent = `Motor ${id}: dir=${m.direction}  speed=${m.speed}`;
  };
})();