/* ROS-2 Motor Dashboard – robust host handling  */

/* ───── Determine backend base URL ───────────────────────────── */
const API_PORT = 8000;
const PAGE_PROTO = location.protocol;           // "http:" | "https:"
const WS_PROTO   = PAGE_PROTO === "https:" ? "wss" : "ws";

const RAW  = location.hostname || "localhost";
const HOST = (RAW === "::1" || RAW === "localhost" || RAW === "0.0.0.0")
           ? "127.0.0.1" : RAW;

const BACKEND   = `${PAGE_PROTO}//${HOST}:${API_PORT}`;
const WS_STATE  = `${WS_PROTO}://${HOST}:${API_PORT}/ws/motor_state`;
const WS_LOGS   = `${WS_PROTO}://${HOST}:${API_PORT}/ws/logs`;      /* NEW */

/* ───── UI helper ────────────────────────────────────────────── */
function log(msg){
  const box = document.getElementById("commandLog");
  box.textContent += msg + "\n";
  box.scrollTop    = box.scrollHeight;
}

/* ───── Power toggle → /start /stop ──────────────────────────── */
const t = document.getElementById("powerToggle");
const L = document.getElementById("powerLabel");
const C = document.getElementById("motorControls");

t.addEventListener("change", () => {
  const on = t.checked;
  L.textContent           = on ? "ON" : "OFF";
  C.style.opacity         = on ? "1"  : "0.5";
  C.style.pointerEvents   = on ? "auto" : "none";
  backendCmd(on ? "start" : "stop");
});

/* ───── Generic POST helper ──────────────────────────────────── */
async function backendCmd(ep, body=null){
  try{
    const r = await fetch(`${BACKEND}/${ep}`, {
      method  : "POST",
      headers : body ? { "Content-Type": "application/json" } : undefined,
      body    : body ? JSON.stringify(body)                   : undefined
    });
    log(await r.text());
  }catch(e){ log("Backend error: " + e); }
}

/* ───── Send /set_motor ──────────────────────────────────────── */
document.getElementById("sendBtn").addEventListener("click", () => {
  backendCmd("set_motor", {
    motor_id : Number(document.getElementById("motorSelect").value),
    direction: document.getElementById("directionSelect").value,
    speed    : Number(document.getElementById("speedInput").value)
  });
});

/* ───── Live motor_state feed ────────────────────────────────── */
(function connectState(){
  const ws = new WebSocket(WS_STATE);
  ws.onopen   = ()  => log("📡 WS connected");
  ws.onerror  = (e) => log("WebSocket error: " + e);
  ws.onclose  = ()  => { log("🔌 WS closed — reconnect in 1 s");
                         setTimeout(connectState, 1000); };
  ws.onmessage= (e)  => {
    const m  = JSON.parse(e.data);
    const id = m.motor_id ?? 0;
    const rowId = `motor${id}Row`;
    let row = document.getElementById(rowId);
    if(!row){
      row = document.createElement("div");
      row.id = rowId;
      row.className = "p-2 bg-gray-50 rounded";
      document.getElementById("statusContainer").appendChild(row);
    }
    row.textContent = `Motor ${id}: dir=${m.direction}  speed=${m.speed}`;
  };
})();

/* ───── Live backend log stream – fills right-hand pane ──────── */
(function connectLogs(){
  const pane = document.getElementById("logContainer");
  const ws   = new WebSocket(WS_LOGS);
  ws.onopen   = ()  => append("🪵 log stream connected");
  ws.onerror  = (e) => append("log stream error " + e);
  ws.onclose  = ()  => { append("log stream closed — reconnect in 1 s");
                         setTimeout(connectLogs, 1000); };
  ws.onmessage= (e)  => append(e.data);

  function append(line){
    pane.textContent += line + "\n";
    pane.scrollTop   = pane.scrollHeight;
  }
})();