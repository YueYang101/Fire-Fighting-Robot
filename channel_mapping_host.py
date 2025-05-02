from flask import Flask, render_template_string, request, jsonify
import socket
import json

app = Flask(__name__)

# ---------------------------------------------------------------------------
# CONFIG – tweak here only
# ---------------------------------------------------------------------------
PI_IP   = "192.168.2.3"   # "127.0.0.1" if Flask is on the Pi
PI_PORT = 12345
PWM_CHANNELS = 16          # PCA9685 outputs (0‑15)

# ---------------------------------------------------------------------------
# HTML template – slimmed to essentials, now sends *mapping* automatically
# ---------------------------------------------------------------------------
TEMPLATE = """<!DOCTYPE html>
<html><head><title>Robot Motor Control</title></head><body>
<h1>Robot Motor Control</h1>
<p><strong>Pi Status:</strong> <span id="status">checking…</span></p>
<label>Motor 0 base channel:</label>
<select id="ch0" onchange="setMap(0)">{% for c in channels %}<option>{{c}}</option>{% endfor %}</select>
<br/>
<label>Motor 1 base channel:</label>
<select id="ch1" onchange="setMap(1)">{% for c in channels %}<option>{{c}}</option>{% endfor %}</select>
<hr/>
<label>Direction</label>
<select id="dir"><option value="forward">Forward</option><option value="backward">Backward</option></select>
<label>Speed</label><input type="range" id="spd" min="0" max="100" value="50" oninput="lbl.textContent=this.value+'%'">
<span id="lbl">50%</span>
<button onclick="drive(0)">Run Motor 0</button>
<button onclick="drive(1)">Run Motor 1</button>
<button onclick="brake()">Brake All</button>
<p>Response: <span id="resp"></span></p>
<script>
// Check Pi alive
function ping(){fetch('/ping').then(r=>r.json()).then(d=>status.textContent=d.ok?'connected':'offline').catch(_=>status.textContent='error');}
setInterval(ping,2000);ping();
// Map motor when dropdown changes
function setMap(id){let base=parseInt(document.getElementById('ch'+id).value);fetch('/map',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({motor_id:id,base:base})}).then(r=>r.json()).then(d=>resp.textContent=d.msg);}
// Drive command
function drive(id){let dir=document.getElementById('dir').value;let pct=parseInt(document.getElementById('spd').value);let speed=Math.round(pct/100*65535);fetch('/drive',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({motor_id:id,direction:dir,speed:speed})}).then(r=>r.json()).then(d=>resp.textContent=d.msg);}
function brake(){fetch('/brake').then(r=>r.json()).then(d=>resp.textContent=d.msg);}</script>
</body></html>"""

# ---------------------------------------------------------------------------
# Routes
# ---------------------------------------------------------------------------
@app.route('/')
def index():
    return render_template_string(TEMPLATE, channels=list(range(PWM_CHANNELS)))

@app.route('/ping')
def ping():
    return jsonify(ok=(send_to_pi("ping") == "pong"))

@app.route('/map', methods=['POST'])
def map_motor():
    j = request.get_json(force=True)
    motor_id = int(j['motor_id'])
    base     = int(j['base'])
    cmd = f"map,{motor_id},{base},{base+1}"
    return jsonify(msg=send_to_pi(cmd))

@app.route('/drive', methods=['POST'])
def drive():
    j = request.get_json(force=True)
    cmd = f"{j['motor_id']},{j['direction']},{j['speed']}"
    return jsonify(msg=send_to_pi(cmd))

@app.route('/brake')
def brake():
    # brake every mapped motor (0 and 1 for simplicity)
    msgs = [send_to_pi(f"{m},brake,0") for m in (0,1)]
    return jsonify(msg=" | ".join(msgs))

# ---------------------------------------------------------------------------
# Helper to talk to Pi
# ---------------------------------------------------------------------------

def send_to_pi(cmd: str) -> str:
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(2)
            s.connect((PI_IP, PI_PORT))
            s.sendall(cmd.encode())
            return s.recv(1024).decode()
    except Exception as e:
        return f"ERROR contacting Pi: {e}"

# ---------------------------------------------------------------------------
if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)
