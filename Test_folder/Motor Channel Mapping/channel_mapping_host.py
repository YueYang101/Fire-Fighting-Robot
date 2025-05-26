from flask import Flask, render_template_string, request, jsonify
import socket, sys

# ------------ configuration ------------
PI_IP   = "192.168.2.3"   # "127.0.0.1" if you host on the Pi
PI_PORT = 12345
PWM_CHANNELS = 16          # 0‑15 outputs on PCA9685
MOTOR_ID = 0               # logical motor we exercise
# ---------------------------------------

app = Flask(__name__)

HTML = """<!doctype html><html><head><title>Robot Motor Control</title></head><body>
<h1>Robot Motor Control</h1>
<p><strong>Pi status:</strong> <span id=status>checking…</span></p>
<label>Speed channel:</label>
<select id=speed onchange=mapped()>{% for c in channels %}<option>{{c}}</option>{% endfor %}</select>
&nbsp;&nbsp;
<label>Direction channel:</label>
<select id=dirch onchange=mapped()>{% for c in channels %}<option>{{c}}</option>{% endfor %}</select>
<hr>
<label>Direction</label>
<select id=dir><option value=forward>Forward</option><option value=backward>Backward</option></select>
<label>Speed</label><input type=range id=spd min=0 max=100 value=50 oninput="slbl.textContent=this.value+'%';">
<span id=slbl>50%</span>
<button onclick=run()>Run Motor</button>
<button onclick=brake()>Brake</button>
<p>Response: <span id=resp></span></p>
<script>
function ping(){fetch('/ping').then(r=>r.json()).then(j=>status.textContent=j.ok?'connected':'offline')}
setInterval(ping,2000); ping();
function mapped(){let s=document.getElementById('speed').value;let d=document.getElementById('dirch').value;fetch(`/map/${s}/${d}`).then(r=>r.json()).then(j=>resp.textContent=j.msg)}
function run(){let dir=document.getElementById('dir').value;let pct=document.getElementById('spd').value;let speed=Math.round(pct/100*65535);fetch(`/drive/${dir}/${speed}`).then(r=>r.json()).then(j=>resp.textContent=j.msg)}
function brake(){fetch('/brake').then(r=>r.json()).then(j=>resp.textContent=j.msg)}
</script></body></html>"""

# -------- helper to talk to Pi ---------

def send(cmd: str) -> str:
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(2)
            s.connect((PI_IP, PI_PORT))
            s.sendall(cmd.encode())
            return s.recv(1024).decode()
    except Exception as e:
        return f"ERROR: {e}"

# -------------- routes -----------------
@app.route('/')
def index():
    return render_template_string(HTML, channels=list(range(PWM_CHANNELS)))

@app.route('/ping')
def ping():
    return jsonify(ok=(send('ping')=='pong'))

@app.route('/map/<int:speed_ch>/<int:dir_ch>')
def map_motor(speed_ch, dir_ch):
    return jsonify(msg=send(f"map,{MOTOR_ID},{speed_ch},{dir_ch}"))

@app.route('/drive/<dir>/<int:speed>')
def drive(dir, speed):
    return jsonify(msg=send(f"{MOTOR_ID},{dir},{speed}"))

@app.route('/brake')
def brake():
    return jsonify(msg=send(f"{MOTOR_ID},brake,0"))

# -------------- main -------------------
if __name__ == '__main__':
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 9000
    app.run(host='0.0.0.0', port=port, debug=True)