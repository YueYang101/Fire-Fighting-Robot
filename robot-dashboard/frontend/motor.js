/* ROS-2 Motor Dashboard – robust host handling  */

/* ───── Determine backend base URL ─────────────────────────── */
const API_PORT = 8000;
const PAGE_PROTO = location.protocol;          // "http:" or "https:"
const WS_PROTO   = PAGE_PROTO === 'https:' ? 'wss' : 'ws';

const RAW = location.hostname || 'localhost';

/* Map troublesome hosts (IPv6 loopback) to the IPv4 loopback
   because uvicorn --host 0.0.0.0 binds only to IPv4.            */
const HOST = (RAW === '::1' || RAW === 'localhost') ? '127.0.0.1'
           :  (RAW === '0.0.0.0')                   ? '127.0.0.1'
           :  RAW;

const BACKEND = `${PAGE_PROTO}//${HOST}:${API_PORT}`;
const WS_URL  = `${WS_PROTO}://${HOST}:${API_PORT}/ws/motor_state`;

/* ───── UI helper ───────────────────────────────────────────── */
function log(m){const b=document.getElementById('commandLog');b.textContent+=m+'\n';b.scrollTop=b.scrollHeight;}

/* ───── Power toggle → /start /stop ─────────────────────────── */
const t=document.getElementById('powerToggle');
const L=document.getElementById('powerLabel');
const C=document.getElementById('motorControls');

t.addEventListener('change',()=>{const on=t.checked;
  L.textContent=on?'ON':'OFF';
  C.style.opacity=on?'1':'0.5';
  C.style.pointerEvents=on?'auto':'none';
  backendCmd(on?'start':'stop');
});

/* ───── Generic POST helper ─────────────────────────────────── */
async function backendCmd(ep,body=null){
  try{
    const r=await fetch(`${BACKEND}/${ep}`,{
      method:'POST',
      headers:body?{'Content-Type':'application/json'}:undefined,
      body:body?JSON.stringify(body):undefined
    });
    log(await r.text());
  }catch(e){log('Backend error: '+e);}
}

/* ───── Send /set_motor ─────────────────────────────────────── */
document.getElementById('sendBtn').addEventListener('click',()=>{
  backendCmd('set_motor',{
    motor_id:Number(document.getElementById('motorSelect').value),
    direction:document.getElementById('directionSelect').value,
    speed:Number(document.getElementById('speedInput').value)
  });
});

/* ───── Live motor_state feed ───────────────────────────────── */
(function connect(){
  const ws=new WebSocket(WS_URL);
  ws.onopen   =()=>log('📡 WS connected');
  ws.onerror  =e =>log('WebSocket error: '+e);
  ws.onclose  =()=>{log('🔌 WS closed – reconnect in 1 s');setTimeout(connect,1000);};
  ws.onmessage=e =>{
      const m=JSON.parse(e.data), id=m.motor_id??0, rowId=`motor${id}Row`;
      let row=document.getElementById(rowId);
      if(!row){row=document.createElement('div');row.id=rowId;
               row.className='p-2 bg-gray-50 rounded';
               document.getElementById('statusContainer').appendChild(row);}
      row.textContent=`Motor ${id}: dir=${m.direction}  speed=${m.speed}`;
  };
})();