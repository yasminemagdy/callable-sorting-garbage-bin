#!/usr/bin/env python
from __future__ import print_function
import os
import time
import subprocess
import threading
import yaml
import rospy
import actionlib
from flask import Flask, request
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty

try:
    import Queue as queue   # Python 2 (Melodic)
except ImportError:
    import queue            # Python 3

WAYPOINTS = os.path.expanduser("~/kobuki_nav_config/waypoints/majlis_positions_v3.yaml")

def load_seats():
    with open(WAYPOINTS, "r") as f:
        data = yaml.safe_load(f)
    if isinstance(data, dict) and "positions" in data and isinstance(data["positions"], dict):
        data = data["positions"]
    seats = {}
    for name, v in data.items():
        if not isinstance(v, dict):
            continue
        if all(k in v for k in ("x", "y", "qz", "qw")):
            seats[str(name).upper()] = {"x": float(v["x"]), "y": float(v["y"]),
                                        "qz": float(v["qz"]), "qw": float(v["qw"])}
    return seats

SEATS = load_seats()
print("Loaded {0} seats: {1}".format(len(SEATS), ", ".join(sorted(SEATS))))

PI_HOST = "ubuntu@192.168.0.158"

mb_client = None

# ---------------- serialized mission queue ----------------
# One worker thread runs missions one at a time. /dispatch enqueues and returns
# immediately, so several calls can arrive at once and the robot handles them
# sequentially instead of two orchestrators fighting over one robot.
mission_q = queue.Queue()
_qlock = threading.Lock()
_pending = []                        # seats waiting (not yet running), in order
_current = {"seat": None, "since": None}
_history = []                        # recent (seat, rc), newest first

def clear_costmaps():
    try:
        srv = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)
        srv.wait_for_service(timeout=3.0)
        srv()
        rospy.loginfo("costmaps cleared")
    except Exception as e:
        rospy.logwarn("clear_costmaps failed: %s", e)

def _run_mission_blocking(seat, s):
    remote = (
        "source /opt/ros/noetic/setup.bash && "
        "source ~/kobuki_ws/devel/setup.bash && "
        "export ROS_MASTER_URI=http://192.168.0.156:11311 && "
        "export ROS_HOSTNAME=192.168.0.158 && "
        "python3 ~/mission_orchestrator.py {seat} {x} {y} {qz} {qw} "
        "> ~/mission_last.log 2>&1"
    ).format(seat=seat, x=s["x"], y=s["y"], qz=s["qz"], qw=s["qw"])
    # Blocks until the orchestrator exits (i.e. the whole mission is finished).
    return subprocess.call(["ssh", "-o", "StrictHostKeyChecking=no", PI_HOST, remote])

def mission_worker():
    while True:
        seat = mission_q.get()
        s = SEATS.get(seat)
        if s is None:
            mission_q.task_done()
            continue
        with _qlock:
            if seat in _pending:
                _pending.remove(seat)
            _current["seat"] = seat
            _current["since"] = time.time()
        rospy.loginfo("MISSION START seat %s", seat)
        clear_costmaps()
        try:
            rc = _run_mission_blocking(seat, s)
        except Exception as e:
            rospy.logwarn("mission error seat %s: %s", seat, e)
            rc = -1
        rospy.loginfo("MISSION DONE seat %s rc=%s", seat, rc)
        with _qlock:
            _current["seat"] = None
            _current["since"] = None
            _history.insert(0, (seat, rc))
            del _history[6:]
        mission_q.task_done()

# ---------------- web UI ----------------
STYLE = """
 body{font-family:-apple-system,Segoe UI,Roboto,sans-serif;background:#0f1226;
   color:#e8eaf6;margin:0;padding:28px;text-align:center}
 .card{max-width:420px;margin:0 auto;background:#191d3a;border-radius:18px;
   padding:28px 22px;box-shadow:0 10px 30px rgba(0,0,0,.35)}
 h1{font-size:20px;margin:0 0 4px} .sub{color:#9aa0c7;font-size:14px;margin:0 0 18px}
 .seat{font-size:54px;font-weight:700;margin:10px 0 4px;color:#7dd3fc}
 .coord{color:#9aa0c7;font-size:13px;margin-bottom:22px}
 a.btn{display:block;text-decoration:none;background:#22c55e;color:#06210f;
   font-size:20px;font-weight:700;padding:18px;border-radius:14px;margin-top:8px}
 a.link{display:inline-block;color:#7dd3fc;text-decoration:none;margin-top:16px;font-size:14px}
 .ok{font-size:48px;margin:6px 0} .msg{color:#9aa0c7;font-size:15px}
 .row{display:flex;justify-content:space-between;padding:10px 6px;border-bottom:1px solid #2a2f55}
 .run{color:#7dd3fc;font-weight:700} .wait{color:#fbbf24} .done{color:#9aa0c7}
 .fail{color:#f87171}
"""

PAGE = ("<!doctype html><html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width, initial-scale=1'>"
        "<title>Callable Sorting Bin</title><style>{style}</style></head>"
        "<body><div class='card'>{body}</div></body></html>")

REFRESH_PAGE = ("<!doctype html><html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width, initial-scale=1'>"
        "<meta http-equiv='refresh' content='2'>"
        "<title>Mission Queue</title><style>{style}</style></head>"
        "<body><div class='card'>{body}</div></body></html>")

def render(body, refresh=False):
    tmpl = REFRESH_PAGE if refresh else PAGE
    return tmpl.format(style=STYLE, body=body)

app = Flask(__name__)

from fill_routes import register_fill_routes
register_fill_routes(app)

@app.route("/")
def index():
    items = "".join("<div>{0}</div>".format(s) for s in sorted(SEATS))
    return render("<h1>Callable Sorting Bin</h1>"
                  "<p class='sub'>Server online. Scan a seat tag to call the robot.</p>"
                  "<p class='coord'>Loaded seats:<br>{0}</p>"
                  "<a class='link' href='/queue'>View mission queue &rarr;</a>".format(items))

@app.route("/call")
def call():
    seat = (request.args.get("seat") or "").upper().strip()
    if seat not in SEATS:
        return render("<h1>Unknown seat</h1><p class='sub'>'{0}' is not a recorded seat.</p>".format(seat)), 404
    s = SEATS[seat]
    body = ("<h1>NFC Tag Scanned</h1><p class='sub'>Seat identified</p>"
            "<div class='seat'>{0}</div>"
            "<div class='coord'>x={1:.3f}  y={2:.3f}</div>"
            "<a class='btn' href='/dispatch?seat={0}'>Call Robot</a>"
            "<hr>"
            "<iframe src='/fill_widget' "
            "style='width:100%; height:380px; border:0; border-radius:14px;'>"
            "</iframe>"
            "<a class='link' href='/queue'>View mission queue &rarr;</a>").format(seat, s["x"], s["y"])    
    return render(body)

@app.route("/dispatch")
def dispatch():
    seat = (request.args.get("seat") or "").upper().strip()
    if seat not in SEATS:
        return render("<h1>Unknown seat</h1><p class='sub'>'{0}' is not a recorded seat.</p>".format(seat)), 404
    with _qlock:
        if seat == _current["seat"] or seat in _pending:
            ahead = -1
        else:
            _pending.append(seat)
            mission_q.put(seat)
            ahead = (len(_pending) - 1) + (1 if _current["seat"] else 0)
    if ahead < 0:
        body = ("<div class='ok'>&#8635;</div><h1>Already in queue</h1>"
                "<p class='msg'>Seat <b>{0}</b> is already running or waiting.</p>"
                "<a class='link' href='/queue'>View mission queue &rarr;</a>").format(seat)
    elif ahead == 0:
        body = ("<div class='ok'>&#10003;</div><h1>Robot is on its way</h1>"
                "<p class='msg'>Going to <b>Seat {0}</b>, scanning the waste, opening the "
                "right compartment, and returning HOME.</p>"
                "<a class='link' href='/queue'>View mission queue &rarr;</a>").format(seat)
    else:
        body = ("<div class='ok'>&#10003;</div><h1>Added to queue</h1>"
                "<p class='msg'>Seat <b>{0}</b> queued &mdash; {1} mission(s) ahead. "
                "The robot handles them one at a time.</p>"
                "<a class='link' href='/queue'>View mission queue &rarr;</a>").format(seat, ahead)
    return render(body)

@app.route("/queue")
def queue_view():
    with _qlock:
        cur = _current["seat"]
        since = _current["since"]
        pend = list(_pending)
        hist = list(_history)
    rows = []
    if cur:
        el = int(time.time() - since) if since else 0
        rows.append("<div class='row'><span class='run'>&#9654; Seat {0}</span>"
                    "<span class='run'>running {1}s</span></div>".format(cur, el))
    for i, s in enumerate(pend):
        rows.append("<div class='row'><span class='wait'>Seat {0}</span>"
                    "<span class='wait'>waiting #{1}</span></div>".format(s, i + 1))
    if not cur and not pend:
        rows.append("<div class='row'><span class='done'>idle</span>"
                    "<span class='done'>no missions</span></div>")
    for s, rc in hist:
        cls = "done" if rc == 0 else "fail"
        label = "done" if rc == 0 else "failed"
        rows.append("<div class='row'><span class='{0}'>Seat {1}</span>"
                    "<span class='{0}'>{2} (rc={3})</span></div>".format(cls, s, label, rc))
    body = ("<h1>Mission Queue</h1><p class='sub'>Live &mdash; one mission at a time</p>"
            "{0}<a class='link' href='/'>&larr; Home</a>").format("".join(rows))
    return render(body, refresh=True)

@app.route("/status")
def status():
    state = "n/a"
    try:
        state = str(mb_client.get_state())
    except Exception:
        pass
    with _qlock:
        cur = _current["seat"]
        pend = list(_pending)
    return render("<h1>Status</h1><p class='sub'>current: {0}<br>pending: {1}<br>"
                  "move_base state: {2}</p>".format(cur, ", ".join(pend) or "none", state))

if __name__ == "__main__":
    rospy.init_node("nfc_caller", disable_signals=True)
    mb_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    print("waiting for move_base action server ...")
    if mb_client.wait_for_server(rospy.Duration(10.0)):
        print("connected to move_base.")
    else:
        print("WARNING: move_base not found yet (is the nav stack up?). Server still starting.")
    worker = threading.Thread(target=mission_worker)
    worker.daemon = True
    worker.start()
    print("mission queue worker started (serialized: one mission at a time)")
    print("NFC caller listening on http://192.168.0.156:5000")
    app.run(host="0.0.0.0", port=5000, threaded=True)
