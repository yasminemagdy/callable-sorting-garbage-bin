from __future__ import print_function

import json
import time

try:
    from urllib2 import urlopen
except ImportError:
    from urllib.request import urlopen

from flask import jsonify, render_template_string


PI_IP = "192.168.0.158"
ULTRASONIC_API = "http://%s:5050/api/fill_levels" % PI_IP


def get_fill_levels():
    try:
        raw = urlopen(ULTRASONIC_API, timeout=8.0).read()

        if not isinstance(raw, str):
            raw = raw.decode("utf-8")

        return json.loads(raw)

    except Exception as e:
        return {
            "ok": False,
            "error": str(e),
            "updated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "levels": {}
        }


FILL_WIDGET_HTML = """
<!doctype html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
body {
    font-family: Arial, sans-serif;
    margin: 12px;
    background: #f7f7f7;
}
.card {
    background: white;
    border-radius: 14px;
    padding: 14px;
    margin-bottom: 12px;
    box-shadow: 0 2px 8px rgba(0,0,0,0.12);
}
.title {
    font-size: 20px;
    font-weight: bold;
    margin-bottom: 8px;
}
.row {
    margin: 10px 0;
}
.label {
    font-weight: bold;
    margin-bottom: 4px;
}
.bar-bg {
    width: 100%;
    height: 22px;
    background: #ddd;
    border-radius: 11px;
    overflow: hidden;
}
.bar {
    height: 100%;
    width: 0%;
    background: #444;
}
.small {
    color: #555;
    font-size: 13px;
}
.error {
    color: #b00020;
    font-weight: bold;
}
.full {
    color: #b00020;
    font-weight: bold;
}
.ok {
    color: #087f23;
    font-weight: bold;
}
button {
    padding: 10px 14px;
    border: 0;
    border-radius: 8px;
    background: #222;
    color: white;
    font-size: 15px;
}
</style>
</head>
<body>

<div class="card">
    <div class="title">Bin Fill Levels</div>
    <div id="levels">Loading...</div>
    <p class="small" id="updated"></p>
    <button onclick="refreshFillLevels()">Refresh</button>
</div>

<script>
function makeRow(name, item) {
    if (!item || item.ok === false || item.fill_percent < 0) {
        return `
        <div class="row">
            <div class="label">${name}</div>
            <div class="error">Sensor Error</div>
        </div>`;
    }

    let fill = item.fill_percent;
    let statusClass = item.full ? "full" : "ok";

    return `
    <div class="row">
        <div class="label">${name}: ${fill}% <span class="${statusClass}">(${item.status})</span></div>
        <div class="bar-bg">
            <div class="bar" style="width:${fill}%"></div>
        </div>
        <div class="small">Distance: ${item.distance_cm} cm</div>
    </div>`;
}

function refreshFillLevels() {
    fetch("/fill_levels")
        .then(response => response.json())
        .then(data => {
            if (!data.ok) {
                document.getElementById("levels").innerHTML =
                    `<div class="error">Cannot read ultrasonic server: ${data.error || "unknown error"}</div>`;
                document.getElementById("updated").innerText = "Updated: " + (data.updated_at || "");
                return;
            }

            const order = ["General", "Paper", "Recyclable"];
            let html = "";

            for (let i = 0; i < order.length; i++) {
                let name = order[i];
                html += makeRow(name, data.levels[name]);
            }

            document.getElementById("levels").innerHTML = html;
            document.getElementById("updated").innerText = "Updated: " + data.updated_at;
        })
        .catch(err => {
            document.getElementById("levels").innerHTML =
                `<div class="error">Fill-level request failed: ${err}</div>`;
        });
}

refreshFillLevels();
setInterval(refreshFillLevels, 8000);
</script>

</body>
</html>
"""


def register_fill_routes(app):
    @app.route("/fill_levels")
    def fill_levels():
        return jsonify(get_fill_levels())

    @app.route("/fill_widget")
    def fill_widget():
        return render_template_string(FILL_WIDGET_HTML)

    @app.route("/dashboard")
    def dashboard():
        return render_template_string(FILL_WIDGET_HTML)
