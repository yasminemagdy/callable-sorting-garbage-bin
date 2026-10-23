#!/usr/bin/env python3
import time
import statistics
import threading
import atexit
from flask import Flask, jsonify

import RPi.GPIO as GPIO

app = Flask(__name__)

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

SENSORS = {
    "General":    {"trig": 17, "echo": 27},
    "Paper":      {"trig": 22, "echo": 23},
    "Recyclable": {"trig": 24, "echo": 25},
}

BIN_HEIGHT_CM = 30.0
FULL_THRESHOLD_PERCENT = 85.0

read_lock = threading.Lock()

for sensor in SENSORS.values():
    GPIO.setup(sensor["trig"], GPIO.OUT)
    GPIO.setup(sensor["echo"], GPIO.IN)
    GPIO.output(sensor["trig"], False)

time.sleep(2)


def cleanup_gpio():
    GPIO.cleanup()


atexit.register(cleanup_gpio)


def read_distance_cm(trig, echo):
    GPIO.output(trig, False)
    time.sleep(0.00002)

    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)

    timeout = time.time() + 0.04

    while GPIO.input(echo) == 0:
        if time.time() > timeout:
            return None

    pulse_start = time.time()
    timeout = pulse_start + 0.04

    while GPIO.input(echo) == 1:
        if time.time() > timeout:
            return None

    pulse_end = time.time()

    distance = (pulse_end - pulse_start) * 17150.0

    if 2 <= distance <= 400:
        return round(distance, 2)

    return None


def get_stable_distance_cm(trig, echo, samples=5):
    readings = []

    for _ in range(samples):
        distance = read_distance_cm(trig, echo)
        if distance is not None:
            readings.append(distance)
        time.sleep(0.05)

    if not readings:
        return None

    return round(statistics.median(readings), 2)


def fill_percent(distance_cm):
    if distance_cm is None:
        return None

    percent = ((BIN_HEIGHT_CM - distance_cm) / BIN_HEIGHT_CM) * 100.0
    percent = max(0.0, min(100.0, percent))
    return round(percent, 1)


def read_all_levels():
    levels = {}

    with read_lock:
        for name, pins in SENSORS.items():
            distance = get_stable_distance_cm(pins["trig"], pins["echo"])
            fill = fill_percent(distance)

            if distance is None or fill is None:
                levels[name] = {
                    "ok": False,
                    "distance_cm": None,
                    "fill_percent": -1,
                    "full": False,
                    "status": "Sensor Error"
                }
            else:
                levels[name] = {
                    "ok": True,
                    "distance_cm": distance,
                    "fill_percent": fill,
                    "full": fill >= FULL_THRESHOLD_PERCENT,
                    "status": "FULL" if fill >= FULL_THRESHOLD_PERCENT else "OK"
                }

            time.sleep(0.08)

    return levels


@app.route("/")
def index():
    return """
    <h2>Ultrasonic Fill-Level API</h2>
    <p>Use <a href="/api/fill_levels">/api/fill_levels</a> to read bin levels.</p>
    """


@app.route("/api/fill_levels")
def api_fill_levels():
    return jsonify({
        "ok": True,
        "bin_height_cm": BIN_HEIGHT_CM,
        "full_threshold_percent": FULL_THRESHOLD_PERCENT,
        "updated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "levels": read_all_levels()
    })


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5050, debug=False)
