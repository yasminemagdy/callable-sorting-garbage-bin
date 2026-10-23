import RPi.GPIO as GPIO
import time
import statistics

GPIO.setmode(GPIO.BCM)

SENSORS = {
    "General": {"trig": 17, "echo": 27},
    "Paper":    {"trig": 22, "echo": 23},
    "Recyclable":      {"trig": 24, "echo": 25},
}

BIN_HEIGHT = 30.0  # cm, sensor to empty bin floor

for s in SENSORS.values():
    GPIO.setup(s["trig"], GPIO.OUT)
    GPIO.setup(s["echo"], GPIO.IN)
    GPIO.output(s["trig"], False)

print("Initializing... wait 2s")
time.sleep(2)

def read_distance(trig, echo):
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)
    timeout = time.time() + 0.04
    pulse_start = pulse_end = None
    while GPIO.input(echo) == 0:
        pulse_start = time.time()
        if pulse_start > timeout:
            return None
    while GPIO.input(echo) == 1:
        pulse_end = time.time()
        if pulse_end > timeout:
            return None
    if pulse_start is None or pulse_end is None:
        return None
    d = (pulse_end - pulse_start) * 17150
    return round(d, 2) if 2 <= d <= 400 else None

def get_stable_distance(trig, echo, samples=5):
    readings = [d for _ in range(samples) if (d := read_distance(trig, echo)) is not None and not time.sleep(0.05)]
    return round(statistics.median(readings), 2) if readings else None

def fill_percent(distance):
    if distance is None:
        return -1
    return round(max(0, min(100, ((BIN_HEIGHT - distance) / BIN_HEIGHT) * 100)), 1)

try:
    while True:
        print("\n==============================")
        print("       BIN FILL LEVELS")
        print("==============================")
        for name, pins in SENSORS.items():
            d = get_stable_distance(pins["trig"], pins["echo"])
            f = fill_percent(d)
            if d is None:
                print(f"{name:<12} | Sensor Error")
            else:
                print(f"{name:<12} | Distance: {d:>6} cm | Fill: {f:>5}%")
            time.sleep(0.1)
        time.sleep(1)
except KeyboardInterrupt:
    print("\nStopping...")
finally:
    GPIO.cleanup()
    print("GPIO cleaned up.")
