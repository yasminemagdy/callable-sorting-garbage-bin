#!/usr/bin/env python3
import sys, cv2, numpy as np
from PIL import Image
from tflite_runtime.interpreter import Interpreter
CAMERA_DEV = "/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0"
MODEL_PATH = "/home/ubuntu/waste_ai/hala.tflite"
LABELS = ["general_waste", "paper", "recyclable"]
BOX = 300            # center crop size (matches tested script)
WARMUP = 10          # discard frames for auto-exposure
AVG_FRAMES = 5       # frames to average for stability
def classify_once(save_debug=True):
    interp = Interpreter(model_path=MODEL_PATH, num_threads=4)
    interp.allocate_tensors()
    inp = interp.get_input_details()
    out = interp.get_output_details()
    H = int(inp[0]["shape"][1]); W = int(inp[0]["shape"][2])
    cap = cv2.VideoCapture(CAMERA_DEV, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        print("ERROR: camera not opened", file=sys.stderr); sys.exit(2)
    for _ in range(WARMUP):
        cap.read()
    probs_sum = None; last_crop = None; shape = None
    for _ in range(AVG_FRAMES):
        ok, frame = cap.read()
        if not ok: continue
        shape = frame.shape
        h, w, _ = frame.shape
        x1 = (w - BOX)//2; y1 = (h - BOX)//2
        crop = frame[y1:y1+BOX, x1:x1+BOX]
        last_crop = crop
        rgb = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(rgb).resize((W, H))
        x = np.expand_dims(np.array(img).astype(np.float32), axis=0)
        interp.set_tensor(inp[0]["index"], x)
        interp.invoke()
        p = interp.get_tensor(out[0]["index"])[0]
        probs_sum = p if probs_sum is None else probs_sum + p
    cap.release()
    if probs_sum is None:
        print("ERROR: no frames captured", file=sys.stderr); sys.exit(3)
    probs = probs_sum / AVG_FRAMES
    idx = int(np.argmax(probs))
    label = LABELS[idx]; conf = float(probs[idx])
    if save_debug and last_crop is not None:
        cv2.imwrite("/home/ubuntu/waste_ai/debug_crop.jpg", last_crop)
    print("FRAME_SHAPE", shape, file=sys.stderr)
    print("PROBS " + " ".join(f"{LABELS[i]}={probs[i]:.2f}" for i in range(len(LABELS))), file=sys.stderr)
    return label, conf, probs
if __name__ == "__main__":
    label, conf, _ = classify_once()
    print(label)          # final stdout line = clean label for the orchestrator
