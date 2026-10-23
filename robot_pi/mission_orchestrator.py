#!/usr/bin/env python3
import rospy, sys, os, subprocess, signal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalStatusArray
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion

HOME_DIR = os.path.expanduser("~")
BASE_FILE = os.path.join(HOME_DIR, ".compartment_base_yaw.txt")
STATE_FILE = os.path.join(HOME_DIR, ".current_compartment.txt")
CLASSIFY = ["python3", os.path.join(HOME_DIR, "waste_ai/classify_once.py")]
ROTATE = os.path.join(HOME_DIR, "adaptive_compartment_rotate.py")
SERVO = ["python3", os.path.join(HOME_DIR, "visual_servo_home.py")]
ROTATE_HOME = ["python3", os.path.join(HOME_DIR, "rotate_to_home_yaw.py")]
ORBBEC_LAUNCH = os.path.join(HOME_DIR, "kobuki_launch/orbbec_rgb.launch")

SETTLE_AFTER_NAV_SEC = 4.0
DISPOSAL_WAIT_SEC = 3.0
SETTLE_BEFORE_SERVO_SEC = 2.0
GOAL_TIMEOUT_SEC = 90.0
CAMERA_WARMUP_SEC = 6.0
NAV_ATTEMPTS = 2

HOME_TAG_FACING = (0.130063062364, -0.0399486733446, -0.998949108472, 0.0458331613886)
BUILTIN = {"C": (2.117, 0.149, -0.755, 0.656)}

current_yaw = None
latest_status = None

def state(msg):
    print("\n========== [STATE] %s ==========" % msg, flush=True)

def info(msg):
    print("   -> " + str(msg), flush=True)

def amcl_cb(msg):
    global current_yaw
    q = msg.pose.pose.orientation
    current_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

def status_cb(msg):
    global latest_status
    if msg.status_list:
        latest_status = msg.status_list[-1].status

def clear_costmaps():
    try:
        srv = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)
        srv.wait_for_service(timeout=3.0)
        srv()
        info("costmaps cleared")
    except Exception as e:
        info("clear_costmaps failed: %s" % e)

def _publish_goal_and_wait(pub, name, pose):
    global latest_status
    x, y, z, w = pose
    clear_costmaps()
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.orientation.z = z
    goal.pose.orientation.w = w
    t0 = rospy.Time.now().to_sec()
    while pub.get_num_connections() == 0 and rospy.Time.now().to_sec() - t0 < 5.0:
        rospy.sleep(0.1)
    rospy.sleep(0.3)
    latest_status = None
    pub.publish(goal)
    info("goal published, waiting for move_base...")
    start = rospy.Time.now().to_sec()
    saw_active = False
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        s = latest_status
        if s == 1 and not saw_active:
            saw_active = True
            info("move_base ACTIVE (driving)")
        if saw_active and s == 3:
            info("move_base SUCCEEDED")
            return True
        if saw_active and s in (4, 5):
            info("move_base returned status %s" % s)
            return False
        if rospy.Time.now().to_sec() - start > GOAL_TIMEOUT_SEC:
            info("NAVIGATION TIMEOUT")
            return False
        rate.sleep()
    return False

def go_to(pub, name, pose):
    x, y, z, w = pose
    state("NAVIGATING to %s" % name)
    info("target x=%.3f y=%.3f qz=%.3f qw=%.3f" % (x, y, z, w))
    for attempt in range(1, NAV_ATTEMPTS + 1):
        if attempt > 1:
            info("retry %d/%d (clearing costmaps)" % (attempt, NAV_ATTEMPTS))
        if _publish_goal_and_wait(pub, name, pose):
            state("ARRIVED at %s" % name)
            return True
    state("NAVIGATION FAILED at %s" % name)
    return False

def wait_yaw():
    start = rospy.Time.now().to_sec()
    while not rospy.is_shutdown() and current_yaw is None:
        if rospy.Time.now().to_sec() - start > 10.0:
            info("ERROR: no /amcl_pose received"); sys.exit(1)
        rospy.sleep(0.1)

def set_reference_to_arrival():
    state("CAPTURING arrival reference (recyclable-facing-user)")
    wait_yaw()
    with open(BASE_FILE, "w") as f:
        f.write(str(current_yaw))
    with open(STATE_FILE, "w") as f:
        f.write("recyclable")
    info("reference yaw_rad=%.4f saved" % current_yaw)

def classify():
    state("CLASSIFYING waste with camera")
    out = subprocess.run(CLASSIFY, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    err = out.stderr.decode().strip()
    for line in err.splitlines():
        if line.startswith("PROBS") or line.startswith("FRAME_SHAPE"):
            info(line)
    lines = out.stdout.decode().strip().splitlines()
    if not lines:
        info("ERROR: classifier returned nothing"); info(err); sys.exit(2)
    label = lines[-1].strip()
    state("CLASSIFIED as: %s" % label.upper())
    return label

def rotate(label):
    state("ROTATING compartment -> %s" % label)
    subprocess.run(["python3", ROTATE, label])
    info("rotation done")

def start_apriltag_camera():
    info("starting AprilTag camera (orbbec_rgb.launch) - one camera at a time")
    proc = subprocess.Popen(["roslaunch", ORBBEC_LAUNCH],
                            stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL,
                            preexec_fn=os.setsid)
    info("camera warming up %.1fs" % CAMERA_WARMUP_SEC)
    rospy.sleep(CAMERA_WARMUP_SEC)
    return proc

def stop_apriltag_camera(proc):
    if proc is None:
        return
    info("stopping AprilTag camera (freeing USB)")
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=10)
    except Exception as e:
        info("camera stop issue: %s" % e)
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            pass
    rospy.sleep(2.0)

def return_home(pub):
    if not go_to(pub, "HOME (tag-facing)", HOME_TAG_FACING):
        state("MISSION ABORTED (return HOME failed)"); sys.exit(4)
    state("SETTLING %.1fs before visual servo" % SETTLE_BEFORE_SERVO_SEC)
    rospy.sleep(SETTLE_BEFORE_SERVO_SEC)
    cam = start_apriltag_camera()
    try:
        state("VISUAL SERVO docking to HOME")
        rc = subprocess.run(SERVO).returncode
        info("visual_servo_home exit code %s" % rc)
        state("ROTATING to face Majlis")
        subprocess.run(ROTATE_HOME)
        info("home orientation set")
    finally:
        stop_apriltag_camera(cam)

def get_target():
    if len(sys.argv) >= 6:
        return sys.argv[1].upper(), (float(sys.argv[2]), float(sys.argv[3]),
                                     float(sys.argv[4]), float(sys.argv[5]))
    if len(sys.argv) == 2 and sys.argv[1].upper() in BUILTIN:
        n = sys.argv[1].upper()
        return n, BUILTIN[n]
    print("Usage: python3 ~/mission_orchestrator.py <SEAT> [x y qz qw]"); sys.exit(0)

def main():
    name, pose = get_target()
    state("MISSION START -> seat %s" % name)
    rospy.init_node("mission_orchestrator")
    pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, amcl_cb)
    rospy.Subscriber("/move_base/status", GoalStatusArray, status_cb)
    rospy.sleep(1.0)

    if not go_to(pub, name, pose):
        state("MISSION ABORTED (nav to seat failed)"); sys.exit(3)

    state("SETTLING %.1fs for final yaw correction" % SETTLE_AFTER_NAV_SEC)
    rospy.sleep(SETTLE_AFTER_NAV_SEC)

    set_reference_to_arrival()
    label = classify()
    rotate(label)

    state("WAITING %.1fs for user disposal" % DISPOSAL_WAIT_SEC)
    rospy.sleep(DISPOSAL_WAIT_SEC)

    return_home(pub)
    state("MISSION COMPLETE")

if __name__ == "__main__":
    main()
