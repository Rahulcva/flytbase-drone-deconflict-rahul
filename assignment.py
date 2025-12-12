#!/usr/bin/env python3
import rospy, time, os, csv, json, threading, math, traceback
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, Pose
from gazebo_msgs.msg import ModelStates
import numpy as np

# Optional plotting for final PNG
PLOTTING = True
try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa
except Exception:
    PLOTTING = False

# Try to import KDTree for speed in deconflict
try:
    from scipy.spatial import cKDTree
except Exception:
    cKDTree = None

# ---------------- CONFIG ----------------
MISSION_FILE = "multi_mission.json"   # your JSON path
OUT_DIR = os.path.expanduser("~/drone_trajs")
ALL_CSV = os.path.join(OUT_DIR, "all_trajs.csv")
CONFLICTS_JSON = os.path.expanduser("~/conflicts.json")
DECONFLICT_REPORT = os.path.join(OUT_DIR, "deconflict_report.txt")
PRIMARY_PLOT = os.path.join(OUT_DIR, "primary_4d.png")

DT = 0.1            # interpolation timestep (s)
SAFETY = 1.0        # meters threshold for conflict
WAYPOINT_TOL = 0.25
KP = 1.0
MAX_V = 1.5

# ---------- shared state ----------
_shared_positions = {}   # model -> (x,y,z,t)
_shared_lock = threading.Lock()

# ---------------- Recorder ----------------
class Recorder:
    def __init__(self, csv_path=ALL_CSV):
        self.csv_path = csv_path
        os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
        # header
        with open(self.csv_path, 'w') as f:
            csv.writer(f).writerow(['time', 'model', 'x', 'y', 'z'])
        self.lock = threading.Lock()
        # we will dynamically subscribe to any /<name>/gt_pose we find
        self.subs = {}
        # subscribe to model_states as a robust fallback to record even after ground plugins stop publishing gt_pose
        rospy.Subscriber('/gazebo/model_states', ModelStates, self._model_cb, queue_size=20)
        # run a periodic discoverer to subscribe to gt_pose topics if they exist
        self.timer = rospy.Timer(rospy.Duration(1.5), lambda ev: self._discover_gt_pose())
        rospy.loginfo("[Recorder] Initialized, writing to %s", self.csv_path)

    def _discover_gt_pose(self):
        try:
            pubs = rospy.get_published_topics()
        except Exception:
            pubs = []
        for topic, ttype in pubs:
            if topic.endswith('/gt_pose') and topic not in self.subs:
                # try subscribing as geometry_msgs/Pose (most common)
                try:
                    sub = rospy.Subscriber(topic, Pose, self._make_gt_cb(topic), queue_size=20)
                    self.subs[topic] = sub
                    rospy.loginfo("[Recorder] Subscribed to %s", topic)
                except Exception as e:
                    rospy.logwarn("[Recorder] Could not subscribe to %s: %s", topic, str(e))

    def _make_gt_cb(self, topic):
        model = topic.rsplit('/', 1)[0].lstrip('/')
        def cb(msg):
            try:
                x = float(msg.position.x)
                y = float(msg.position.y)
                z = float(msg.position.z)
            except Exception:
                return
            t = rospy.Time.now().to_sec()
            with self.lock:
                with open(self.csv_path, 'a') as f:
                    csv.writer(f).writerow([t, model, x, y, z])
            with _shared_lock:
                _shared_positions[model] = (x, y, z, t)
        return cb

    def _model_cb(self, msg):
        # safe fallback: iterate names and write ardrone* poses
        t = rospy.Time.now().to_sec()
        try:
            with self.lock:
                with open(self.csv_path, 'a') as f:
                    writer = csv.writer(f)
                    for i, name in enumerate(msg.name):
                        if name.startswith('ardrone'):
                            p = msg.pose[i].position
                            writer.writerow([t, name, float(p.x), float(p.y), float(p.z)])
                            with _shared_lock:
                                _shared_positions[name] = (float(p.x), float(p.y), float(p.z), t)
        except Exception as e:
            rospy.logwarn_throttle(5, "[Recorder] model_states write failed: %s", str(e))

    def stop(self):
        try:
            self.timer.shutdown()
        except Exception:
            pass
        rospy.loginfo("[Recorder] stopped")

# ---------------- Drone controller ----------------
class DroneController:
    def __init__(self, name, waypoints, takeoff_alt=1.0, start_delay=0.0, loiter_time=0.0):
        self.name = name
        self.waypoints = waypoints
        self.takeoff_alt = float(takeoff_alt)
        self.start_delay = float(start_delay)
        self.loiter_time = float(loiter_time)

        self.cmd_pub = rospy.Publisher('/%s/cmd_vel' % self.name, Twist, queue_size=1)
        self.takeoff_pub = rospy.Publisher('/%s/takeoff' % self.name, Empty, queue_size=1)
        self.land_pub = rospy.Publisher('/%s/land' % self.name, Empty, queue_size=1)

    def burst_publish(self, pub, n=8, rate_s=0.12):
        for _ in range(n):
            try:
                pub.publish(Empty())
            except Exception:
                pass
            time.sleep(rate_s)

    def publish_twist(self, vx, vy, vz):
        tmsg = Twist()
        tmsg.linear.x = vx
        tmsg.linear.y = vy
        tmsg.linear.z = vz
        try:
            self.cmd_pub.publish(tmsg)
        except Exception:
            pass

    def goto(self, tx, ty, tz, timeout=30.0):
        start = time.time()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and (time.time() - start) < timeout:
            with _shared_lock:
                cur = _shared_positions.get(self.name, None)
            if cur is None:
                # wait for recorder to populate
                time.sleep(0.05)
                continue
            cx, cy, cz, _ = cur
            ex, ey, ez = tx - cx, ty - cy, tz - cz
            dist = math.sqrt(ex*ex + ey*ey + ez*ez)
            if dist <= WAYPOINT_TOL:
                self.publish_twist(0,0,0)
                return True
            vx = KP * ex; vy = KP * ey; vz = KP * ez
            # saturate
            scale = max(1.0, abs(vx)/MAX_V, abs(vy)/MAX_V, abs(vz)/MAX_V)
            vx, vy, vz = vx/scale, vy/scale, vz/scale
            self.publish_twist(vx, vy, vz)
            rate.sleep()
        # stop
        self.publish_twist(0,0,0)
        return False

    def run_mission(self):
        # optional staggered start
        if self.start_delay and self.start_delay > 0.001:
            rospy.loginfo("[%s] start delay %.2fs" % (self.name, self.start_delay))
            time.sleep(self.start_delay)

        rospy.loginfo("[%s] takeoff burst (alt %.2f)" % (self.name, self.takeoff_alt))
        self.burst_publish(self.takeoff_pub, n=8, rate_s=0.12)

        # wait for approximate altitude (simple wait; model_states will record actual heights)
        t0 = time.time()
        while time.time() - t0 < 6.0:
            with _shared_lock:
                cur = _shared_positions.get(self.name, None)
            if cur and cur[2] >= (self.takeoff_alt * 0.6):
                break
            time.sleep(0.1)

        # follow waypoints
        for wp in self.waypoints:
            tx, ty, tz = float(wp['x']), float(wp['y']), float(wp['z'])
            rospy.loginfo("[%s] goto %.2f %.2f %.2f" % (self.name, tx, ty, tz))
            ok = self.goto(tx, ty, tz, timeout=30.0)
            if not ok:
                rospy.logwarn("[%s] waypoint timeout to (%.2f,%.2f,%.2f)" % (self.name, tx,ty,tz))

        # loiter briefly to let recorder catch last positions
        if self.loiter_time and self.loiter_time > 0.01:
            rospy.loginfo("[%s] loitering %.2fs before landing" % (self.name, self.loiter_time))
            t_lo = time.time()
            while time.time() - t_lo < self.loiter_time:
                self.publish_twist(0,0,0)
                time.sleep(0.1)

        # automatic landing - publish multiple times to ensure plugin receives it
        rospy.loginfo("[%s] landing" % self.name)
        self.burst_publish(self.land_pub, n=12, rate_s=0.12)
        # brief wait
        time.sleep(0.5)
        self.publish_twist(0,0,0)

# ---------------- Deconflict functions ----------------
def load_all_csv(csvfile):
    if not os.path.exists(csvfile):
        raise FileNotFoundError(csvfile)
    data = {}
    tmin = float('inf'); tmax = 0.0
    with open(csvfile) as f:
        r = csv.DictReader(f)
        for row in r:
            t = float(row['time']); m = row['model']
            x = float(row['x']); y = float(row['y']); z = float(row['z'])
            data.setdefault(m, []).append((t,x,y,z))
            tmin = min(tmin, t); tmax = max(tmax, t)
    for k in data:
        data[k].sort(key=lambda p: p[0])
    return data, tmin, tmax

def interp_traj(points, tgrid):
    ts = np.array([p[0] for p in points])
    xs = np.array([p[1] for p in points])
    ys = np.array([p[2] for p in points])
    zs = np.array([p[3] for p in points])
    if len(ts) == 0:
        return np.zeros((len(tgrid), 3))
    X = np.vstack([np.interp(tgrid, ts, xs),
                   np.interp(tgrid, ts, ys),
                   np.interp(tgrid, ts, zs)]).T
    return X

def run_deconflict(csvfile, primary, safety=SAFETY, dt=DT, out_json=CONFLICTS_JSON):
    data, tmin, tmax = load_all_csv(csvfile)
    if primary not in data:
        rospy.logwarn("Primary %s not in CSV data." % primary)
        return []
    tgrid = np.arange(tmin, tmax + dt/2.0, dt)
    trajs = {k: interp_traj(v, tgrid) for k,v in data.items()}
    others = [k for k in trajs.keys() if k != primary]
    conflicts = []
    if not others:
        rospy.loginfo("No other drones found for deconflict.")
    for i, t in enumerate(tgrid):
        ppos = trajs[primary][i]
        # query pairwise
        for o in others:
            opos = trajs[o][i]
            d = np.linalg.norm(ppos - opos)
            if d < safety:
                conflicts.append({
                    'time': float(t),
                    'primary_pos': ppos.tolist(),
                    'other': o,
                    'other_pos': opos.tolist(),
                    'dist': float(d)
                })
    with open(out_json, 'w') as f:
        json.dump(conflicts, f, indent=2)
    rospy.loginfo("Deconflict: found %d conflicts. Saved %s" % (len(conflicts), out_json))
    return conflicts

# ---------------- plotting ----------------
def plot_primary(csvfile, primary, out_png=PRIMARY_PLOT):
    if not PLOTTING:
        rospy.logwarn("Matplotlib not available; skipping plot.")
        return
    data, _, _ = load_all_csv(csvfile)
    if primary not in data:
        rospy.logwarn("Primary %s not present; skip plotting." % primary)
        return
    arr = np.array(data[primary])
    t = arr[:,0]; x = arr[:,1]; y = arr[:,2]; z = arr[:,3]
    if len(t) < 5:
        rospy.logwarn("Primary traj too short for plot.")
        return
    fig = plt.figure(figsize=(8,6))
    ax = fig.add_subplot(111, projection='3d')
    sc = ax.scatter(x, y, z, c=t, cmap='viridis', s=10)
    ax.plot(x, y, z, alpha=0.6)
    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
    plt.colorbar(sc, label='time (s)')
    plt.title('Primary trajectory (time color)')
    os.makedirs(OUT_DIR, exist_ok=True)
    fig.savefig(out_png, dpi=150)
    plt.close(fig)
    rospy.loginfo("Saved static plot %s" % out_png)

# ---------------- main pipeline ----------------
def pipeline_main():
    rospy.init_node('takeoff_pipeline', anonymous=False, log_level=rospy.INFO)
    # load mission
    if not os.path.exists(MISSION_FILE):
        rospy.logerr("Mission file missing: %s" % MISSION_FILE)
        return
    with open(MISSION_FILE) as f:
        mission = json.load(f)
    primary = mission.get('primary', 'ardrone_1')
    drones_info = mission.get('drones', {})
    if not drones_info:
        rospy.logerr("No drones in mission JSON")
        return

    # recorder
    recorder = Recorder()
    time.sleep(0.8)  # allow recorder to populate initial positions

    # create controllers
    controllers = []
    for idx, (name, info) in enumerate(drones_info.items()):
        waypoints = info.get('waypoints', [])
        takeoff_alt = info.get('takeoff_alt', 1.0)
        # basic stagger: uncomment if you want automatic staggers
        start_delay = info.get('start_delay', idx * 1.5)
        loiter_time = info.get('loiter_time', 1.5)
        c = DroneController(name, waypoints, takeoff_alt, start_delay=start_delay, loiter_time=loiter_time)
        controllers.append(c)

    # run all mission threads
    threads = []
    for c in controllers:
        t = threading.Thread(target=c.run_mission, daemon=True)
        t.start(); threads.append(t)

    try:
        for t in threads:
            t.join()
    except KeyboardInterrupt:
        rospy.logwarn("Interrupted by user; continuing to collect/finish.")

    # give recorder a moment to flush last entries
    rospy.loginfo("Waiting 2s to flush recorder entries...")
    time.sleep(2.0)
    recorder.stop()

    # deconflict
    rospy.loginfo("Running deconflict (safety=%.2fm) ..." % SAFETY)
    try:
        conflicts = run_deconflict(ALL_CSV, primary, safety=SAFETY)
    except Exception as e:
        rospy.logerr("Deconflict failed: %s\n%s" % (str(e), traceback.format_exc()))
        conflicts = []

    # write report
    os.makedirs(OUT_DIR, exist_ok=True)
    with open(DECONFLICT_REPORT, 'w') as f:
        f.write("Deconflict report\n")
        f.write("Primary: %s\n" % primary)
        f.write("Total conflicts: %d\n" % len(conflicts))
        if conflicts:
            dists = [c['dist'] for c in conflicts]
            f.write("Min dist: %.3f\n" % min(dists))
            f.write("Mean dist: %.3f\n" % (sum(dists)/len(dists)))
    rospy.loginfo("Wrote deconflict report: %s" % DECONFLICT_REPORT)

    # plot
    try:
        plot_primary(ALL_CSV, primary)
    except Exception as e:
        rospy.logwarn("Plotting failed: %s" % str(e))

    rospy.loginfo("Pipeline complete. Files:\n  %s\n  %s\n  %s" % (ALL_CSV, CONFLICTS_JSON, PRIMARY_PLOT))

if __name__ == "__main__":
    pipeline_main()
