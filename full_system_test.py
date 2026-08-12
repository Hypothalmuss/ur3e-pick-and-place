"""End-to-end acceptance for the whole cell.

Every physical claim is checked against /gazebo/model_states, never against
perception - perception is one of the things under test.
"""
import json
import math
import threading
import time
import urllib.request

import rclpy
from gazebo_msgs.msg import ModelStates
from rclpy.node import Node

BASE = 'http://127.0.0.1:8080'
DZ_X, DZ_Y, HALF = 0.30, 0.22, 0.08
results = []


# ---------------------------------------------------------------- ground truth
class Truth(Node):
    def __init__(self):
        super().__init__('full_test_truth')
        self.create_subscription(ModelStates, '/gazebo/model_states', self._cb, 10)
        self.pose = {}

    def _cb(self, msg):
        self.pose = {n: (p.position.x, p.position.y, p.position.z)
                     for n, p in zip(msg.name, msg.pose) if n.startswith('cube')}


rclpy.init()
_truth = Truth()
threading.Thread(target=lambda: rclpy.spin(_truth), daemon=True).start()


def truth():
    return dict(_truth.pose)


def inside(x, y):
    return abs(x - DZ_X) <= HALF and abs(y - DZ_Y) <= HALF


# ---------------------------------------------------------------- http helpers
def state():
    with urllib.request.urlopen(BASE + '/api/state', timeout=5) as r:
        return json.load(r)


def post(task, x=0.0, y=0.0, ids=None):
    body = json.dumps({'task': task, 'x': x, 'y': y, 'ids': ids or []}).encode()
    req = urllib.request.Request(BASE + '/api/task', data=body,
                                 headers={'Content-Type': 'application/json'},
                                 method='POST')
    try:
        with urllib.request.urlopen(req, timeout=12) as r:
            return json.load(r)
    except urllib.error.HTTPError as e:
        return json.load(e)


def wait_idle(timeout=300):
    time.sleep(2.5)                      # let the 0.5 s tick pick the task up
    t0 = time.time()
    while time.time() - t0 < timeout:
        s = state()
        if (not s.get('busy') and s.get('state') == 'WAITING'
                and not (s.get('tidy') or {}).get('active')):
            return s
        time.sleep(1.0)
    return state()


def check(name, ok, detail=''):
    results.append((name, ok))
    print(f'{"PASS" if ok else "FAIL"}  {name:38} {detail}', flush=True)


# ---------------------------------------------------------------------- checks
print('=== 1. startup and detection ===')
s = state()
check('orchestrator online', s.get('orchestrator_online'))
check('camera online', s.get('camera_online'))
check('3 cubes detected', len(s.get('cubes') or []) == 3,
      f"got {len(s.get('cubes') or [])}")
check('all cubes in perfect zone',
      all(c['zone'] == 'perfect' for c in s.get('cubes', [])))
check('zones published', bool(s.get('zones')) and bool(s.get('drop_zone')))

print('\n=== 2. rejections ===')
check('unknown task refused', not post('fly')['accepted'])
check('out-of-reach pick_to refused', not post('pick_to', 0.9, 0.9)['accepted'])
check('empty selection refused', not post('pick_selected')['accepted'])
check('unknown cube id refused',
      not post('pick_selected', ids=[9999])['accepted'])

print('\n=== 3. simple motions ===')
for task, verify in (
        ('close_gripper', lambda s: s['gripper'] > 0.25),
        ('open_gripper', lambda s: s['gripper'] < 0.05),
        ('retract', lambda s: abs(s['joints']['shoulder_pan_joint'] - 1.5707) < 0.25),
        ('home', lambda s: abs(s['joints']['shoulder_pan_joint']) < 0.2)):
    r = post(task)
    s = wait_idle(120) if r['accepted'] else state()
    check(task, r['accepted'] and verify(s), f"result={s.get('last_result')!r}")

print('\n=== 4. spawn a cube and pick that specific one ===')
before = set(truth())
r = post('spawn_cube')
check('spawn accepted', r['accepted'], r['message'])
time.sleep(6)
new = set(truth()) - before
check('cube exists in gazebo', len(new) == 1, f'new models: {sorted(new)}')
s = state()
check('4 cubes now detected', len(s.get('cubes') or []) == 4,
      f"got {len(s.get('cubes') or [])}")

spawned = sorted(new)[0] if new else None
target_id = None
if spawned:
    sx, sy, _ = truth()[spawned]
    # match the perception id nearest the spawned cube's true position
    best = min(s.get('cubes', []),
               key=lambda c: math.hypot(c['x'] - sx, c['y'] - sy), default=None)
    if best and math.hypot(best['x'] - sx, best['y'] - sy) < 0.05:
        target_id = best['id']
check('spawned cube has a perception id', target_id is not None,
      f'id={target_id}')

if target_id is not None:
    others = {n: p[:2] for n, p in truth().items() if n != spawned}
    r = post('pick_selected', ids=[target_id])
    check('pick_selected accepted', r['accepted'], r['message'])
    wait_idle(300)
    time.sleep(3)
    t = truth()
    if spawned in t:
        x, y, _ = t[spawned]
        check('spawned cube placed in zone', inside(x, y),
              f'({x:+.3f},{y:+.3f})')
    moved = [n for n, (ox, oy) in others.items()
             if n in t and math.hypot(t[n][0] - ox, t[n][1] - oy) > 0.03]
    check('other cubes untouched by selection', not moved, f'moved: {moved}')

print('\n=== 5. emergency stop mid-motion ===')
post('tidy')
time.sleep(7)
s = state()
was_busy = s.get('busy')
j0 = dict(s.get('joints', {}))
r = post('estop')
check('estop accepted mid-task', r['accepted'] and was_busy, r['message'])
time.sleep(1.5)
s1 = state()
j1 = dict(s1.get('joints', {}))
check('state is ESTOP and latched',
      s1.get('state') == 'ESTOP' and s1.get('estopped'))
time.sleep(4)
j2 = dict(state().get('joints', {}))
drift = max((abs(j2[k] - j1[k]) for k in j1 if k in j2), default=0.0)
check('arm holds position after estop', drift < 0.02, f'drift={drift:.4f} rad')
check('commands refused while latched',
      not post('home')['accepted'] and not post('tidy')['accepted'])

print('\n=== 6. reset ===')
r = post('reset')
check('reset accepted', r['accepted'], r['message'])
s = wait_idle(180)
check('reset clears latch and homes',
      not s.get('estopped') and s.get('state') == 'WAITING'
      and abs(s['joints']['shoulder_pan_joint']) < 0.2,
      f"result={s.get('last_result')!r}")
check('accepts work after reset', post('home')['accepted'])
wait_idle(120)

print('\n=== 7. pick_to a chosen point ===')
tx, ty = 0.35, 0.25
r = post('pick_to', tx, ty)
if r['accepted']:
    wait_idle(300)
    time.sleep(3)
    near = [p for p in truth().values() if math.hypot(p[0] - tx, p[1] - ty) < 0.06]
    check('pick_to lands a cube at the point', bool(near),
          f'nearest={min((math.hypot(p[0]-tx,p[1]-ty) for p in truth().values()), default=-1)*1000:.0f}mm')
else:
    check('pick_to lands a cube at the point', False, r['message'])

print('\n=== 8. tidy: pick all remaining ===')
r = post('tidy')
check('tidy accepted', r['accepted'], r['message'])
wait_idle(600)
time.sleep(4)
t = truth()
n_in = sum(inside(x, y) for x, y, _ in t.values())
check('every cube ends inside the square', n_in == len(t),
      f'{n_in}/{len(t)} inside')

# Inside the square is not enough: three cubes stacked in slot 1 are all
# 'inside'. This check was missing and the suite passed a run that stacked
# two cubes at 0 mm separation.
ps = [(x, y) for x, y, _ in t.values()]
closest = min((math.hypot(a[0] - b[0], a[1] - b[1])
               for i, a in enumerate(ps) for b in ps[i + 1:]), default=9.9)
check('placed cubes are not stacked', closest > 0.06,
      f'closest pair {closest*1000:.0f}mm (cube is 60mm)')

print('\n=== 9. run health ===')
s = state()
check('no estop left engaged', not s.get('estopped'))
check('ends idle', s.get('state') == 'WAITING' and not s.get('busy'))

print()
bad = [n for n, ok in results if not ok]
print(f'{len(results) - len(bad)}/{len(results)} passed')
if bad:
    print('FAILED:')
    for n in bad:
        print('  -', n)
