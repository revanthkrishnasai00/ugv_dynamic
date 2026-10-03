"""UGV navigation with UNKNOWN + MOVING obstacles: sense -> replan (A*) -> move.
Same grid/buildings as ugv_simple.py. UGV only sees obstacles within SENSOR_RANGE cells."""
import heapq, math, random
import matplotlib.pyplot as plt

N, CELL, SENSOR_RANGE, MAX_STEPS = 20, 25, 3, 200
START, GOAL = (12, 14), (15, 4)
BUILDINGS = {"IT Block-II": (11,15,14,17), "School of Law": (3,11,5,13), "Football Gnd": (6,5,10,10),
             "Girls Hostel": (3,3,4,4), "Himalaya Blk": (2,0,4,1), "Bolt/Snooker": (8,2,9,3), "MU Building": (14,2,16,3)}
known = {(x, y) for (x1,y1,x2,y2) in BUILDINGS.values() for x in range(x1,x2+1) for y in range(y1,y2+1)}
MOVES = [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]

def actions(s, blocked):
    out = []
    for dx, dy in MOVES:
        n = (s[0]+dx, s[1]+dy)
        if 0 <= n[0] < N and 0 <= n[1] < N and n not in blocked:
            if dx and dy and ((s[0]+dx, s[1]) in blocked or (s[0], s[1]+dy) in blocked): continue
            out.append(n)
    return out

def astar(start, goal, blocked):                       # plans from the CURRENT position
    h = lambda s: math.dist(s, goal)
    g, parent, closed, pq = {start: 0}, {start: None}, set(), [(h(start), start)]
    while pq:
        _, s = heapq.heappop(pq)
        if s in closed: continue
        closed.add(s)
        if s == goal:
            path = []
            while s: path.append(s); s = parent[s]
            return path[::-1]
        for n in actions(s, blocked):
            ng = g[s] + (math.sqrt(2) if n[0] != s[0] and n[1] != s[1] else 1)
            if ng < g.get(n, 1e9): g[n], parent[n] = ng, s; heapq.heappush(pq, (ng + h(n), n))
    return None

def simulate(n_dynamic, seed):
    rnd = random.Random(seed)
    free = [(x, y) for x in range(N) for y in range(N) if (x, y) not in known and math.dist((x, y), START) > 3 and math.dist((x, y), GOAL) > 3]
    rnd.shuffle(free)
    hidden = set(free[:15])                            # unknown static obstacles (~5%)
    dyn = free[15:15 + n_dynamic]                      # moving obstacles (vehicles/people)
    belief = set(known)                                # what the UGV knows (buildings only at start)
    pos, trail, plan = START, [START], None
    replans = waits = collisions = 0; dist = 0.0
    for step in range(MAX_STEPS):
        if pos == GOAL: break
        # 1. SENSE: discover hidden static obstacles and see moving ones in range
        belief |= {c for c in hidden if math.dist(c, pos) <= SENSOR_RANGE}
        seen = {c for c in dyn if math.dist(c, pos) <= SENSOR_RANGE}
        margin = {(x+dx, y+dy) for (x, y) in seen for dx in (-1,0,1) for dy in (-1,0,1)}   # safety margin
        blocked = (belief | margin) - {pos}
        # 2. REPLAN only if no plan, or the next step / remaining path is now blocked
        if plan is None or any(c in blocked for c in plan[1:]):
            plan = astar(pos, GOAL, blocked); replans += 1
        # 3. ACT: move one step, or wait if no path exists right now
        if plan is None or len(plan) < 2: waits += 1; plan = None
        else:
            nxt = plan[1]; dist += math.dist(pos, nxt); pos = nxt; plan = plan[1:]
        trail.append(pos)
        # 4. WORLD CHANGES: moving obstacles take a random step
        for i, (x, y) in enumerate(dyn):
            dx, dy = rnd.choice(MOVES)
            c = (x+dx, y+dy)
            if 0 <= c[0] < N and 0 <= c[1] < N and c not in known and c not in hidden: dyn[i] = c
        if pos in dyn: collisions += 1
    return dict(reached=pos == GOAL, steps=len(trail)-1, dist_m=dist*CELL, replans=replans,
                waits=waits, collisions=collisions, trail=trail, hidden=hidden, dyn=dyn)

LEVELS = {"Low": 3, "Medium": 8, "High": 15}           # number of moving obstacles
static_opt = astar(START, GOAL, known)
opt_m = sum(math.dist(a, b) for a, b in zip(static_opt, static_opt[1:])) * CELL
print(f"Reference: shortest path with ALL obstacles known and static = {opt_m:.1f} m\n")
print(f"{'Level':7}{'Success%':>9}{'Dist(m)':>9}{'Eff%':>7}{'Replans':>9}{'Waits':>7}{'Collis.':>8}")
fig, axes = plt.subplots(1, 3, figsize=(16, 5.5))
for ax, (name, k) in zip(axes, LEVELS.items()):
    runs = [simulate(k, s) for s in range(50)]         # 50 random trials per level
    ok = [r for r in runs if r["reached"]]
    avg = lambda key: sum(r[key] for r in ok) / max(1, len(ok))
    print(f"{name:7}{100*len(ok)/len(runs):9.0f}{avg('dist_m'):9.1f}{100*opt_m/avg('dist_m'):7.1f}"
          f"{avg('replans'):9.1f}{avg('waits'):7.1f}{sum(r['collisions'] for r in runs):8d}")
    r = runs[0]                                        # plot one example run
    for (x, y) in known: ax.add_patch(plt.Rectangle((x-.5, y-.5), 1, 1, color="dimgray"))
    for (x, y) in r["hidden"]: ax.add_patch(plt.Rectangle((x-.5, y-.5), 1, 1, color="salmon"))
    for (x, y) in r["dyn"]: ax.plot(x, y, "m^", ms=9)
    ax.plot([p[0] for p in r["trail"]], [p[1] for p in r["trail"]], "b-o", ms=3)
    ax.plot(*START, "go", ms=11); ax.plot(*GOAL, "o", color="orange", ms=11)
    ax.set_xlim(-.5, N-.5); ax.set_ylim(-.5, N-.5); ax.set_aspect("equal"); ax.grid(alpha=.3)
    ax.set_title(f"{name}: {k} moving obstacles | {r['dist_m']:.0f} m, {r['replans']} replans")
plt.suptitle("Dynamic environment: blue = actual UGV trail, salmon = hidden static, magenta = moving obstacles (final positions)")
plt.tight_layout(); plt.savefig("ugv_dynamic.png", dpi=110)