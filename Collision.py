import numpy as np
from Body import Body

def detect_collisions(bodies, qt):
    collisions = set()
    for b in bodies:
        region = (*b.position - b.radius*2, b.radius*4, b.radius*4)
        nearby = qt.query(region)
        for o in nearby:
            if o['index'] == b.index: continue
            dist2 = np.dot(b.position - o['pos'], b.position - o['pos'])
            rad_sum2 = (b.radius + o['radius'])**2
            if dist2 < rad_sum2:
                pair = tuple(sorted((b.index, o['index'])))
                collisions.add(pair)
    return list(collisions)

def handle_collisions(bodies, collisions, tracker, step, delay, tracked_index):
    collisions = set(collisions)
    unique_pairs = {tuple(sorted((i, j))) for i, j in collisions}
    to_merge = []
    still_colliding = set()
    for k in unique_pairs:
        if k not in tracker:
            tracker[k] = step
            still_colliding.add(k)
    for k in list(tracker.keys()):
        if tracker[k] + delay <= step:
            if k in unique_pairs:
                to_merge.append(k)
                tracker.pop(k, None)
            else:
                tracker.pop(k, None)
                unique_pairs.discard(k)
                collisions.discard(k)
        else:
            still_colliding.add(k)
    merged = set()
    for i, j in to_merge:
        if i in merged or j in merged:
            continue
        tracked_index = min(i, j) if tracked_index in (i, j) else tracked_index
        tracked_index = merge_bodies(bodies, i, j, tracked_index)
        merged.update((i, j))
    tracker = {k: v for k, v in tracker.items() if k in still_colliding}
    return tracked_index

def merge_bodies(bodies, i, j, tracked_index):
    try:
        if bodies[j].mass > bodies[i].mass:
            i, j = j, i
        b1, b2 = bodies[i], bodies[j]
        m1, m2 = b1.mass, b2.mass
        total_mass = m1 + m2
        new_pos = (m1 * b1.position + m2 * b2.position) / total_mass
        new_vel = (m1 * b1.velocity + m2 * b2.velocity) / total_mass
        new_color = tuple(int((b1.color[k] * m1 + b2.color[k] * m2) / total_mass) for k in range(3))
        new_radius = (b1.radius**3 + b2.radius**3)**(1/3) - 1
        bodies[i] = Body(total_mass, new_pos, new_vel, new_color, index=i, radius=new_radius)
        if tracked_index and tracked_index > i:
            tracked_index -= 1
        del bodies[j]
        for idx, b in enumerate(bodies):
            b.index = idx
        return tracked_index
    except IndexError:
        print(f"IndexError: Cannot merge bodies at indices {i} and {j}")
        return None 