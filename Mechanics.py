import numpy as np
from quadtree import QuadTree
from Body import Body
from Camera import get_visible_bounds

def compute_accelerations(bodies, qt, G, ignore_index):
    acc = [np.zeros(2) for _ in bodies]
    for i, b in enumerate(bodies):
        indices = compute_vectors(bodies, qt, i, ignore_index)
        for j in indices:
            r = b.position - bodies[j].position
            d = max(np.linalg.norm(r), b.radius + bodies[j].radius)
            acc[i] += -G * bodies[j].mass * r / (d**3 + 1e-10)
    return acc

def compute_vectors(bodies, qt, i, ignore_index):
    neighbors = qt.query(qt.boundary)
    return [n['index'] for n in neighbors
        if n['index'] != i and n['index'] != ignore_index]

def rk4(bodies, dt, G, offset, zoom):
    def compute_k(state):
        qt = QuadTree(get_visible_bounds(offset, zoom, 1400, 1400), 3)
        [qt.insert(b.to_dict()) for b in state]
        return compute_accelerations(state, qt, G, ignore_index=None)
    orig = [Body(b.mass, b.position.copy(), b.velocity.copy(), b.color, b.index) for b in bodies]
    k1v = [b.velocity.copy() for b in orig]
    k1a = compute_k(orig)
    temp = [Body(b.mass, b.position + 0.5 * dt * k1v[i], b.velocity + 0.5 * dt * k1a[i], b.color, b.index) for i, b in enumerate(orig)]
    k2v = [b.velocity.copy() for b in temp]
    k2a = compute_k(temp)
    temp = [Body(b.mass, b.position + 0.5 * dt * k2v[i], b.velocity + 0.5 * dt * k2a[i], b.color, b.index) for i, b in enumerate(orig)]
    k3v = [b.velocity.copy() for b in temp]
    k3a = compute_k(temp)
    temp = [Body(b.mass, b.position + dt * k3v[i], b.velocity + dt * k3a[i], b.color, b.index) for i, b in enumerate(orig)]
    k4v = [b.velocity.copy() for b in temp]
    k4a = compute_k(temp)
    for i, b in enumerate(bodies):
        b.position = orig[i].position + (dt / 6) * (k1v[i] + 2 * k2v[i] + 2 * k3v[i] + k4v[i])
        b.velocity = orig[i].velocity + (dt / 6) * (k1a[i] + 2 * k2a[i] + 2 * k3a[i] + k4a[i])

def compute_orbital_motion(bodies, dt, screen, offset, zoom, G, qt):
    rk4(bodies, dt, G, offset, zoom)
    qt.clear()
    [qt.insert(b.to_dict()) for b in bodies]
    ignore_index = 0  # Exclude the Sun for visualization in mode 1
    acc = compute_accelerations(bodies, qt, G, ignore_index)
    return acc