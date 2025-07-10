import pygame, numpy as np

class QuadTree:
    def __init__(self, boundary, capacity):
        self.boundary, self.capacity = boundary, capacity
        self.bodies, self.divided = [], False
        self.total_mass, self.center_of_mass = 0, np.zeros(2)
    def subdivide(self):
        x, y, w, h = self.boundary
        hw, hh = w / 2, h / 2
        self.nw, self.ne = QuadTree((x, y, hw, hh), self.capacity), QuadTree((x + hw, y, hw, hh), self.capacity)
        self.sw, self.se = QuadTree((x, y + hh, hw, hh), self.capacity), QuadTree((x + hw, y + hh, hw, hh), self.capacity)
        self.divided = True
    def contains(self, body):
        x, y, w, h = self.boundary
        bx, by = body['pos']
        return x <= bx < x + w and y <= by < y + h
    def insert(self, body):
        if not self.contains(body): return False
        if len(self.bodies) < self.capacity:
            self.bodies.append(body); self._update_mass_center(); return True
        if not self.divided: self.subdivide()
        if any(q.insert(body) for q in [self.nw, self.ne, self.sw, self.se]):
            self._update_mass_center(); return True
        return False
    def _update_mass_center(self):
        if not self.bodies: return
        self.total_mass = sum(b['mass'] for b in self.bodies)
        if self.total_mass: self.center_of_mass = sum(b['mass'] * b['pos'] for b in self.bodies) / self.total_mass
    def query(self, r):
        if not self.intersects(r): return []
        found = [b for b in self.bodies if r[0] <= b['pos'][0] < r[0] + r[2] and r[1] <= b['pos'][1] < r[1] + r[3]]
        if self.divided:
            for qt in [self.nw, self.ne, self.sw, self.se]: found.extend(qt.query(r))
        return found
    def intersects(self, r):
        x, y, w, h = self.boundary
        rx, ry, rw, rh = r
        return not (rx > x + w or rx + rw < x or ry > y + h or ry + rh < y)
    def draw(self, screen, camera_offset, zoom):
        x, y, w, h = self.boundary
        top_left = ((np.array([x, y]) - camera_offset) * zoom).astype(int)
        size = (np.array([w, h]) * zoom).astype(int)
        pygame.draw.rect(screen, (5, 5, 5), (*top_left, *size), 1)
        if self.divided:
            for qt in [self.nw, self.ne, self.sw, self.se]:
                qt.draw(screen, camera_offset, zoom)
    def clear(self):
        self.bodies, self.total_mass = [], 0
        self.center_of_mass = np.zeros(2)
        if self.divided:
            for qt in [self.nw, self.ne, self.sw, self.se]: qt.clear()
            self.nw = self.ne = self.sw = self.se = None
            self.divided = False
    def compute_force(self, body, G, theta):
        if not self.bodies and not self.divided: return np.zeros(2)
        dx, dy = self.center_of_mass - body['pos']
        dist = np.hypot(dx, dy) + 1e-5
        width = self.boundary[2]
        if (not self.divided and self.bodies and self.bodies[0]['index'] != body['index']) or (width / dist < theta):
            if self.total_mass and self.bodies[0]['index'] != body['index']:
                return G * self.total_mass / dist**2 * np.array([dx, dy]) / dist
        return sum((qt.compute_force(body, G, theta) for qt in [self.nw, self.ne, self.sw, self.se]), np.zeros(2)) if self.divided else np.zeros(2)