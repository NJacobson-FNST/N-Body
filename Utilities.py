import pygame
import numpy as np
from Body import Body

G = 6.67430e-11
WIDTH, HEIGHT = 1400, 1400
BLACK = (15, 15, 15)
SCALE = 1e-9
dt = 0.0001 #3-4 less orders of magnitude for mode 1
PLANETS_DATA = [(1.989e30, (255, 255, 0)), (3.285e23, (169, 169, 169)), (4.867e24, (255, 239, 213)),(5.972e24, (0, 128, 0)), (6.390e23, (255, 0, 0)), (1.898e27, (255, 165, 0)),(5.683e26, (210, 180, 140)), (8.681e25, (173, 216, 230)), (1.024e26, (0, 0, 180)),(8.302e27, (128, 0, 128))]

def generate_solar_system():
    bodies = []
    semi_axes = 50 + (np.arange(1, len(PLANETS_DATA)) ** 2 * 5)
    ecc = np.random.rand(len(PLANETS_DATA) - 1) * 0.1
    theta = np.random.rand(len(PLANETS_DATA) - 1) * 2 * np.pi
    x = WIDTH / 2 + semi_axes * (np.cos(theta) - ecc)
    y = HEIGHT / 2 + semi_axes * np.sqrt(1 - ecc**2) * np.sin(theta)
    for i, (m, c) in enumerate(PLANETS_DATA):
        pos = [WIDTH / 2, HEIGHT / 2] if i == 0 else [x[i-1], y[i-1]]
        v = np.sqrt(G * PLANETS_DATA[0][0] * SCALE * (1 + ecc[i-1]) / semi_axes[i-1]) if i > 0 else 0
        vel = [0, 0] if i == 0 else [-v * np.sin(theta[i-1]), v * np.cos(theta[i-1])]
        radius = max(2, int(np.log10(m * SCALE * SCALE) * 1.0 - 1))
        bodies.append(Body(m * SCALE, pos, vel, c, index=i, radius=radius))
    return bodies
    
def generate_mass_cloud(count,mass_range, velocity_magnitude, area_bounds=(WIDTH, HEIGHT),seed=None):
    if seed is not None:
        np.random.seed(seed)
        print(seed)
    bodies = []
    for i in range(count):
        mass = np.random.uniform(*mass_range)
        position = np.random.rand(2) * area_bounds + (WIDTH/4)
        angle = np.random.rand() * 2 * np.pi
        speed = np.random.rand() * velocity_magnitude
        velocity = speed * np.array([np.cos(angle), np.sin(angle)])
        color = tuple(np.random.randint(50, 255) for _ in range(3))
        radius = max(1, int(np.log10(mass * SCALE * SCALE) * 0.5))
        bodies.append(Body(mass, position, velocity, color, index=i, radius=radius))
    return bodies

def handle_input(camera_offset, zoom, zoom_speed, pan_speed, tracked_body_index, bodies):
    keys = pygame.key.get_pressed()
    camera_offset += zoom**-1 * pan_speed * np.array([keys[pygame.K_RIGHT] - keys[pygame.K_LEFT], keys[pygame.K_DOWN] - keys[pygame.K_UP]])
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            return False, camera_offset, zoom, tracked_body_index
        elif event.type == pygame.MOUSEWHEEL:
            zoom *= 1 + zoom_speed * event.y
            zoom = max(0.01, min(zoom, 100))
        elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            mouse_pos = np.array(pygame.mouse.get_pos())
            world_pos = mouse_pos / zoom + camera_offset
            for b in bodies:
                if np.linalg.norm(world_pos - b.position) < b.radius:
                    tracked_body_index = b.index
                    break
            else:
                tracked_body_index = None
    return True, camera_offset, zoom, tracked_body_index