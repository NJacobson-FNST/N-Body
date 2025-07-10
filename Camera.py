import pygame
import numpy as np

camera_offset = np.array([0.0, 0.0])
zoom = 1.0
zoom_speed = 0.1
pan_speed = 20

def world_to_screen(pos, offset, zoom): return ((pos - offset) * zoom).astype(int)

def screen_to_world(pos, offset, zoom): return pos / zoom + offset

def get_visible_bounds(offset, zoom, w, h): return (*offset, *(np.array([w, h]) / zoom))

def update_camera_to_track_body(bodies, tracked_index, zoom, width, height):
    tracked = next((b for b in bodies if b.index == tracked_index), None)
    return tracked.position - (np.array([width /2 , height /2])/zoom)

def update_camera(bodies, tracked_body_index, zoom, width, height, camera_offset):
    if tracked_body_index is not None:
        new_offset = update_camera_to_track_body(bodies, tracked_body_index, zoom, width, height)
        camera_offset = new_offset
    return camera_offset

def render_scene(screen, bodies, acc, trail, step, zoom, offset, font, clock, tracked_index, qt):
    screen.fill((25, 25, 25))
    qt.draw(screen, offset, zoom)
    for i, b in enumerate(bodies):
        screen_pos = world_to_screen(b.position, offset, zoom)
        if i > 0 and step % 20 == 0: #on for mode 1
            if len(trail) >= 10 * len(bodies): trail.pop(0) #on for mode 1
            trail.append((b.position.copy(), b.color)) #on for mode 1
        if np.linalg.norm(acc[i]) > 7500: #on for mode 1
            mag = min(255,int(np.linalg.norm(acc[i])/255)) #on for mode 1
            pygame.draw.line(screen, (mag, mag, mag), screen_pos, world_to_screen(b.position + acc[i] / 255, offset, zoom), 2) #on for mode 1
        pygame.draw.circle(screen, b.color, screen_pos, max(1, int(b.radius * zoom)))
        if tracked_index == b.index:
            pygame.draw.circle(screen, (255, 255, 255), screen_pos, int(b.radius * zoom) + 5, 1)
    [pygame.draw.circle(screen, c, world_to_screen(p, offset, zoom), 1) for p, c in trail]
    screen.blit(font.render(f"FPS: {int(clock.get_fps())}", True, (50, 255, 100)), (10, 10))
    screen.blit(font.render(f"Bodies: {len(bodies)}", True, (100, 200, 255)), (10, 35))
    pygame.display.flip()