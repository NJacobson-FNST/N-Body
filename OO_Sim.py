import pygame, numpy as np, Body, quadtree, Mechanics, Camera, Utilities, Collision

pygame.init()
font = pygame.font.SysFont("Arial", 18)
screen = pygame.display.set_mode((Utilities.WIDTH, Utilities.HEIGHT))
clock = pygame.time.Clock()
trail = []
running = True
step = 0
tracked_body_index = None
collision_tracker = {}
merge_delay_steps = 30 #30-60 for mode 1

bodies = Utilities.generate_solar_system()
#bodies = Utilities.generate_mass_cloud(100, (1e23, 1e27), 0, (Utilities.WIDTH/2, Utilities.HEIGHT/2))

while running:
    running, Camera.camera_offset, Camera.zoom, tracked_body_index = Utilities.handle_input(Camera.camera_offset,Camera.zoom,Camera.zoom_speed,Camera.pan_speed,tracked_body_index,bodies)
    Camera.camera_offset = Camera.update_camera(bodies,tracked_body_index,Camera.zoom,Utilities.WIDTH,Utilities.HEIGHT,Camera.camera_offset)
    qt = quadtree.QuadTree((Camera.camera_offset[0], Camera.camera_offset[1], Utilities.WIDTH / Camera.zoom, Utilities.HEIGHT / Camera.zoom),1)
    for b in bodies:qt.insert(b.to_dict())
    acc = Mechanics.compute_orbital_motion(bodies,Utilities.dt,screen,Camera.camera_offset,Camera.zoom,Utilities.G,qt)
    collisions = Collision.detect_collisions(bodies, qt)
    tracked_body_index = Collision.handle_collisions(bodies,collisions,collision_tracker,step,merge_delay_steps,tracked_body_index)
    Camera.render_scene(screen,bodies,acc,trail,step,Camera.zoom,Camera.camera_offset,font,clock,tracked_body_index,qt)
    clock.tick(180)
    step += 1