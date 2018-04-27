import pygame

# COLORS
BLACK = (0, 0, 0)

# SPRITES
PARTICLE_SPRITE = 'particle.png'
ROBOT_SPRITE = 'robot.png'
BACKGROUND_FILE = 'sparse_obstacles.png'

# MOVEMENT
DELTA_LINEAR = 1
DELTA_ANGULAR = 0.1

# KEY BINDINGS
SPEED_DELTAS = {
    pygame.K_w: (0, DELTA_LINEAR),
    pygame.K_s: (0, -DELTA_LINEAR),
    pygame.K_a: (1, DELTA_ANGULAR),
    pygame.K_d: (1, -DELTA_ANGULAR),
}
