import sys
import math
import pygame

from pf import Particle
from config import *


class Window:
    def __init__(self, on_update=None, background_file=None, robot=None, particles=None):
        '''
        Args:
            background_file(str): Background filename
            robot(Particle): Particle representing actual robot
            particles(list[Particle]): Simulation particles
        '''
        self.on_update = on_update

        # Initialize view
        pygame.init()
        if background_file is None:
            background_file = BACKGROUND_FILE
        self.background = pygame.image.load(background_file)
        self.particle_sprite = pygame.image.load(PARTICLE_SPRITE)
        self.robot_sprite = pygame.image.load(ROBOT_SPRITE)
        self.size = self.background.get_size()
        self.screen = pygame.display.set_mode(self.size)

        # Initialize simulation components
        self.robot_speed = [0, 0] # [LINEAR, ANGULAR]
        self.robot = robot
        if self.robot is None:
            self.robot = Particle(self.size[0]/2, self.size[1]/2)
        self.particles = []
        if particles:
            self.particles = particles

    def run(self):
        '''
        Game loop.
        '''
        while True:
            self.on_events(pygame.event.get())
            self.robot.move(*self.robot_speed)

            if self.on_update is not None:
                self.on_update(self.robot, self.particles, self.robot_speed)

            self.draw()

    def on_events(self, events):
        '''
        Handle user generated events.
        '''
        for event in events:
            if event.type == pygame.QUIT:
                sys.exit()
            elif event.type == pygame.KEYDOWN:
                if event.key in SPEED_DELTAS:
                    delta = SPEED_DELTAS[event.key]
                    self.robot_speed[delta[0]] += delta[1]
            elif event.type == pygame.KEYUP:
                if event.key in SPEED_DELTAS:
                    delta = SPEED_DELTAS[event.key]
                    self.robot_speed[delta[0]] -= delta[1]

    def draw(self):
        '''
        Draw everything (background, particles, and robot).
        '''
        self.screen.fill(BLACK)
        self.screen.blit(self.background, self.background.get_rect())

        # Draw particles
        for particle in self.particles:
            self._blit_rotated(self.particle_sprite, particle)

        self._blit_rotated(self.robot_sprite, self.robot)

        pygame.display.flip()

    def _blit_rotated(self, sprite, element):
        '''
        Blit sprite in the position indicated by element (element must have
        attributes: x, y, theta).
        '''
        angle = math.degrees(element.theta)
        sprite = pygame.transform.rotate(sprite, angle)
        x = int(element.x - sprite.get_width() / 2)
        y = int(element.y - sprite.get_height() / 2)
        self.screen.blit(sprite, (x, y))


if __name__ == '__main__':
    import math
    from random import random
    from pf import Particle

    def on_update(robot, particles, robot_speed):
        print(robot.x, robot.y, robot.theta)

    win = Window(on_update=on_update)

    w, h = win.size
    def random_particle():
        x = random() * w
        y = random() * h
        theta = random() * 2 * math.pi
        return Particle(x, y, theta)

    win.particles = [random_particle() for _ in range(50)]

    win.run()
