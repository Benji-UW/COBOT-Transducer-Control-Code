import numpy as np
import pygame

BLACK = pygame.Color('black')
WHITE = pygame.Color('white')


class RobotGUI:
    def __init__(self):
        self.x = 12
        self.y = 12
        self.x_margin = 12
        self.y_margin = 12
        self.line_height = 20
        self.font = pygame.font.Font(None, 24)
        self.max_array_length = 400

    def tprint(self, screen, text_string):
        text_bitmap = self.font.render(text_string, True, BLACK)
        screen.blit(text_bitmap, (self.x, self.y))
        self.y += self.line_height

    def plot_graph(self, screen, size, x_data, y_data, colour=pygame.Color(25, 25, 205), width=2, hold=False):
        if x_data.size > self.max_array_length:
            x_data = x_data[-1-self.max_array_length:-1]
            y_data = y_data[-1-self.max_array_length:-1]
        dx = np.max(x_data) - np.min(x_data)
        dy = np.max(y_data) - np.min(y_data)
        scaled_x = self.x_margin + (x_data - np.min(x_data)) * (size[0] - 2 * self.x_margin) / dx
        if dy > 0:
            scaled_y = self.y_margin + (y_data - np.min(y_data)) * (size[1] - 2 * self.y_margin) / dy
        else:
            scaled_y = self.y_margin + (y_data - np.min(y_data)) * (size[1] - 2 * self.y_margin)
        scaled_y += self.y
        pts = np.array([scaled_x, scaled_y])
        pygame.draw.lines(screen, colour, False, pts.T.tolist(), width)
        if hold:
            self.y += size[1]
            txt = 'dz= %3.2f mm' % (dy*1E3)
            text_bitmap = self.font.render(txt, True, colour)
            screen.blit(text_bitmap, (self.x, self.y))
            self.x += 8*len(txt)
            self.y -= size[1]
        else:
            self.y += size[1]
            text_bitmap = self.font.render('dz= %3.2f mm, dt= %4.2f s' % (dy*1E3, dx), True, colour)
            screen.blit(text_bitmap, (self.x, self.y))
            self.y += self.line_height

    def reset(self):
        self.x = 12
        self.y = 12
        self.line_height = 20

    def indent(self):
        self.x += 12

    def unindent(self):
        self.x -= 12
