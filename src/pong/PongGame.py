#!/usr/bin/python3
import pygame
import sys
import random
import numpy as np

DIR_UP = 1
DIR_DOWN = 0
PLAYER1 = 0
PLAYER2 = 1
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)        
SCREEN_WIDTH, SCREEN_HEIGHT = 800, 600
        
PADDLE_WIDTH,PADDLE_HEIGHT = 10, 100
PADDLE_SPEED = 5
BALL_SIZE = 20
BALL_SPEED_X, BALL_SPEED_Y = 5, 5
BALL_SPEED = 5

class PongGame:
    def __init__(self):
        self.PADDLE_WIDTH, self.PADDLE_HEIGHT = 10, 100
        self.PADDLE_SPEED = 5
        self.BALL_SIZE = 20
        self.ball_speed_x, self.ball_speed_y = 5, 5
        self.WIDTH, self.HEIGHT = SCREEN_WIDTH,SCREEN_HEIGHT
        self.score1, self.score2 = 0, 0
        self.font = None
        self.screen = None
        self.paddle1 = None
        self.paddle2 = None
        self.ball = None
    
    def move_paddle(self, player, direction):
        if player == 0:
            paddle = self.paddle1
        else:
            paddle = self.paddle2
        if direction == 0 and paddle.top > 0:
            paddle.y -= self.PADDLE_SPEED * 3
        if direction == 1 and paddle.bottom < self.HEIGHT:
            paddle.y += self.PADDLE_SPEED * 3

    def move_ball(self):
        self.ball.x += self.ball_speed_x
        self.ball.y += self.ball_speed_y

    def get_ball_pos(self):
        return self.ball.x,self.ball.y
    
    def get_paddle_pos(self,player):
        if player == PLAYER1:
            return self.paddle1.y
        elif player == PLAYER2:
            return self.paddle2.y
        
    def launch_game(self):
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        pygame.display.set_caption("Pong Game")

        self.paddle1 = pygame.Rect(10, (self.HEIGHT - self.PADDLE_HEIGHT) // 2, self.PADDLE_WIDTH, self.PADDLE_HEIGHT)
        self.paddle2 = pygame.Rect(self.WIDTH - 20, (self.HEIGHT - self.PADDLE_HEIGHT) // 2, self.PADDLE_WIDTH, self.PADDLE_HEIGHT)
        self.ball = pygame.Rect(self.WIDTH // 2 - self.BALL_SIZE // 2, self.HEIGHT // 2 - self.BALL_SIZE // 2, self.BALL_SIZE, self.BALL_SIZE)
        self.font = pygame.font.Font(None, 74)
        self.reset_ball()
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
            
            keys = pygame.key.get_pressed()
            if keys[pygame.K_w] and self.paddle1.bottom > 0:
                self.move_paddle(PLAYER1, DIR_DOWN)
            if keys[pygame.K_s] and self.paddle1.top < self.HEIGHT:
                self.move_paddle(PLAYER1, DIR_UP)
            if keys[pygame.K_UP] and self.paddle2.bottom > 0:
                self.move_paddle(PLAYER2, DIR_DOWN)
            if keys[pygame.K_DOWN] and self.paddle2.top < self.HEIGHT:
                self.move_paddle(PLAYER2, DIR_UP)

            self.move_ball()

            if self.ball.top <= 0 or self.ball.bottom >= self.HEIGHT:
                self.ball_speed_y = -self.ball_speed_y
            if self.ball.colliderect(self.paddle1) or self.ball.colliderect(self.paddle2):
                self.ball_speed_x = -self.ball_speed_x
                # detect collision point
                if self.ball.colliderect(self.paddle1):
                    self.ball_speed_y = -2 * (self.paddle1.centery - self.ball.centery)/self.PADDLE_HEIGHT *  BALL_SPEED
                elif self.ball.colliderect(self.paddle2):
                    self.ball_speed_y = -2 * (self.paddle2.centery - self.ball.centery)/self.PADDLE_HEIGHT * BALL_SPEED
                else:
                    self.ball_speed_y = np.sign(self.ball_speed_y) * BALL_SPEED
            if self.ball.left <= 0:
                self.score2 += 1
                self.reset_ball()
            elif self.ball.right >= self.WIDTH:
                self.score1 += 1
                self.reset_ball()
            self.draw()
            pygame.display.flip()
            pygame.time.Clock().tick(60)
            if keys[pygame.K_ESCAPE]:
                pygame.quit()
                sys.exit()

    def reset_ball(self):
        self.ball.x, self.ball.y = self.WIDTH // 2 - self.BALL_SIZE // 2, self.HEIGHT // 2 - self.BALL_SIZE // 2
        self.ball_speed_x, self.ball_speed_y = BALL_SPEED_X, BALL_SPEED_Y
        self.ball_speed_x *= random.choice((1, -1))
        self.ball_speed_y *= random.uniform(1, -1)

    def move_ball(self):
        self.ball.x += self.ball_speed_x
        self.ball.y += self.ball_speed_y
    
    def draw(self):
        self.screen.fill(BLACK)
        pygame.draw.rect(self.screen, WHITE, self.paddle1)
        pygame.draw.rect(self.screen, WHITE, self.paddle2)
        pygame.draw.ellipse(self.screen, WHITE, self.ball)
        pygame.draw.aaline(self.screen, WHITE, (self.WIDTH // 2, 0), (self.WIDTH // 2, self.HEIGHT))
        score_text1 = self.font.render(str(self.score1), True, WHITE)
        score_text2 = self.font.render(str(self.score2), True, WHITE)
        self.screen.blit(score_text1, (self.WIDTH // 4, 20))
        self.screen.blit(score_text2, (self.WIDTH * 3 // 4, 20))

if __name__ == '__main__':
    game = PongGame()
    game.launch_game()