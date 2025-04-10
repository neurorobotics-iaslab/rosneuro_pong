#!/usr/bin/python3
import pygame
import random
import numpy as np
from constants import *

class Paddle(pygame.Rect):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.speed = 0

class Ball(pygame.Rect):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.speed_x = 0
        self.speed_y = 0

class PongGame:
    def __init__(self):
        self.PADDLE_WIDTH, self.PADDLE_HEIGHT = PADDLE_HEIGHT, PADDLE_HEIGHT
        self.PADDLE_SPEED = PADDLE_SPEED
        self.BALL_SIZE = BALL_SIZE
        self.WIDTH, self.HEIGHT = SCREEN_WIDTH,SCREEN_HEIGHT
        self.score1, self.score2 = 0, 0
        
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        self.font = pygame.font.Font(None, 74)
        pygame.display.set_caption("Pong Game")

        self.paddle1 = Paddle(10, (self.HEIGHT - self.PADDLE_HEIGHT) // 2, PADDLE_WIDTH, PADDLE_HEIGHT)
        self.paddle2 = Paddle(self.WIDTH - 20, (self.HEIGHT - self.PADDLE_HEIGHT) // 2, PADDLE_WIDTH, PADDLE_HEIGHT)

        self.ball = Ball(self.WIDTH // 2 - self.BALL_SIZE // 2, self.HEIGHT // 2 - self.BALL_SIZE // 2, self.BALL_SIZE, self.BALL_SIZE)
        self.p1_training = False
        self.p2_training = False

    def move_ball(self):
        self.ball.x += self.ball.speed_x
        self.ball.y += self.ball.speed_y

    def get_ball_pos(self):
        return self.ball.x,self.ball.y
    
    def get_ball_speed(self):
        return self.ball.speed_x,self.ball.speed_y
    
    def get_paddle_pos(self,player):
        if player == PLAYER1:
            return self.paddle1.y
        elif player == PLAYER2:
            return self.paddle2.y
        
    def get_paddle_speed(self,player):
        if player == PLAYER1:
            return self.paddle1.speed
        elif player == PLAYER2:
            return self.paddle2.speed
        
    def get_score(self):
        return self.score1, self.score2

    def get_state(self):
        return self.state
    
    def set_state(self, new_state):
        self.state= new_state

    def reset_ball(self):
        self.ball.x, self.ball.y = self.WIDTH // 2 - self.BALL_SIZE // 2, self.HEIGHT // 2 - self.BALL_SIZE // 2
        self.ball.speed_x, self.ball.speed_y = BALL_SPEED_X, BALL_SPEED_Y
        self.ball.speed_x *= random.choice((1, -1))
        self.ball.speed_y *= random.uniform(1, -1)

    def move_ball(self):
        self.ball.x += self.ball.speed_x
        self.ball.y += self.ball.speed_y
        if self.ball.top <= 0 or self.ball.bottom >= self.HEIGHT:
            self.ball.speed_y = -self.ball.speed_y
        if self.ball.colliderect(self.paddle1) or self.ball.colliderect(self.paddle2):
            self.ball.speed_x = -self.ball.speed_x
            # detect collision point
            if self.ball.colliderect(self.paddle1):
                self.ball.speed_y = -2 * (self.paddle1.centery - self.ball.centery)/self.PADDLE_HEIGHT *  BALL_SPEED_Y
            elif self.ball.colliderect(self.paddle2):
                self.ball.speed_y = -2 * (self.paddle2.centery - self.ball.centery)/self.PADDLE_HEIGHT * BALL_SPEED_Y
            else:
                self.ball.speed_y = np.sign(self.ball.speed_y) * BALL_SPEED_Y
        if self.ball.left <= 0:
            self.score2 += 1
            self.reset_ball()
            self.set_state(STATE_NEWBALL)
        elif self.ball.right >= self.WIDTH:
            self.score1 += 1
            self.reset_ball()
            self.set_state(STATE_NEWBALL)

    def set_paddle_speed(self, player, direction):
        if player == PLAYER1:
            paddle = self.paddle1
        elif player == PLAYER2:
            paddle = self.paddle2

        if direction == DIR_UP:
            if paddle.top > 0:
                paddle.speed = -self.PADDLE_SPEED
            else: 
                paddle.speed = 0
        if direction == DIR_DOWN:
            if paddle.bottom < self.HEIGHT:
                paddle.speed = +self.PADDLE_SPEED
            else: 
                paddle.speed = 0
        
        if direction == DIR_STOP:
            paddle.speed = 0

    def move_paddles(self):
        self.paddle1.y += self.paddle1.speed
        self.paddle2.y += self.paddle2.speed

    def get_training(self):
        return int(self.p1_training), int(self.p2_training)
        
    def draw(self):
        self.screen.fill(BLACK)
        # Field, Paddles and Ball
        pygame.draw.aaline(self.screen, WHITE, (self.WIDTH // 2, 0), (self.WIDTH // 2, self.HEIGHT))
        pygame.draw.rect(self.screen, WHITE, self.paddle1)
        pygame.draw.rect(self.screen, WHITE, self.paddle2)
        pygame.draw.ellipse(self.screen, WHITE, self.ball)

        border_color = GRAY
        c1_center = (self.WIDTH // 2 -30, 40)
        c2_center = (self.WIDTH // 2 +30, 40)
        pygame.draw.circle(self.screen, border_color, c1_center, 8)  # Border
        pygame.draw.circle(self.screen, GREEN if self.p1_training else BLACK, c1_center, 6)  # Inner circle

        pygame.draw.circle(self.screen, border_color, c2_center, 8)  # Border
        pygame.draw.circle(self.screen, GREEN if self.p2_training else BLACK, c2_center, 6)  # Inner circle
        
        # Score
        score_text1 = self.font.render(str(self.score1), True, WHITE)
        score_text2 = self.font.render(str(self.score2), True, WHITE)
        self.screen.blit(score_text1, (self.WIDTH // 4, 20))
        self.screen.blit(score_text2, (self.WIDTH * 3 // 4, 20))
        
    def launch_game(self):
        self.set_state(STATE_PAUSED)
        self.reset_ball()
        self.draw()
        while True: 
            keys = pygame.key.get_pressed()
            if keys[pygame.K_w] and self.paddle1.bottom > 0:
                self.set_paddle_speed(PLAYER1, DIR_UP)
            if keys[pygame.K_s] and self.paddle1.top < self.HEIGHT:
                self.set_paddle_speed(PLAYER1, DIR_DOWN)
            if keys[pygame.K_UP] and self.paddle2.bottom > 0:
                self.set_paddle_speed(PLAYER2, DIR_UP)
            if keys[pygame.K_DOWN] and self.paddle2.top < self.HEIGHT:
                self.set_paddle_speed(PLAYER2, DIR_DOWN)            
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.set_state(STATE_GAMEOVER)
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE:
                        if self.state == STATE_PAUSED:
                            self.set_state(STATE_RUNNING)
                        elif self.state == STATE_RUNNING:
                            self.set_state(STATE_PAUSED)
                    if event.key == pygame.K_ESCAPE:
                        self.set_state(STATE_GAMEOVER)
                    if event.key == pygame.K_r:
                        self.reset_ball()
                        self.draw()
                        pygame.display.flip()
                        self.set_state(STATE_NEWBALL)
                    if event.key == pygame.K_t:
                        self.p1_training = not self.p1_training
                    if event.key == pygame.K_y:
                        self.p2_training = not self.p2_training

                if event.type == pygame.KEYUP:
                    if event.key == pygame.K_s or event.key == pygame.K_w:
                        self.set_paddle_speed(PLAYER1, DIR_STOP)
                    if event.key == pygame.K_UP or event.key == pygame.K_DOWN:
                        self.set_paddle_speed(PLAYER2, DIR_STOP)

            if self.state == STATE_NEWBALL:
                pygame.time.wait(2000)
                self.set_state(STATE_RUNNING)
                
            if self.state == STATE_PAUSED:
                # Draw a semi transparent overlay with the pause text
                self.draw()
                overlay = pygame.Surface((self.WIDTH, self.HEIGHT))
                overlay.fill((0, 0, 0))
                overlay.set_alpha(128)
                self.screen.blit(overlay, (0, 0))
                pause_text = self.font.render("PAUSED", True, WHITE)
                text_rect = pause_text.get_rect(center=(self.WIDTH // 2, self.HEIGHT // 2))
                self.screen.blit(pause_text, text_rect)
                pygame.display.flip()
                continue

            if self.state == STATE_RUNNING:
                self.move_paddles()
                self.move_ball()
                self.draw()
                pygame.display.flip()

            if self.state == STATE_GAMEOVER:
                pygame.quit()
                return
            
            pygame.time.Clock().tick(60)

    def quit(self):
        self.set_state(STATE_GAMEOVER)

if __name__ == '__main__':
    game = PongGame()
    game.launch_game()