import time
from threading import Thread

from PongGame import DIR_UP,DIR_DOWN,STATE_GAMEOVER

MIN_SPEED = 0 # [movements per second]
MAX_SPEED = 32  # [movements per second]

class PongBot():
    def __init__(self,game,player,speed=50):
        self.player = player
        self.game = game
        self.r = (speed)*(MAX_SPEED-MIN_SPEED)/100 + MIN_SPEED
        print(f"Bot {player} speed = {self.r}")
        self.r = 1/self.r
    
    def main_loop(self):
        time.sleep(2) # wait for the game to start
        while True:
            if self.game.get_state() == STATE_GAMEOVER:
                break
            paddley = self.game.get_paddle_pos(self.player)
            bally = self.game.get_ball_pos()[1]
            if bally > paddley:
                self.game.set_paddle_speed(self.player,DIR_UP)
            if bally < paddley:
                self.game.set_paddle_speed(self.player,DIR_DOWN)
            time.sleep(self.r)

    def run(self):
        print('Bot wake up!')
        self.t = Thread(target=self.main_loop)
        self.t.start()
    
    def quit(self):
        self.t.join()
        print('Bot is sleeping...')