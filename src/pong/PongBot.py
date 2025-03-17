import time
from threading import Thread

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
            paddley = self.game.get_paddle_pos(self.player)
            bally = self.game.get_ball_pos()[1]
            if bally > paddley:
                self.game.move_paddle(self.player,1)
            if bally < paddley:
                self.game.move_paddle(self.player,0)
            time.sleep(self.r)

    def run(self):
        print('Bot wake up!')
        t = Thread(target=self.main_loop)
        t.start()