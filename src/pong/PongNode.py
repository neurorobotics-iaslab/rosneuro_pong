#!/usr/bin/python3
import numpy as np
import rospy
from rosneuro_msgs.msg import NeuroEvent,NeuroOutput
import numpy as np

from PongGame import PongGame, PLAYER1, PLAYER2
from PongBot import PongBot

from rosneuro_pong.msg import GameInfo
from threading import Thread

import sys

class PongNode:
    def __init__(self):
        
        rospy.init_node('pong')
        self.game =  PongGame()

        # Create bot players
        bot1_speed = rospy.get_param('~bot1_speed',0)
        assert (0 <= bot1_speed <= 100, f"Bot 1 speed must be included between 0 (bot inactive) and 100 (max speed). You provided {bot1_speed}")
        bot2_speed = rospy.get_param('~bot2_speed',0)
        assert (0 <= bot2_speed <= 100, f"Bot 2 speed must be included between 0 (bot inactive) and 100 (max speed). You provided {bot2_speed}")
        
        self.bot1_t = None
        if bot1_speed > 0:
            self.bot1 = PongBot(self.game,PLAYER1,bot1_speed)
            self.bot1.run()
        self.bot2_t= None
        if bot2_speed > 0:
            self.bot2 = PongBot(self.game,PLAYER2,bot2_speed)
            self.bot2.run()

        # Init game and subscribe to player topics
        sub1 = rospy.Subscriber('/smr/neuroprediction/player1', NeuroOutput, lambda msg: self.receive_probabilities(msg,PLAYER1))
        sub2 = rospy.Subscriber('/smr/neuroprediction/player2', NeuroOutput, lambda msg: self.receive_probabilities(msg,PLAYER2))
        self.game_info_pub = rospy.Publisher('/gameinfo', GameInfo, queue_size=1)
        self.rate = rospy.Rate(16) #hz

    def run(self):
        self.pub = Thread(target=self.pub_and_sleep)
        self.pub.start()
        self.game.launch_game()
        self.exit()

    def pub_and_sleep(self):
        rospy.sleep(1)
        while not rospy.is_shutdown():
            self.publish_game_info()
            self.rate.sleep()
        print('Game is over! Porcodio!')
        self.game.quit()

    def exit(self):
        rospy.signal_shutdown('Game over')        
                    
    def publish_game_info(self):
        game_info = GameInfo()
        game_info.ball.x, game_info.ball.y  = self.game.get_ball_pos()
        game_info.ball.speed_x, game_info.ball.speed_y  = self.game.get_ball_speed()
        
        game_info.paddle1.y = self.game.get_paddle_pos(PLAYER1)
        game_info.paddle2.y = self.game.get_paddle_pos(PLAYER2)
        game_info.paddle1.speed_y = self.game.get_paddle_speed(PLAYER1)
        game_info.paddle2.speed_y = self.game.get_paddle_speed(PLAYER2)
        game_info.game.score1,game_info.game.score2 = self.game.get_score()
        game_info.game.state = self.game.get_state()

        self.game_info_pub.publish(game_info)
        self.rate.sleep()

    def receive_probabilities(self, msg, player):
        pred = np.where(np.array(msg.hardpredict.data)>0)[0].item()
        self.game.set_paddle_speed(player,pred)

if __name__ == '__main__':
  node = PongNode()
  node.run()