#!/usr/bin/python3
import numpy as np
import rospy
from rosneuro_msgs.msg import NeuroEvent,NeuroOutput
import numpy as np

from PongGame import PongGame, PLAYER1, PLAYER2
from PongBot import PongBot

def main():
    global game
    rospy.init_node('pong')
    game =  PongGame()

    # Create bot players
    bot1_speed = rospy.get_param('~bot1_speed',0)
    assert (0 <= bot1_speed <= 100, f"Bot 1 speed must be included between 0 (bot inactive) and 100 (max speed). You provided {bot1_speed}")
    bot2_speed = rospy.get_param('~bot2_speed',0)
    assert (0 <= bot2_speed <= 100, f"Bot 2 speed must be included between 0 (bot inactive) and 100 (max speed). You provided {bot2_speed}")
    if bot1_speed > 0:
        bot1 = PongBot(game,PLAYER1,bot1_speed)
        bot1.run()
    if bot2_speed > 0:
        bot2 = PongBot(game,PLAYER2,bot2_speed)
        bot2.run()

    # Init game and subscribe to player topics
    sub1 = rospy.Subscriber('/smr/neuroprediction/player1', NeuroOutput, lambda msg: receive_probabilities(msg,PLAYER1))
    sub2 = rospy.Subscriber('/smr/neuroprediction/player2', NeuroOutput, lambda msg: receive_probabilities(msg,PLAYER2))
    game.launch_game()
    rospy.spin()
    rospy.signal_shutdown('End of Game')
    
def receive_probabilities(msg, player):
    global game
    pred = np.where(np.array(msg.hardpredict.data)>0)[0].item()
    game.move_paddle(player,pred)

if __name__ == '__main__':
  main()