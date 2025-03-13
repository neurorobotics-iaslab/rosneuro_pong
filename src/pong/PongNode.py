#!/usr/bin/python3
import numpy as np
import rospy
from rosneuro_msgs.msg import NeuroEvent,NeuroOutput
import numpy as np

from PongGame import PongGame

def main():
    global game
    rospy.init_node('pongGUI', anonymous=True)

    sub1 = rospy.Subscriber('/smr/neuroprediction/player1', NeuroOutput, lambda msg: receive_probabilities(msg,0))
    sub2 = rospy.Subscriber('/smr/neuroprediction/player2', NeuroOutput, lambda msg: receive_probabilities(msg,1))
    game =  PongGame()
    game.launch_game()
    rospy.signal_shutdown('End of Game')
    
def receive_probabilities(msg, player):
    global game
    pred = np.where(np.array(msg.hardpredict.data)>0)[0].item()
    game.move_paddle(player,pred)

if __name__ == '__main__':
  main()