#!/usr/bin/env python3

from Avoider import Avoider

import rospy
from geometry_msgs.msg import Twist #ros msg that deals with moving the robot
from sensor_msgs.msg import LaserScan #ros msg that gets the laser scans

def run():
    vel = Twist()
    # Istanziamo l'oggetto avoider
    avoider = Avoider(vel)
    # Sottoscriviamoci al topic /scan per ricevere i dati provenienti dal lidar
    rospy.Subscriber("/scan", LaserScan, avoider.indentify_regions)
    # Istanziamo un oggetto publisher per poter pubblicare i dati 
    publisher = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    # Decidiamo una frequenza a cui far girare il ciclo seguente
    rate = rospy.Rate(10) #10Hz
    # Eseguiamo il ciclo fino a quando il nodo non ha terminato la propria esecuzione
    while not rospy.is_shutdown():
        # Ad ogni iterazione, calcoliamo le velocita` corrette tramite l'oggetto definito in precedenza
        vel = avoider.avoid()
        # Pubblichiamo le velocita` sul topic corretto
        publisher.publish(vel)
        rate.sleep()

if __name__ == "__main__":
    try:
        # Inizializziamo il nodo responsabile dell'obstacle avoidance assegnandogli un nome adatto
        rospy.init_node("obstacle_avoider_node")
        run()
    except rospy.ROSInterruptException:
        pass
