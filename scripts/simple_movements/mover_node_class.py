#!/usr/bin/env python3

from Mover import Mover

import rospy
from geometry_msgs.msg import Twist # messaggio ros che gestisce il movimento del robot

def run():

    # Scegliamo il tipo di movimento desiderato tra: 'linear', 'diagonal', 'square'
    # e il corrispondente numero di steps da eseguire: 10 (linear), 10 (diagonal) oppure 56 (square)
    move_type = 'diagonal'
    t_end = 14
    # Assegnamo un valore alla velocità (fissa) che il robot avrà nelle diverse direzioni quando diversa da zero
    # Velocità lineare in direzione x [m/s] (quando diversa da 0)
    lin_vel_x_when_not_zero=0.1
    # Velocità angolare [rad/s] (quando diversa da 0)
    ang_vel_when_not_zero=-0.4

    # Istanziamo un oggettp Twist, che in ros gestisce il movimento del robot, in base alle velocità
    vel = Twist()

    # Istanziamo l'oggetto mover
    mover = Mover(vel,
                  lin_vel_x_when_not_zero,
                  ang_vel_when_not_zero)
    
    # Istanziamo un oggetto publisher per poter pubblicare i dati 
    publisher = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    # Decidiamo una frequenza a cui far girare il ciclo seguente
    rate = rospy.Rate(10) # 10Hz, ossia uno step al secondo

    t0 = rospy.Time.now().to_sec()
    t_interval = 0

    while(t_interval < t_end):
        # Ad ogni iterazione, calcoliamo le velocita` corrette tramite l'oggetto definito in precedenza
        vel = mover.move(t_interval, move_type)
        # Pubblichiamo le velocita` sul topic corretto
        publisher.publish(vel)
        t1 = rospy.Time.now().to_sec()
        t_interval = t1 - t0
        #rate.sleep()

if __name__ == "__main__":
    try:
        # Inizializziamo il nodo responsabile dell'obstacle avoidance assegnandogli un nome adatto
        rospy.init_node("obstacle_avoider_node")
        run()
    except rospy.ROSInterruptException:
        pass
