#!/usr/bin/env python3

from Mover import Mover

import rospy
from geometry_msgs.msg import Twist # messaggio ros che gestisce il movimento del robot

def run():

    # Scegliamo il tipo di movimento desiderato tra: 'linear', 'diagonal', 'square'
    # e un intervallo di tempo massimo per eseguire il movimento: 10 (linear), 16 (diagonal) oppure 56 (square)
    # volendo possiamo anche impostare un valore più alto, programmando opportunamente il robot in modo che si
    # fermi dopo aver finito il movimento
    move_type = "linear"
    t_end = 60
    # Definiamo un valore di velocità (fissa) che il robot avrà nelle diverse direzioni quando diversa da zero
    # Velocità lineare in direzione x [m/s] (quando diversa da 0)
    lin_vel_x_when_not_zero=0.1 # non cambiare
    # Velocità angolare [rad/s] (quando diversa da 0)
    ang_vel_when_not_zero=-0.4 # non cambiare

    # Istanziamo un oggettp Twist, che in ros gestisce il movimento del robot, in base alle velocità
    vel = Twist()

    # Istanziamo l'oggetto mover della classe Mover
    mover = Mover(vel,
                  lin_vel_x_when_not_zero,
                  ang_vel_when_not_zero)
    
    # Istanziamo un oggetto publisher per poter pubblicare i dati 
    publisher = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    # Salviamo nella variabile t0 l'istante di tempo in cui il movimento ha inizio
    t0 = rospy.Time.now().to_sec()
    # t_elapsed è una variabile in cui man mano salveremo il tempo trascorso dall'inizio del movimento,
    # e ci serve a impostare in modo opportuno le velocità del robot in modo da fargli svolgere il
    # movimento desiderato; all'inizio t_elapsed è 0
    t_elapsed = 0

    # Il ciclo seguente si ripete finchè non raggiungiamo il tempo massimo, che abbiamo deciso all'inizio del codice
    # P.S. il ciclo viene ripetuto molteplici volte al secondo, quindi in maniera approssimata assumiamo di poter
    # controllare la velocità in modo quasi continuo nel tempo
    while(t_elapsed < t_end):
        # Calcoliamo il tempo trascorso dall'inizio del movimento
        t1 = rospy.Time.now().to_sec()
        t_elapsed = t1 - t0
        # Calcoliamo le velocità corrette tramite l'oggetto mover, definito in precedenza
        # I valori delle velocità dipenderanno dal tempo trascorso dall'inizio del movimento,
        # che abbiamo salvato in t_elapsed, e dal tipo di mvimento scelto, salvato in move_type
        vel = mover.move(t_elapsed, move_type)
        # Pubblichiamo le velocità sul topic corretto
        publisher.publish(vel)

if __name__ == "__main__":
    try:
        # Inizializziamo il nodo responsabile del movimento assegnandogli un nome adatto
        rospy.init_node("mover_node")
        run()
    except rospy.ROSInterruptException:
        pass
