#!/usr/bin/env python3

class Mover():
	''' Questa classe implementa delle semplici modalità di movimento per un robot ROS '''

	def __init__(self, vel_obj, lin_vel_x_when_not_zero, ang_vel_when_not_zero):
		'''
		:param vel_obj                  : Oggetto velocità; conterrà i comandi di velocità (dati) da impostare al robot; Twist()
		:param lin_vel_x_when_not_zero  : La velocità lineare che avrà il robot in direzione x quando diversa da 0
		:param ang_vel_when_not_zero    : La velocità angolare che avrà il robot quando diversa da 0
		'''

		self.vel_obj = vel_obj		
		self.LIN_VEL_X = lin_vel_x_when_not_zero
		self.ANG_VEL  = ang_vel_when_not_zero

	def move(self, t_interval, move_type="linear"):
		'''
		:param step: Lo step a cui ci troviamo
		:param move_type: Il tipo di movimento per il robot, a scelta tra 'linear', 'diagonal' e 'square'
		'''
		# Il metodo move si occupa di far muovere il robot in base alla tipologia di movimento
		# richiesta. In questo caso implementiamo tre possibili movimenti:
		# 'linear': il robot si muove in orizzontale percorrendo un lato del quadrato
		# 'diagonal': il robot si muove in diagonale, percorrendo una diagonale del quadrato
		# 'square': il robot si muove lungo tutto il perimetro del quadrato, tornando al punto di partenza
		
		# Calcoliamo le velocità da impostare al robot in base al tipo di movimento desiderato
		# e al time step a cui ci troviamo

		if move_type == "linear":
			lin_vel_x, ang_vel = self._linear(t_interval)
		elif move_type == "diagonal":
			lin_vel_x, ang_vel = self._diagonal(t_interval)
		elif move_type == "square":
			lin_vel_x, ang_vel = self._square(t_interval)

		# Ora possiamo utilizzare il metodo _steer per impostare le velocità appena calcolate al robot
		self._steer(lin_vel_x, ang_vel)

		return self.vel_obj
	
	def _linear(self, t_interval):
		'''
		:param t_interval	: Il tempo trascorso dall'inizio (t_interval=t-t0)
		'''
		# Il robot si muove in linea retta per 1m
		# Ad una vecità di 0.1m/s impiega 10s
		if t_interval >= 0 and t_interval <= 10:
			lin_vel_x = self.LIN_VEL_X
			ang_vel = 0
		# Il robot è arrivato a destinazione!
		# Possiamo fermarlo impostando per ogni step successivo tutte le velocità su 0
		else:
			lin_vel_x = 0
			ang_vel = 0
		# Come ultima cosa, la funzione deve ritornare le due variabili lin_vel_x e ang_vel
		return lin_vel_x, ang_vel

	def _diagonal(self, t_interval):
		'''
		:param t_interval	: Il tempo trascorso dall'inizio (t_interval=t-t0)
		'''
		# Il robot si gira di 45° (pi/4 rad)
		# Ad una velocità di 0.4rad/s impiega circa 2s
		if t_interval >= 0 and t_interval < 2:
			lin_vel_x = 0.0
			ang_vel = self.ANG_VEL
		# Il robot si muove in linea retta per circa 1.4m (1*sqrt(2))
		# Ad una vecità di 0.1m/s impiega 14s
		elif t_interval >= 2 and t_interval <= 16:
			lin_vel_x = self.LIN_VEL_X
			ang_vel = 0
		# Il robot è arrivato a destinazione!
		# Possiamo fermarlo impostando per ogni step successivo tutte le velocità su 0
		else:
			lin_vel_x = 0
			ang_vel = 0
		# Come ultima cosa, la funzione deve ritornare le due variabili lin_vel_x e ang_vel
		return lin_vel_x, ang_vel

	def _square(self, t_interval):
		'''
		:param t_interval	: Il tempo trascorso dall'inizio (t_interval=t-t0)
		'''
		# Primo lato del quadrato: il robot si muove in linea retta per 1m
		# Ad una vecità di 0.1m/s impiega 10s
		if t_interval >= 0 and t_interval <10:
			lin_vel_x = self.LIN_VEL_X
			ang_vel = 0
		# Prima rotazione: il robot si gira di 90° (pi/2 rad)
		# Ad una velocità di 0.4rad/s impiega circa 4s
		elif t_interval >= 10 and t_interval < 14:
			lin_vel_x = 0
			ang_vel = self.ANG_VEL
		# Secondo lato del quadrato: il robot si muove in linea retta per 1m
		# Ad una vecità di 0.1m/s impiega 10s
		elif t_interval >= 14 and t_interval < 24:
			lin_vel_x = self.LIN_VEL_X
			ang_vel = 0
		# Seconda rotazione: il robot si gira di 90° (pi/2 rad)
		# Ad una velocità di 0.4rad/s impiega circa 4s
		elif t_interval >= 24 and t_interval < 28:
			lin_vel_x = 0
			ang_vel = self.ANG_VEL
		# Terzo lato del quadrato: il robot si muove in linea retta per 1m
		# Ad una vecità di 0.1m/s impiega 10s
		elif t_interval >= 28 and t_interval < 38:
			lin_vel_x = self.LIN_VEL_X
			ang_vel = 0
		# Terza rotazione: il robot si gira di 90° (pi/2 rad)
		# Ad una velocità di 0.4rad/s impiega circa 4s
		elif t_interval >= 38 and t_interval < 42:
			lin_vel_x = 0
			ang_vel = self.ANG_VEL
		# Quarto lato del quadrato: il robot si muove in linea retta per 1m
		# Ad una vecità di 0.1m/s impiega 10s
		elif t_interval >= 42 and t_interval < 52:
			lin_vel_x = self.LIN_VEL_X
			ang_vel = 0
		# Quarta rotazione: il robot si gira di 90° (pi/2 rad)
		# Ad una velocità di 0.4rad/s impiega circa 4s
		elif t_interval >= 52 and t_interval < 56:
			lin_vel_x = 0
			ang_vel = self.ANG_VEL
		# Il robot è arrivato a destinazione!
		# Possiamo fermarlo impostando per ogni step successivo tutte le velocità su 0
		else:
			lin_vel_x = 0
			ang_vel = 0
		# Come ultima cosa, la funzione deve ritornare le due variabili lin_vel_x e ang_vel
		return lin_vel_x, ang_vel


	def _steer(self, lin_vel_x, ang_vel):
		'''
		:param lin_vel_x  : La velocità lineare del robot in direzione x
		:param ang_vel    : La velocità angolare del robot
		'''
		self.vel_obj.linear.x = lin_vel_x
		self.vel_obj.linear.y  = 0
		self.vel_obj.linear.z  = 0
		self.vel_obj.angular.x = 0
		self.vel_obj.angular.y = 0
		self.vel_obj.angular.z = ang_vel
