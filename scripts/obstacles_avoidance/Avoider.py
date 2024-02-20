#!/usr/bin/env python3

class Avoider():
	''' This class provides simple obstacle avoidance functionalities to a ROS robot '''

	# Costruiamo un dizionario per tenere traccia delle distanze delle varie regioni 
	Regions_Report = {
	                     "front_C": [], "front_L": [], "left_R" : [],
	                     "left_C" : [], "left_L" : [], "back_R" : [],
	                     "back_C" : [], "back_L" : [], "right_R": [],
	                     "right_C": [], "right_L": [], "front_R": [],
	                 }
	
	# Associamo un costo per ruotare ad ogni regione (front_C)
	Regions_Distances = {
	                     "front_C":  0, "front_L":  1, "left_R" :  2,
	                     "left_C" :  3, "left_L" :  4, "back_R" :  5,
	                     "back_C" :  6, "back_L" : -5, "right_R": -4,
	                     "right_C": -3, "right_L": -2, "front_R": -1,
	                 	}

	# Costruttore: che cosa ci serve?
	def __init__(self, 
			  	vel_obj, 
			  	obstacle_threshold=0.5, 
				regional_angle=30, 
				normal_lin_vel=0.1, 
				trans_lin_vel=-0.09, 
				trans_ang_vel=0.5):
		'''
		:param vel_obj           : Oggetto velocità; conterrà il comando velocità(data); Twist()
		:param obstacle_threshold: Soglia a cui gli oggetti sono considerati vicini (in m)
		:param regional_angle    : Angolo di estensione delle regioni
		:param normal_lin_vel    : Velocità lineare mantenuta dal robot in assenza di ostacoli
		:param trans_lin_vel     : Velocità di transizione del robot (il robot arretra lentamente mentre ruota per poter ruotare sul posto)
		:param trans_ang_vel 	 : Velocità di rotazione angolare
		'''
		self.vel_obj        = vel_obj
		self.OBSTACLE_DIST  = obstacle_threshold
		self.REGIONAL_ANGLE = regional_angle
		self.NORMAL_LIN_VEL = normal_lin_vel
		self.TRANS_LIN_VEL  = trans_lin_vel
		self.TRANS_ANG_VEL  = trans_ang_vel


	# Funzione che identifica le regioni attorno al robot
	def indentify_regions(self, scan):
		'''
		:param scan: Scannerizza gli oggetti contenuti nei dati del LIDAR 
		'''

		# Lista per tracciare le regioni
		REGIONS = [
		             "front_C", "front_L", "left_R" ,
		             "left_C" , "left_L" , "back_R" ,
		             "back_C" , "back_L" , "right_R",
		             "right_C", "right_L", "front_R",
				  ]

		# Scannerizziamo la prima regione: frontale che va da 15 a -15 gradi rispetto al centro dello scanner
		intermediary = scan.ranges[:int(self.REGIONAL_ANGLE/2)]\
					 + scan.ranges[(len(scan.ranges)-1)*int(self.REGIONAL_ANGLE/2):]
		
		# Controlliamo la regione frontale è libera, in caso positivo salviamo il risultato della scannerizzazione
		new_front_scan = []
		for x in intermediary: 
			if x < self.OBSTACLE_DIST and x!= 'inf' and x!= 0:
				new_front_scan.append(x)
		self.Regions_Report["front_C"] = new_front_scan


		# Numeriamo tutte le altre regioni...
		for i, region in enumerate(REGIONS[1:]):
			
			# ...e scannerizziamole come fatto per la centrale
			
				# Salviamo il risultato se sono libere
			
			
			self.Regions_Report[region] = new_scan
			


	# Funzione che calcola il clearest path
	def _clearance_test(self):

		# Impostiamo i parametri desiderati
		goal = "front_C"
		closest = 10e6
		regional_dist = 0
		maxima = {"destination": "back_C", "distance": 10e-6}

		# Scannerizziamo le regioni
		for region in self.Regions_Report.items():

			# Valutiamo le distanze delle varie regioni (ricordiamo che le regioni vengono salvate in un ordine preciso)
			

			# Se non ci sono ostacoli nella regione trovata...
			
			
				# ... controlliamo che sia la regione meno costosa
			
			
					# Sovrascriviamo i parametri in caso positivo
			
			
			# Altrimenti salviamola lo stesso: potrebbe essere comunque il path più libero
			
				# Sovrascriviamo i parametri
			

		# Calcoliamo il costo di rotazione del robot
		
		
		# Risultati: ci muoviamo? (torniamo "act" come vero o falso)
		
		
		# Risultati: ruotiamo? (torniamo "ang_vel" con il segno appropriato)
		
		
		return act, ang_vel
	


	# Funzione per guidare il robot
	def _steer(self, steer=False, ang_vel=0):
		'''
		:param steer  : Parametro booleano -> Evitiamo l'ostacolo o continuiamo in linea retta?
		:param ang_vel: Velocità angolare del robot
		'''
		if not steer:
		
		else:
		
		# Impostiamo anche le altre velocità
		self.vel_obj.linear.y  = 0
		self.vel_obj.linear.z  = 0
		self.vel_obj.angular.x = 0
		self.vel_obj.angular.y = 0
		self.vel_obj.angular.z = ang_vel


	# Funzione avoid: controlla la velocità del robot		
	def avoid(self):

		# Controlliamo se il percorso è libero
		
		
		# Se "act" è False, "ang_vel" è 0
		
		
		# Usiamo steer per guidare il robot
		
		
		return self.vel_obj
