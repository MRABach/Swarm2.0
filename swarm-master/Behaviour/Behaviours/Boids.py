#!/usr/bin/env python
'''
This class calculates vector for the Boids behaviour

needs:
    from self
        current movement and pos
    from others in swarm
        speed and bearing
        distance and relative angle
        distance to east (x)
        distance to north (x)

Questions: anhellesnes@fhs.mil.no
'''
import numpy as np
import math as m

from Classes.GPS_class import GPS
from Classes.Vector_class import Vector
from Classes.AIS_class import avoidance_vectors
from pyais.stream import TCPConnection

class boidBehavior():
    ''' Object for running calculations of Boids behaviour '''
    def __init__(self):

        self.Ka = 1.5   #Adjusting alignment
        self.Kc = 1.85  #Adjusting cohesion
        self.Ks = 0.5   #Adjusting separation

        self.maxForce   = 1.2 # maximum magnitude of cohesion and separation
        self.maxSpeed   = 2.0 # Maximum speed in m/s
        self.perception = 100.0 # Max distance to percieve other boats


        self.position = GPS()
        self.movement = Vector()

        self.has_newCurr = False

    def __call__(self, position, movement, global_list):
        ''' Call function for initiated boid behaviour

        args:
            position: Vector of current position
            movement: Vector of current movement
            global_list: Pre-made list of dictionaries from caller with behaviour based
            data from other boats in swarm
        '''
        self._handle_current(position, movement)

        host = "153.44.253.27"
        port = 5631
        mylon = position.lon
        mylat = position.lat
        buffer = 0.1
        list_vec_3d = [] # En liste med lister av vector objekter
        list_dist_3d = [] # En liste med lister av distanser til fartøy

        # AIS kontakt for å teste unvikelse
        
        #Kontakt ved kaien, burde gi en kraftig vektor mot simonsviken
        sim_lat = 60.393807
        sim_lon = 5.263284
        # Kontakt ved simonsviken, burde gi kraftig vektor mot kaia
        #sim_lat = 60.394058
        #sim_lon = 5.268667

        # Kontakt lengre i fra kai 
        #sim_lat = 60.393920
        #sim_lon = 5.255885
        sim_pos = (sim_lat, sim_lon)
        sim_speed = 1
        sim_bearing = 1
        #bearing som burde få simonsviken avstøtningsvektoren til å gå mer sør-vest
        #sim_bearing = 310

        sim = avoidance_vectors(sim_pos,sim_speed, sim_bearing)
        b_list = sim(position, sim_pos, sim_speed)
        sim_vectors = b_list[0]
        sim_distances = b_list[1]

                   
        """
        for msg in TCPConnection(host, port):
       # print(test)
            boat = msg.decode() #Dekoder AIS meldingen
            ais_content = boat
            ais_dict = ais_content.asdict() # Gjør den dekodede meldingen om til en dict så den kan brukes i koden
            #print(ais_dict)
            # Filter for å få bort AIS meldinger som ikke er relevante for navigering av dronene
            if ais_dict["msg_type"] == 1 or ais_dict["msg_type"] == 2 or ais_dict["msg_type"] == 3 or ais_dict["msg_type"] == 18 or ais_dict["msg_type"] == 19:
            # print(ais_dict)
                #Filter for å få bort fartøy langt unna
                if ((ais_dict["lon"] < mylon + buffer) and  (ais_dict["lon"] > mylon - buffer)) and  ((ais_dict["lat"] < mylat + buffer) and  (ais_dict["lat"] > mylat - buffer)):
                    #print(ais_dict)
                    other_pos = (ais_dict["lat"], ais_dict["lon"])
                    
                    #Lager en instanse av klassen avoidance_vectors
                    instans = avoidance_vectors(other_pos, ais_dict["speed"], ais_dict["course"])
                    #Kaller call funksjonen til vektoren og returnerer en liste med unvikelsesvektorer
                    get_lists = instans(position,other_pos,ais_dict["speed"])
                    get_vec_list = get_lists[0]
                    get_distances = get_lists[1]
                    #Legger til listen med vektorene i en liste
                    list_vec_3d.append(get_vec_list) 
                    list_dist_3d.append(get_distances)
                   
                    """

        alignment = self._calculate_alignment(global_list)
        cohesion = self._calculate_cohesion(global_list)
        separation = self._calculate_separation(global_list)

        """
        #Legger sammen alle unvikelsesvektorene for å få en samlet unnvikelsesvektor
        tot_avoid_vec = 0
        teller1 = 0
        teller2 = 0
        for list in list_vec_3d:
            for obj in list:
                # Henter ut distansen fra eget fartøy til det aktuelle fartøyet / aliaset
                r = get_distances[teller1][teller2]
                #obj.magnitude = obj.magnitude + 3/(m.pow(r,2)+0.25)
                tot_avoid_vec += obj * (3/(m.pow(r,2)+0.25))
                teller2 += 1
        teller1 += 1
        """
        #Kd = 3/(m.pow(r,2)+0.25) #Adjusting dodge, r is radius

        # TIL SIM
        index = 0
        for vecs in sim_vectors:
            r = sim_distances[index]
            tot_avoid_vec = vecs * (3/(m.pow(r,2)+0.25))

        print("alignment x: ", alignment.magnitude * self.Ka, " y : ", alignment.angle * self.Ka)
        print("cohesion x: ", cohesion.magnitude * self.Kc , " y : ", cohesion.angle * self.Kc)
        print("separation x: ", separation.magnitude * self.Ks, " y : ", separation.angle * self.Ks)

        wantedXY = alignment * self.Ka + cohesion * self.Kc + separation * self.Ks + tot_avoid_vec

        return wantedXY

    def _handle_current(self, current_movement, current_position):
        self.position = current_position
        self.movement = current_movement

        self.has_newCurr = True

    def _calculate_cohesion(self, boats):
        '''
        Function to calculate cohesion vector

        Args:
            boats: list of dictionaries containing data from other boats
        
        Returns:
            A vector containing cohesion for unit
        '''
        cohesion = Vector()
        total = 0.0
        center_of_mass = Vector()

        for boid in boats:
            if boid['distance'] < self.perception and boid['distance'] != 0.0: # only boids within perception calculated
                center_of_mass.magnitude += boid['x']
                center_of_mass.angle     += boid['y']
                total += 1.0

        if total > 0.0 and center_of_mass.magnitude != 0.0 and center_of_mass.angle != 0.0: #only manipulates vector if there is one
            cohesion = center_of_mass.__truediv__(total) #- self.position
            cohesion_tot = m.sqrt(m.pow(cohesion.magnitude, 2.0)+m.pow(cohesion.angle, 2.0))

            if cohesion_tot > 0.0: #Proportions the vector to maxSpeed
                cohesion = (cohesion.__truediv__(cohesion_tot)) * self.maxSpeed
            if cohesion_tot > self.maxForce:
                cohesion = (cohesion.__truediv__(cohesion_tot)) * self.maxForce

        return cohesion # vector towards center of mass - cohesion

    def _calculate_separation(self, boats):
        '''
        Function to calculate separation vector

        Args:
            boats: list of dictionaries containing data from other boats
        
        Returns:
            A vector containing separation for unit
        '''
        separation = Vector()
        total = 0.0
        average_vector = Vector()

        for boid in boats:
            if boid['distance'] < self.perception and boid['distance'] != 0.0: # only boids within perception calculated
                diff = Vector(-boid['x'], -boid['y'])
                diff = diff.__truediv__(boid['distance'])
                average_vector += diff
                total += 1.0

        if total > 0.0 and average_vector.magnitude != 0.0 and average_vector.angle != 0.0:          
            separation = average_vector.__truediv__(total)
            separation_tot = m.sqrt(m.pow(separation.magnitude, 2.0)+m.pow(separation.angle, 2.0))

            if separation_tot > 0.0: #Proportions the vector to maxSpeed
                separation = (separation.__truediv__(separation_tot)) * self.maxSpeed
            if separation_tot > self.maxForce:
                separation = (separation.__truediv__(separation_tot)) * self.maxForce

        return separation

    def _calculate_alignment(self, boats): 
        '''
        Function to calculate alignment vector

        Args:
            boats: list of dictionaries containing data from other boats
        
        Returns:
            A vector containing alignment for unit
        '''

        alignment = Vector()
        total = 0.0
        average_temp = Vector(0.0, 0.0)
        average_vector = Vector(0.0, 0.0)

        for boid in boats:
            if boid['distance'] < self.perception and boid['distance'] != 0.0: #finds number of boids within perception

                dx = boid['speed'] * m.sin(m.radians(boid['bearing'])) #converts vector to x,y components
                dy = boid['speed'] * m.cos(m.radians(boid['bearing']))
                average_temp.set(dx,dy)

                average_vector += average_temp
                total += 1.0

        if total > 0.0:
            try:
                alignment.magnitude = average_vector.magnitude / total
            except ValueError:
                pass

            try:
                alignment.angle = average_vector.angle / total
            except ValueError:
                pass

            alignment_tot = m.sqrt(m.pow(average_vector.magnitude, 2.0) + m.pow(average_vector.angle, 2.0))

            if alignment_tot > self.maxForce:
                alignment = (alignment.__truediv__(alignment_tot)) * self.maxForce

        return alignment