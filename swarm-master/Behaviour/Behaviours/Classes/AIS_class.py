from typing import Any
from Classes.GPS_class import GPS
from Classes.Vector_class import Vector
import math as m
from geopy import distance 

class avoidance_vectors():
    def __init__(self, other, other_speed, other_bearing):

        #self.own = GPS()
        self.own = (60.3947, 5.2675) #bruker denne til test når båt er koblet på og får inn GPS data
        self.other = other
        self.other_speed = other_speed
        self.other_bearing = other_bearing
        self.distance = distance.distance(self.own, self.other)
        
       # self.other_vec
        #self.aliases = self.create_aliases(self.other, self.other_speed, )
        self.avoidance_vecs = []

        #self.relative = self.calc_bearing(self.own,self.other)

    def __call__(self, own, other, other_speed):
        ang = self.calc_bearing(own, other)
        aliases = self.create_aliases(other, other_speed, ang)
        test_liste = self.make_avoid_vec(own,aliases)

        return test_liste
    def calc_bearing(self, cordA, cordB):
        PointA = cordA
        PointB = cordB
        delta_lon =  PointB[1] - PointA[1]
        #delta_lat = PointA["lat"] - PointB["lat"]
        X = m.cos(PointB[0]*(m.pi/180))*m.sin(delta_lon*(m.pi/180))
        Y = m.cos(PointA[0]*(m.pi/180))*m.sin(PointB[0]*(m.pi/180))-m.sin(PointA[0]*(m.pi/180))*m.cos(PointB[0]*(m.pi/180))*m.cos(delta_lon*(m.pi/180))
        sigma = m.atan2(X,Y)
        bearing = m.degrees(sigma)
        return bearing
    
    def create_aliases(self, cord, speed, rel_angle):
   
        intervals = [60, 300, 600]
        la1 = cord[0]
        lo1 = cord[1]
        
        speed_mps = (speed * 1.85200)*(5/18) #Konverter fra knop til meter per sekund
        distances = [i * speed_mps for i in intervals] 
        alias_list = []
        alias_list.append(cord) #Legger til original posisjon først i lista
        #Looper gjennom distanse lista for å regne ut posisjonen den vil tilsvare i koordinatsystemet 
        #Sidekommentar: gått over til m.radians her grunnet jeg trodde det var en måte på å fikse en feil, men feilen lå et annet sted så nå tør jeg ikke endre det
        for dist in distances:
            a_d = dist / 6371 # Angular distance, distansen delt på joras radius
            la2 = la1 + m.asin(m.sin(m.radians(la1))*m.cos(a_d)+ m.cos(m.radians(la1))*m.sin(a_d)*m.cos(m.radians(rel_angle)))
            lo2 = lo1 + m.atan2(m.sin(m.radians(rel_angle))*m.sin(a_d)*m.cos(m.radians(la1)),m.cos(a_d)-m.sin(m.radians(la1))*m.sin(m.radians(la2)))
            al_pos = (la2, lo2)
            alias_list.append(al_pos)
        
        return alias_list
    
    def make_avoid_vec(self ,own_position, other_positions):
        angle_list = []
        vector_list = []
        distance_list = []
        for positions in other_positions:
            ang = self.calc_bearing(own_position,positions)
            al_dist = distance.distance(own_position,positions)
            avoid_ang = ang + 180
            if avoid_ang > 360:
                avoid_ang = avoid_ang - 360
            else:
                avoid_ang = avoid_ang
            angle_list.append(avoid_ang)
            distance_list.append(al_dist)
        for i in range(len(angle_list)):
            vec = Vector(1/(i+1),angle_list[i]) # Lager vektorer der fremtidige posisjoner blir svakere vektet
            vector_list.append(vec)
        return_arg = (vector_list,distance_list)
        return return_arg
    


