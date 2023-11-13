# Må sette inn dette når i linux: #!/usr/bin/env python3
from pyais.stream import TCPConnection
from geopy import distance 
from geopy import units
from Behaviour.Behaviours.Classes.GPS_class import GPS
import math as m
from Behaviour.Behaviours.Classes.Vector_class import Vector 
from Behaviour.Behaviours.Classes.AIS_class import avoidance_vectors

host = "153.44.253.27"
port = 5631

mylat = 60.3947
mylon = 5.2675 
buffer = 0.1
own_pos = (mylat,mylon)

# Regner ut avstanden mellom 2 punkter. Brukes til å finne avstand til andre fartøyer
def calc_distance(cord1, cord2):
    own = cord1
    other = cord2
    dist = distance.distance(own,other).km
    return dist 


file_clear = open(r"Vecs_from_ais","w")
file_clear.truncate()
file_clear.close
for msg in TCPConnection(host, port):
    # print(test)
    boat = msg.decode() #Dekoder AIS meldingen
    ais_content = boat
    ais_dict = ais_content.asdict() # Gjør den dekodede meldingen om til en dict så den kan brukes i koden
    tot_vector = Vector(0,0)
    #print(ais_dict)
    # Filter for å få bort AIS meldinger som ikke er relevante for navigering av dronene
    if ais_dict["msg_type"] == 1 or ais_dict["msg_type"] == 2 or ais_dict["msg_type"] == 3 or ais_dict["msg_type"] == 18 or ais_dict["msg_type"] == 19:

        #Filter for å få bort fartøy langt unna
        if ((ais_dict["lon"] < mylon + buffer) and  (ais_dict["lon"] > mylon - buffer)) and  ((ais_dict["lat"] < mylat + buffer) and  (ais_dict["lat"] > mylat - buffer)):
            #print(ais_dict)
            other_pos = (ais_dict["lat"], ais_dict["lon"])
            dist = calc_distance(own_pos, other_pos)
            #yeet = calc_bearing(own_pos, other_pos)

            instsans = avoidance_vectors(other_pos, ais_dict["speed"], ais_dict["course"])
            instans_vectors = instsans(own_pos,other_pos,ais_dict["speed"],ais_dict["course"])
            #Henter ut de to listene som returneres
            get_vec_list = instans_vectors[0]
            get_distances = instans_vectors[1]
            counter1 = 0
            print("*"*80)
            for vec in get_vec_list:
                r = get_distances[counter1]
                print(r)

                weight = (3/(m.pow(r,2)+0.25))
                temp_vec = vec * weight
                tot_vector += temp_vec
                counter1 += 1
            print("*"*80)
             
            vec_magn_string = str(tot_vector.magnitude)  
            vec_ang_string = str(tot_vector.angle)
            space = "-"
            tot_vector_string = vec_magn_string + space + vec_ang_string
            file = open(r"Vecs_from_ais","a")
            file.write(tot_vector_string)
            file.write("\n")
            file.close
                


