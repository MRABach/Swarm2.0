from pyais.stream import TCPConnection
from geopy import distance 
from Classes.GPS_class import GPS
import math as m
from Classes.Vector_class import Vector 

# Variabler for å se på kystverket sin AIS
host = "153.44.253.27"
port = 5631

intervals = [60, 300, 600]


"""
 Dette legges til når man kjører scriptet i en raspberry pi
 pos_dict = GPS()
 mylat = pos_dict["lat"]
 mylon = pos_dict["lon"]
"""
mylat = 60.3947
mylon = 5.2675
mybearing = 120
myspeed = 5

own_pos = (mylat, mylon)
buffer = 0.1



# Regner ut avstanden mellom 2 punkter. Brukes til å finne avstand til andre fartøyer
def calc_distance(cord1, cord2):
    own = cord1
    other = cord2
    dist = distance.distance(own,other).km
    return dist 

# Funksjon som regner ut bearing fra et punkt A til et punkt B. Brukes for å finne vinkel mellom eget og andre fartøyer
def calc_bearing(cordA, cordB):
    PointA = cordA
    PointB = cordB
    delta_lon =  PointB[1] - PointA[1]
    #delta_lat = PointA["lat"] - PointB["lat"]
    X = m.cos(PointB[0]*(m.pi/180))*m.sin(delta_lon*(m.pi/180))
    Y = m.cos(PointA[0]*(m.pi/180))*m.sin(PointB[0]*(m.pi/180))-m.sin(PointA[0]*(m.pi/180))*m.cos(PointB[0]*(m.pi/180))*m.cos(delta_lon*(m.pi/180))
    sigma = m.atan2(X,Y)
    bearing = m.degrees(sigma)
    return bearing


# Funksjon som definerer hvor et fartøy kommer fra basert på vinkelen fra eget fartøy til punktet. Brukes til å bestemme unnvikelsesmønster
"""
def rel_pos(angle):
    if angle <= 0 and angle >= -90:
        direction = "NW"
    elif angle >0 and angle <= 90:
        direction = "NE"
    elif angle > 90 and angle < 180:
        direction = "SE"
    elif angle < -90 and angle > -180:
        direction = "SW"    
    else:
        print("Shits fucked")
        angle = "error"
    return direction
"""
#"""
p1 = (39.099912, -94.581213)
p2 = (38.627089, -90.200203)
testbearing = calc_bearing(p1, p2)
#print(testbearing)
#"""

#Returnerer en liste med tuples som inneholder koordinatene.
def create_aliases(cord, speed, rel_angle):
   
    
    la1 = cord[0]
    lo1 = cord[1]
    
    speed_kph = speed * 1.85200
    distances = [i * speed_kph for i in intervals] 
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

"""
def calc_avoidance_angle(quadrant, other_heading):
    if quadrant == "NW":
        avoid = other_heading - 90
    elif quadrant == "NE":
        avoid = other_heading + 90
    elif quadrant == "SW":
        avoid = other_heading -180
    elif quadrant == "SE":
        avoid = other_heading + 180
    else:
        print(":)")
    
    if avoid > 360:
        vec_angle = avoid -360
    elif avoid < 0:
        vec_angle = avoid + 360
    else:
        vec_angle = avoid
    return vec_angle
"""
own_aliases = create_aliases(own_pos, myspeed, mybearing)

 
testspeed = 5
kordinat = p2
testalias = create_aliases(kordinat,testspeed, testbearing)
print(testalias)


def make_avoid_vec(own_position, other_positions):
    angle_list = []
    vector_list = []
    for positions in other_positions:
        ang = calc_bearing(own_position,positions)
        avoid_ang = ang + 180
        if avoid_ang > 360:
            avoid_ang = avoid_ang - 360
        else:
            avoid_ang = avoid_ang
        angle_list.append(avoid_ang)
    for i in range(len(angle_list)):
        vec = Vector(1/(i+1),angle_list[i]) # Lager vektorer der fremtidige posisjoner blir svakere vektet
        vector_list.append(vec)
    return vector_list

test_list = make_avoid_vec(p1,testalias)
print("*" *80)
for ye in test_list:
    print(ye.angle)
    print(ye.magnitude)
print("*" *80)
"""
def check_collision(own_positions, other_positions):
    for positions in own_positions:
        for points in other_positions:
            margin = calc_distance(positions,points)
            if margin > 2:

"""
while True:
    #try: 
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
                test = calc_distance(own_pos, other_pos)
                yeet = calc_bearing(own_pos, other_pos)

                """
                intsans = avoidance_vectors(other_pos, ais_dict["speed"], ais_dict["course"])
                
                """



                #yayeet = rel_pos(yeet)
                print(test)
                print(yeet)
                #print(yayeet)
    #except KeyboardInterrupt, e:    
      #  break





