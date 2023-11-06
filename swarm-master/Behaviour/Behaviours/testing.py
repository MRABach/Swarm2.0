from Classes.AIS_class import avoidance_vectors
from Classes.Vector_class import Vector

mylat = 60.4
mylon = 5.3
mybearing = 120
myspeed = 5


own_pos = (60.3947,5.2675)
andre = (mylat, mylon)

test = avoidance_vectors(own_pos,10,mybearing) # denne kaller på klassen slik at test blir en instance av klassen
test_liste = test(own_pos,andre,myspeed) # Denne initsierer __call__ funksjonen i python som returnerer en liste med vektorer

for t in test_liste:
    print(t.magnitude) #Styrken på vektoren
    print(t.angle) # Vinkelen på vektoren