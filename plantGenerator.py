from random import shuffle

figures = ["rot", "blau", "gelb", "grün", "frei", "frei"]
platzierungen = [1, 2, 3, 4, 5, 6]
shuffle(platzierungen)

print("Deine Platzierungen:")

for i, j in zip(figures, platzierungen):
    print(str(i) + "  kommt auf Platz  " + str(j))
