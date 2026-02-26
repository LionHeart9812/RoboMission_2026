from random import shuffle

figures = ["rot", "blau", "gelb", "grün", "schwarz"]
platzierungen = [1, 2, 3, 4, "nichts",]
shuffle(platzierungen)

print("Deine Platzierungen:")

for i, j in zip(figures, platzierungen):
    print(str(i) + "  kommt auf Platz  " + str(j))
