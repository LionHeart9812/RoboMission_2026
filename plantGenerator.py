from random import shuffle

figures = ["1. Ente", "2. Ente", "1. Palme", "2. Palme", "1. Blume", "2. Blume"]
platzierungen = [1, 2, 3, 4, 5, 6]
shuffle(platzierungen)

print("Deine Platzierungen:")

for i, j in zip(figures, platzierungen):
    print(str(i) + "  kommt auf Platz  " + str(j))
