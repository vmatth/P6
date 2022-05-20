import random

parcels = [1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5]



for i in range(len(parcels)):
    j = random.randrange(0,len(parcels))
    type = parcels[j]

    #remove parcel from list
    parcels.pop(j)
    pos = (random.randrange(0, 5) + 1)
    orientation = random.randrange(0, 3) + 1
    angle = random.randrange(0, 91)

    print("-----Parcel ", str(i + 1))
    print("Type: ", type)
    print("Pos: ", pos)
    print("Orientation: ", orientation)
    print("Angle: ", angle)

