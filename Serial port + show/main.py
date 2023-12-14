import os
#pathfinding
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from pathfinding.core.diagonal_movement import DiagonalMovement
import math
#Serial
import re
import serial
import time

# Rozmiar macierzy
size = 11
ROWS, COLS = size, size
#Inicjazliacja portu szeregowego
ser = serial.Serial(port="COM7",baudrate=115200, bytesize=8,timeout=2, stopbits=serial.STOPBITS_ONE)
# Pozycja osoby
x, y = 0, 0
player_pos = [x, y]
# Create a 2D matrix
matrix = [[1 for _ in range(COLS)] for _ in range(ROWS)]
#Stała do określania odległości
previous_dist = float(0)

def get_position(decoded):
    wyrazenie_regularne = r'est\[(.*?),(.*?),(.*?),(.*?)\]'
    wyniki = re.search(wyrazenie_regularne, decoded)
    if wyniki is not None:
        x = float(wyniki.group(1))
        y = float(wyniki.group(2))
        print(x, y)
        # if x >= 1:
        #     x = 0.99
        # if x <= 0:
        #     x = 0.01
        # if y <= 0:
        #     y=0.01
        # if y >= 1:
        #     y = 0.99
        player_pos[0]= int(x * size)
        player_pos[1] = int(size - (y * size))
        print(player_pos)

def distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

# Znajdź punkt z listy, który jest najbliższy aktualnej pozycji
def find_nearest_point(current_x, current_y, points):
    nearest_point = None
    min_distance = float('inf')  # Początkowo ustawiamy na nieskończoność

    for point in points:
        dist = distance(current_x, current_y, point[0], point[1])
        if dist < min_distance:
            min_distance = dist
            nearest_point = point

    return nearest_point, min_distance

def print_matrix():
    for i in range(ROWS):
        for j in range(COLS):
            if [j, i] == player_pos:
                print('P', end=' ')
            else:
                print(matrix[i][j], end=' ')
        print()

#Setup do Path findera
grid = Grid(matrix=matrix)
start = grid.node(player_pos[0],player_pos[1])
end = grid.node(size-1,size-1)
finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
path_tmp, runs = finder.find_path(start, end, grid)
path = []
path.clear()

for i in range(len(path_tmp)):
    path.append((path_tmp[i].x, path_tmp[i].y))

# Game loop
while True:
    #SERIAL
    receive = ser.readline()
    decoded = receive.decode('Ascii')
    get_position(decoded)

    #-----------------
    nearest_point,dist = find_nearest_point(player_pos[0], player_pos[1], path)
    if dist-previous_dist > 0:
        print("Zły kierunek")
    else:
        print('Dobry keirunek')
    previous_dist = dist
    print_matrix()

    #time.sleep(1)




