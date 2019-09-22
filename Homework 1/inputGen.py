from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

size = 100

mat = [[0 for i in range(size)] for j in range(size)]


def circle(val, radius):
    for i in range(radius, size - radius):
        mat[radius][i] = val
        mat[i][radius] = val
        mat[size - radius - 1][i] = val
        mat[i][size - radius - 1] = val


for i in range(1, int((size - 10) / 2)):
    circle((10*i), i)

f = open("inputGen.txt", "w+")

f.write("A*\n")
f.write(str(size) + " " + str(size) + "\n")
f.write("1 1\n")
f.write("50\n")
f.write("1\n")
#f.write(str(size - 2) + " " + str(size - 2) + "\n")
f.write("80 60\n")

for i in range(size):
    print("Writing line:", i)
    f.write(" ".join(map(str, mat[i])) + "\n")

f.close()
