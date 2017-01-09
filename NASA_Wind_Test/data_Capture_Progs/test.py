import os

filename = "IMU_Data1.txt"
i=1
while os.path.isfile(filename):
    i = i + 1
    filename = filename[:8] + str(i) + '.txt'

with open(filename, 'w') as f:
    f.write("Testing")
