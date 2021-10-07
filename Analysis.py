import numpy as np

ac = []
total = []

file = open('data.dat', 'r')
for line in file.readlines():
    fname = line.rstrip().split(',') #using rstrip to remove the \n
    total.append([float(i) for i in fname])

for i in range(0,len(total),2):
    ac.append([total[i],total[i+1]])

#Data is now available as: loc = ac[i][0], time = ac[i][1]

