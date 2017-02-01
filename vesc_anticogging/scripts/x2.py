from math import pi
from data2 import currents, currents_flipped
from data import currents as currents_orig
import matplotlib.pyplot as plt

def sketchy_filter(data):
   window = 3
   for i in range(window, len(data) - window):
      data[i] = list(data[i])
      for j in range(1, window + 1):
         data[i][0] += data[i - j][0]
         data[i][0] += data[i + j][0]
      data[i][0] /= window * 2 + 1

def discretify(input):
   data = [None] * 360 * 4
   for point in input:
      current = point[0]
      position = point[1]
      index = int(round(position / 2 / pi * len(data)))
      if index < 0 or index >= len(data):
         continue
      data[index] = current

   # interpolation
   prev = -1
   for i in range(len(data)):
      value = data[i]

      if value != None and prev != i - 1 and prev != -1:
         for j in range(prev + 1, i):
            data[j] = data[prev] + (data[i] - data[prev]) * ((j - prev) * 1.0 / (i - prev))

      if value != None:
         prev = i

   for i in range(len(data)):
      if data[i] == None:
         data[i] = 0

   return data

# currents = currents[:200:1]
plt.plot([x * 2 * pi / len(currents_orig) for x in range(len(currents_orig))], currents_orig, color='#dddddd')
data = sorted(currents, lambda a, b: (1 if a[1] - b[1] > 0 else -1))

plt.plot([c[1] for c in data], [c[0] for c in data], color='#aaaaff')
sketchy_filter(data)
plt.plot([c[1] for c in data], [c[0] for c in data], color='#0000ff')

data1 = discretify(data)

currents = [(-x[0], x[1]) for x in currents_flipped]

data = sorted(currents, lambda a, b: (1 if a[1] - b[1] > 0 else -1))
plt.plot([c[1] for c in data], [c[0] for c in data], color='#ffaaaa')
sketchy_filter(data)
plt.plot([c[1] for c in data], [c[0] for c in data], color='#ff0000')
# plt.plot([c[1] for c in data], [c[0] for c in data], 'ro')

data2 = discretify(data)

data = data1
plt.plot([i * 2 * pi / len(data) for i in range(len(data))], data, color='#000077')
data = data2
plt.plot([i * 2 * pi / len(data) for i in range(len(data))], data, color='#770000')

data = []
for i in range(len(data1)):
   data.append((data1[i] + data2[i]) / 2.0)
plt.plot([i * 2 * pi / len(data) for i in range(len(data))], data, color='#000000')

plt.show()
