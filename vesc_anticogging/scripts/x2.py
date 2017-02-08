from math import pi
from data2 import currents, currents_flipped
from data import currents as currents_orig
import matplotlib.pyplot as plt

def sketchy_filter(data):
   window = 2
   for i in range(len(data)):
      data[i] = list(data[i])
      for j in range(1, window + 1):
         data[i][0] += data[(i - j) % len(data)][0]
         data[i][0] += data[(i + j) % len(data)][0]
      data[i][0] /= window * 2 + 1

def sketchy_filter_bins(data):
   window = 2
   for i in range(window, len(data) - window):
      for j in range(1, window + 1):
         data[i] += data[i - j]
         data[i] += data[i + j]
      data[i] /= window * 2 + 1

def discretify(input):
   bins = 360 * 4
   data = [0] * bins
   count = [0] * bins
   for point in input:
      current = point[0]
      position = point[1]
      index = int(round(position / 2 / pi * len(data)))
      if index < 0 or index >= len(data):
         continue
      data[index] += current
      count[index] += 1

   for i in range(len(count)):
      if count[i] != 0:
         data[i] /= count[i]

   # interpolation
   prev = -1
   interpolated = 0
   for i in range(len(data)):
      value = data[i]

      if count[i] != 0 and prev != i - 1 and prev != -1:
         for j in range(prev + 1, i):
            data[j] = data[prev] + (data[i] - data[prev]) * ((j - prev) * 1.0 / (i - prev))
            interpolated += 1

      if count[i] != 0:
         prev = i

   # print str(interpolated * 100.0 / bins) + "% interpolated"

   return data

# currents = currents[:200:1]

data = sorted(currents, lambda a, b: (1 if a[1] - b[1] > 0 else -1))
data1 = discretify(data)

plt.plot([x * 2 * pi / len(currents_orig) for x in range(len(currents_orig))], currents_orig, color='#ddffdd')
for i in range(len(currents_orig)):
   if data1[int(i * 1.0 / len(currents_orig) * len(data1))] * currents_orig[i] < 0:
      currents_orig[i] *= -1
sketchy_filter_bins(currents_orig)
plt.plot([x * 2 * pi / len(currents_orig) for x in range(len(currents_orig))], currents_orig, color='#88ff88')

plt.plot([c[1] for c in data], [c[0] for c in data], color='#aaaaff')
sketchy_filter(data)
plt.plot([c[1] for c in data], [c[0] for c in data], color='#0000ff')


currents = [(-x[0], x[1]) for x in currents_flipped]

data = sorted(currents, lambda a, b: (1 if a[1] - b[1] > 0 else -1))
data2 = discretify(data)
plt.plot([c[1] for c in data], [c[0] for c in data], color='#ffaaaa')
sketchy_filter(data)
plt.plot([c[1] for c in data], [c[0] for c in data], color='#ff0000')
# plt.plot([c[1] for c in data], [c[0] for c in data], 'ro')


data = data1
plt.plot([i * 2 * pi / len(data) for i in range(len(data))], data, color='#000077')
data = data2
plt.plot([i * 2 * pi / len(data) for i in range(len(data))], data, color='#770000')

data = []
for i in range(len(data1)):
   data.append((data1[i] + data2[i]) / 2.0)
plt.plot([i * 2 * pi / len(data) for i in range(len(data))], data, color='#000000')

plt.show()

sketchy_filter_bins(data)

plt.plot([x * 2 * pi / len(currents_orig) for x in range(len(currents_orig))], currents_orig, color='#00ff00')
plt.plot([i * 2 * pi / len(data) for i in range(len(data))], data, color='#000000')

data = [round(d, 5) for d in data]

print "lookup_table = " + str(data)
# plt.plot([i * 2 * pi / len(data) for i in range(len(data))], data, 'ro', color='#000000')
plt.show()
