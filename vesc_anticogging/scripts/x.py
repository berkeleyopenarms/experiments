from data import currents
from math import pi
import matplotlib.pyplot as plt

# currents = currents[:200:1]

plt.plot([x * 2 * pi / len(currents) for x in range(len(currents))], currents)
plt.plot([x * 2 * pi / len(currents) for x in range(len(currents))], currents, 'ro')
plt.show()
