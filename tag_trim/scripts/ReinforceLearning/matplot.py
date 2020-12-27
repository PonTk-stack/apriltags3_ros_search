import matplotlib.pyplot as plt
import numpy as np

#%matplotlib inline

fig, ax = plt.subplots(1, 1)

ax.set_ylim((-1.1, 1.1))

x = np.arange(0, 100, 0.5)

for Hz in np.arange(0.1, 10.1, 0.01):
  y = np.sin(2.0 * np.pi * (x * Hz) / 100)
  line, = ax.plot(x, y, color='blue')
  plt.pause(0.01)
  line.remove()
