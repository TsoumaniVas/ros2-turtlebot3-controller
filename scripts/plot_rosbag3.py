import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


df = pd.read_csv('odom3_output.csv')  


df['t'] = df['timestamp'] - df['timestamp'][0]

#grammiki taxitita
df['v'] = np.abs(df['vx'])


plt.figure(figsize=(14, 10))

#taxitites
plt.subplot(4, 1, 1)
plt.plot(df['t'], df['v'], 'g-', label='v(t) [m/s]')
plt.plot(df['t'], df['wz'], 'b-', label='w(t) [rad/s]')
plt.title('taxitites : grammiki kai goniaki')
plt.ylabel('[m/s] & [rad/s]')
plt.legend()
plt.grid(True)

#θ(t)
plt.subplot(4, 1, 2)
plt.plot(df['t'], df['theta'], 'm-', label='θ(t) [rad]')
plt.title('θ(t)')
plt.ylabel('[rad]')
plt.grid(True)

#x(t), y(t)
plt.subplot(4, 1, 3)
plt.plot(df['t'], df['x'], 'r-', label='x(t) [m]')
plt.plot(df['t'], df['y'], 'orange', label='y(t) [m]')
plt.title('x(t) y(t)')
plt.ylabel('[m]')
plt.legend()
plt.grid(True)

#troxia x–y
plt.subplot(4, 1, 4)
plt.plot(df['x'], df['y'], 'k-')
plt.title('troxia x–y')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.axis('equal')
plt.grid(True)

plt.tight_layout()
plt.show()

