import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


df = pd.read_csv('odom2_output.csv')


df['t'] = df['timestamp'] - df['timestamp'][0]

#dimioyrgia grammikis taxititas
df['v'] = np.abs(df['vx'])

#upologismos prosanatolismou
df['theta'] = np.cumsum(df['wz'] * np.gradient(df['t']))


plt.figure(figsize=(12, 9))

#gwniaki kai grammiki
plt.subplot(3, 1, 1)
plt.plot(df['t'], df['wz'], 'b-', label='w(t) [rad/s]')
plt.plot(df['t'], df['v'], 'g-', label='v(t) [m/s]')
plt.title('gwniaki kai grammiki taxitita')
plt.ylabel('[rad/s] / [m/s]')
plt.legend()
plt.grid(True)

#prosanatolismos
plt.subplot(3, 1, 2)
plt.plot(df['t'], df['theta'], 'm-', label='θ(t) [rad]')
plt.title('θ(t)')
plt.ylabel('θ(t) [rad]')
plt.grid(True)

#theseis x(t) και y(t)
plt.subplot(3, 1, 3)
plt.plot(df['t'], df['x'], 'k-', label='x(t)')
plt.plot(df['t'], df['y'], 'gray', label='y(t)')
plt.title('x(t) και y(t)')
plt.xlabel('t [s]')
plt.ylabel('[m]')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()

