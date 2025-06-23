import pandas as pd
import matplotlib.pyplot as plt


df = pd.read_csv('odom_output.csv')


t = df['timestamp'] - df['timestamp'][0]
v = df['v'].values
w = df['w'].values


plt.figure(figsize=(12, 8))

#v(t)
plt.subplot(2, 1, 1)
plt.plot(t, v, 'g-')
plt.title('v(t)')
plt.xlabel('time[s]')
plt.ylabel('v [m/s]')
plt.grid()

#ω(t)
plt.subplot(2, 1, 2)
plt.plot(t, w, 'r-')
plt.title('w(t)')
plt.xlabel('time[s]')
plt.ylabel('w[rad/s]')
plt.grid()

plt.tight_layout()
plt.show()

