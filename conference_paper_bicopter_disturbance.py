import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

plt.rcParams.update({
    'figure.figsize': (3.5, 2.5), 'axes.titlesize': 9, 'axes.labelsize': 8,
    'xtick.labelsize': 8, 'ytick.labelsize': 8, 'legend.fontsize': 7,
    'lines.linewidth': 1.2, 'grid.linewidth': 0.5, 'grid.alpha': 0.7,
    'font.family': 'serif', 'figure.autolayout': True
})

class PlanarBicopter:
    def __init__(self):
        self.I_y, self.l, self.K_d, self.u_max = 0.012, 0.15, 0.005, 15.0
    def dynamics(self, t, x, u):
        return [x[1], (self.l * u - self.K_d * x[1]) / self.I_y]
    def controller(self, t, x):
        return -15.0 * (x[0] - 0.0) - 2.5 * x[1]
    def ode_wrapper(self, t, x):
        u_sat = np.clip(self.controller(t, x), -self.u_max, self.u_max)
        return self.dynamics(t, x, u_sat)

# Simulate Phase 1: 0 to 2 seconds (Nominal hover)
bi = PlanarBicopter()
sol1 = solve_ivp(bi.ode_wrapper, (0, 2), [0.0, 0.0], t_eval=np.arange(0, 2, 0.01))

# Inject Disturbance: +0.5 rad at t=2s
state_disturbed = [sol1.y[0,-1] + 0.5, sol1.y[1,-1]]

# Simulate Phase 2: 2 to 5 seconds (Recovery)
sol2 = solve_ivp(bi.ode_wrapper, (2, 5), state_disturbed, t_eval=np.arange(2, 5, 0.01))

# Combine results
t_total = np.concatenate((sol1.t, sol2.t))
y_total = np.concatenate((sol1.y, sol2.y), axis=1)
u_total = np.clip([bi.controller(t, state) for t, state in zip(t_total, y_total.T)], -bi.u_max, bi.u_max)

# Generate Plot
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
ax1.plot(t_total, y_total[0,:], label=r'Pitch $\phi$ (rad)')
ax1.axvline(2.0, color='gray', linestyle='--', alpha=0.5)
ax1.annotate('Disturbance', xy=(2.0, 0.4), xytext=(2.2, 0.4), arrowprops=dict(arrowstyle="->", color='gray'), fontsize=7)
ax1.set_ylabel('State')
ax1.legend(loc='lower right', framealpha=0.9)
ax1.grid(True)

ax2.plot(t_total, u_total, color='red', label='Control Effort')
ax2.axvline(2.0, color='gray', linestyle='--', alpha=0.5)
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Force (N)')
ax2.legend(loc='lower right', framealpha=0.9)
ax2.grid(True)

plt.savefig('bicopter_disturbance.pdf', format='pdf', bbox_inches='tight')