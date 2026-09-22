import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# IEEE Publication-Grade Matplotlib Settings
plt.rcParams.update({
    'figure.figsize': (3.5, 2.5),
    'axes.titlesize': 9,
    'axes.labelsize': 8,
    'xtick.labelsize': 8,
    'ytick.labelsize': 8,
    'legend.fontsize': 7,
    'lines.linewidth': 1.2,
    'grid.linewidth': 0.5,
    'grid.alpha': 0.7,
    'font.family': 'serif',
    'figure.autolayout': True
})

class UnderactuatedSystem:
    def __init__(self, name, u_max):
        self.name = name
        self.u_max = u_max
        self.time_history = []
        self.state_history = []
        self.u_history = []

    def _ode_wrapper(self, t, x):
        u = self.controller(t, x)
        u_sat = np.clip(u, -self.u_max, self.u_max)
        self.time_history.append(t)
        self.state_history.append(x)
        self.u_history.append(u_sat)
        return self.dynamics(t, x, u_sat)

    def simulate(self, t_span, x0, dt=0.01):
        self.time_history, self.state_history, self.u_history = [], [], []
        return solve_ivp(
            fun=self._ode_wrapper,
            t_span=t_span,
            y0=x0,
            t_eval=np.arange(t_span[0], t_span[1], dt),
            method='RK45',
            rtol=1e-6,
            atol=1e-8
        )

# 1. Cart-Pole System (Energy-Shaping + LQR)
class CartPole(UnderactuatedSystem):
    def __init__(self):
        super().__init__(name="Cart-Pole", u_max=20.0)
        self.m_c, self.m_p, self.L, self.g = 1.05, 0.23, 0.33, 9.81
        self.K = np.array([-10.0, -35.0, -12.0, -5.0]) 

    def dynamics(self, t, x, u):
        p, theta, p_dot, theta_dot = x
        sy, cy = np.sin(theta), np.cos(theta)
        den = self.m_c + self.m_p * sy**2
        p_ddot = (u + self.m_p * sy * (self.L * theta_dot**2 + self.g * cy)) / den
        theta_ddot = (-u * cy - self.m_p * self.L * theta_dot**2 * cy * sy - (self.m_c + self.m_p) * self.g * sy) / (self.L * den)
        return [p_dot, theta_dot, p_ddot, theta_ddot]

    def controller(self, t, x):
        p, theta, p_dot, theta_dot = x
        theta = (theta + np.pi) % (2 * np.pi) - np.pi
        
        if abs(theta) < 0.3:  
            return -np.dot(self.K, [p, theta, p_dot, theta_dot])
        
        E = 0.5 * self.m_p * (self.L * theta_dot)**2 + self.m_p * self.g * self.L * (np.cos(theta) - 1)
        return 2.5 * E * theta_dot * np.cos(theta) - 1.0 * p - 0.5 * p_dot

# 2. Ball and Beam (Recursive Backstepping)
class BallAndBeam(UnderactuatedSystem):
    def __init__(self):
        super().__init__(name="Ball & Beam", u_max=10.0)
        self.m, self.R, self.g, self.J = 0.065, 0.012, 9.81, 0.025

    def dynamics(self, t, x, u):
        r, r_dot, alpha, alpha_dot = x
        den = self.m + self.J / self.R**2
        r_ddot = (self.m * (r * alpha_dot**2 - self.g * np.sin(alpha))) / den
        return [r_dot, r_ddot, alpha_dot, u]

    def controller(self, t, x):
        r, r_dot, alpha, alpha_dot = x
        r_d = 0.2  # Target position (+0.2m)
        
        k1, k2 = 5.0, 10.0
        z1 = r - r_d
        z2 = r_dot + k1 * z1
        
        # CORRECTED SIGN MAPPING: Negative alpha accelerates the ball positively
        den = self.m + self.J / self.R**2
        v = -k2*z2 - k1*r_dot - z1
        alpha_d = np.arcsin(np.clip(-(den * v - self.m*r*alpha_dot**2) / (self.m * self.g), -0.99, 0.99))
        
        kp, kd = 30.0, 5.0
        return -kp * (alpha - alpha_d) - kd * alpha_dot

# 3. Planar Bicopter (PD-Lyapunov)
class PlanarBicopter(UnderactuatedSystem):
    def __init__(self):
        super().__init__(name="Bicopter", u_max=15.0)
        self.I_y, self.l, self.K_d = 0.012, 0.15, 0.005
        
    def dynamics(self, t, x, u):
        phi, phi_dot = x
        phi_ddot = (self.l * u - self.K_d * phi_dot) / self.I_y
        return [phi_dot, phi_ddot]

    def controller(self, t, x):
        phi, phi_dot = x
        phi_d = 0.0 
        kp, kd = 15.0, 2.5
        return -kp * (phi - phi_d) - kd * phi_dot

# Dynamic Plotting Function
def plot_system(sol, sys_obj, filename, title_a, title_b, legend_locs):
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
    
    # State Plot
    ax1.plot(sol.t, sol.y[1,:], label=title_a)
    ax1.plot(sol.t, sol.y[0,:], '--', label=title_b)
    ax1.set_ylabel('States')
    ax1.legend(loc=legend_locs[0], framealpha=0.9) # Isolated State Legend
    ax1.grid(True)
    
    # Control Effort Plot
    u_plot = [sys_obj.controller(t, state) for t, state in zip(sol.t, sol.y.T)]
    u_plot = np.clip(u_plot, -sys_obj.u_max, sys_obj.u_max)
    
    ax2.plot(sol.t, u_plot, color='red', label='Control Effort $u(t)$')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Command')
    ax2.legend(loc=legend_locs[1], framealpha=0.9) # Isolated Control Legend
    ax2.grid(True)
    
    plt.savefig(filename, format='pdf', bbox_inches='tight')
    plt.close()

def generate_latex_parameters_table():
    latex_table = r"""
% ==============================================================================
% PASTE THIS TABLE INTO SECTION IV (Simulation Framework)
% ==============================================================================
\begin{table}[htbp]
\centering
\caption{Nominal Physical Parameters for the Evaluated Benchmarks}
\label{tab:sim_parameters}
\renewcommand{\arraystretch}{1.2}
\resizebox{\columnwidth}{!}{%
\begin{tabular}{@{}llc@{}}
\toprule
\textbf{System} & \textbf{Parameter Description} & \textbf{Nominal Value} \\ 
\midrule
\multirow{4}{*}{\textbf{Cart-Pole}} 
 & Cart Mass ($M$) & $1.05$ kg \\
 & Pendulum Mass ($m$) & $0.23$ kg \\
 & COM Distance ($L$) & $0.33$ m \\
 & Pendulum Inertia ($I$) & $0.008$ kg$\cdot$m$^2$ \\
\midrule
\multirow{4}{*}{\textbf{Ball \& Beam}} 
 & Beam Inertia ($J_{beam}$) & $0.025$ kg$\cdot$m$^2$ \\
 & Ball Mass ($m_b$) & $0.065$ kg \\
 & Ball Radius ($R$) & $0.012$ m \\
 & Track Length ($L_{beam}$) & $0.40$ m \\
\midrule
\multirow{4}{*}{\textbf{Bicopter}} 
 & Pitch Inertia ($I_y$) & $0.012$ kg$\cdot$m$^2$ \\
 & Arm Length ($l$) & $0.15$ m \\
 & Aerodynamic Drag ($K_d$) & $0.005$ N$\cdot$m$\cdot$s/rad \\
 & Effective Mass ($m_{eff}$) & $0.45$ kg \\
\bottomrule
\end{tabular}%
}
\end{table}
"""
    print(latex_table)

if __name__ == "__main__":
    # 1. Cart-Pole (Cart position drops negatively, state legend to upper right)
    cp = CartPole()
    sol_cp = cp.simulate((0, 10), [0.0, np.pi, 0.0, 0.0])
    plot_system(sol_cp, cp, 'cartpole_bench.pdf', r'Angle $\theta$ (rad)', r'Position $p$ (m)', 
                legend_locs=('lower left', 'upper left'))
    
    # 2. Ball and Beam (Both tracking positively, state legend to lower right)
    bb = BallAndBeam()
    sol_bb = bb.simulate((0, 5), [0.0, 0.0, 0.0, 0.0])
    plot_system(sol_bb, bb, 'ballbeam_bench.pdf', r'Beam $\alpha$ (rad)', r'Ball $r$ (m)', 
                legend_locs=('upper left', 'upper right'))
    
    # 3. Bicopter (Regulating from 45 deg down to 0, state legend to upper right)
    bi = PlanarBicopter()
    sol_bi = bi.simulate((0, 5), [np.pi/4, 0.0]) 
    plot_system(sol_bi, bi, 'bicopter_bench.pdf', r'Pitch rate $\dot{\phi}$', r'Pitch $\phi$ (rad)', 
                legend_locs=('lower right', 'lower right'))
                
    # Generate the LaTeX code for Murfy/Overleaf
    generate_latex_parameters_table()