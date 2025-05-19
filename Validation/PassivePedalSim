import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# used ChatGPT for help, see https://chatgpt.com/share/6822e03f-87b0-8012-afe8-0a1ccc3806e4

# Systemparameter
m_p = 1e3       # Pedalmasse [kg]
#b_p = 1e2       # Dämpfung [Ns/m]
k_p = 1e6     # Mechanische Rückstellfeder [N/m]
zeta = 0.5 # 0 = keine Dämpfung, 1 = kritisch gedämpft, >1 = überdämpft
b_critical = 2 * np.sqrt(m_p * k_p)
b_p = b_critical * zeta

# Ziel-Kraft-Weg-Kennlinie: F_target(x) = a*x + b*x^2 + c*x^3
a, b, c = 10000.0, 300000.0, 0.0

# PI-Reglerparameter
K_p = 0.01     # Proportionalanteil
K_i = 1.0     # Integralanteil

# Begrenzungen

rpm_max = 250000 / 6400 * 60 # in rev per minute
spindle_ptch = 0.01 # in meter

x_min, x_max = -0.0, 0.1         # [m] = 0–50 mm Pedalweg
v_max = rpm_max * spindle_ptch / 60 #0.5                      # [m/s] = 500 mm/s
a_max = 500.0                     # [m/s²]


# Eingabekraft durch Benutzer (Wägezelle-Simulation)
def user_position(t):
    return 0.05 if 0.2 <= t <= 1.5 else 0.0  # Benutzer drückt kurz das Pedal

def user_force(t):
    return 500.0 if 0.2 <= t <= 1.5 else 0.0  # Benutzer drückt kurz das Pedal

    #return 50.0 * 9.81 if 0.2 <= t <= 1.5 else 0.0  # Benutzer drückt kurz das Pedal


# Ziel-Kraft-Weg-Kennlinie
def F_target(x):
    # example values
    # travel = 10mm = 0.01m
    # force at max travel = 100kg = 100kg * 9.81m/s^2 = 9810N
    #a = 9810 / 0.1 = 9810 ca 1e4
    force =  a * x + b * x**2 + c * x**3
    gradient =  a  + 2 * b * x + 3* c * x**2
    return [force, gradient]

# soft endstop
def saturation_force(x):
    if x < x_min:
        return -1e7 * (x - x_min) # when x < x_min --> (x - x_min) < 0 --> positive acceleration needed
    elif x > x_max:
        return -1e7 * (x - x_max) # when x > x_max --> (x - x_max) > 0 --> negative acceleration needed
    else:
        return 0.0


# # hard endstop
# def hit_x_max(t, y):
#     return y[0] - x_max  # triggers when x > x_max
#
# def hit_x_min(t, y):
#     return y[0] - x_min  # triggers when x < x_min
#
# hit_x_max.terminal = True
# hit_x_max.direction = 1  # only trigger when crossing upwards
#
# hit_x_min.terminal = True
# hit_x_min.direction = -1  # only trigger when crossing downwards

# ODE-System
def pedal_dynamics(t, y):
    x, v = y  # Position, Geschwindigkeit, Integralregler

    # Optional: Anschlagprüfung
    # if x < x_min:
    #     v = 0
    #     a_pedal = 0
    #     return [0.0, 0.0]
    # elif x > x_max:
    #     v = 0
    #     a_pedal = 0
    #     return [0.0, 0.0]

    # soft endstops
    F_sat = saturation_force(x)

    #F_user = user_force(t)
    x_user = user_position(t)
    F_desired, F_gradient = F_target(x)

    # Dynamikgleichung (Newton 2)
    a_pedal = (-F_desired - b_p * v - k_p * (x - x_user) + F_sat) / m_p

    if (t > 0.75):
        tmp = 5

    if (abs(a_pedal) > 0):
        tmp = 5

    # Begrenzung der Beschleunigung
    a_pedal = np.clip(a_pedal, -a_max, a_max)

    # Begrenzung der Geschwindigkeit (Rückgabewert)
    v = np.clip(v, -v_max, v_max)

    return [v, a_pedal]

# Anfangsbedingungen: [Position, Geschwindigkeit, Integralzustand]
y0 = [0.0, 0.0]

# Simulationszeitraum
t_span = (0, 2.0)
t_eval = np.linspace(*t_span, 2000)

# Numerische Lösung
#solution = solve_ivp(pedal_dynamics, t_span, y0, t_eval=t_eval)
abserr = 1.e-8
relerr = 1.e-8
solution = solve_ivp(pedal_dynamics, t_span, y0, t_eval=t_eval,
                     rtol=relerr, atol=abserr)#,
                     #events=[hit_x_max, hit_x_min])

# Ergebnisse extrahieren
t_res = solution.t
x_res = solution.y[0]
v_res = solution.y[1]

# obtain derivatives
a_res = np.zeros(np.size(t_res))
x_target = np.zeros(np.size(t_res))
for idx in range(np.size(t_res)):
    tmp = pedal_dynamics(solution.t[idx], solution.y[:,idx])
    a_res[idx] = tmp[1]
    x_target[idx] = user_position(solution.t[idx])



#F_user_vals = np.array([user_force(ti) for ti in t_res])
#F_target_vals, F_target_gradient = F_target(x_res)

# Plot
plt.figure(figsize=(12, 8))

# position based plots
plt.subplot(3, 2, 1)
plt.plot(t_res, x_res * 1000, label='pedal travel')
plt.plot(t_res, x_target * 1000, label='user induced travel')
plt.ylabel('pedal travel in mm')
plt.grid()
plt.xlabel('time in sec')
plt.legend()

plt.subplot(3, 2, 3)
plt.plot(t_res, v_res * 1000)
plt.ylabel('pedal velocity in mm/s')
plt.grid()
plt.xlabel('time in sec')

plt.subplot(3, 2, 5)
plt.plot(t_res, a_res * 1000)
plt.ylabel('pedal acceleratrion in mm/s^2')
plt.grid()
plt.xlabel('time in sec')




# force plots
plt.subplot(3, 2, 2)
x_eval = np.linspace(0, x_max, 1000)
F_force_travel, F_target_gradient = F_target(x_eval)
plt.plot(x_eval * 1000, F_force_travel, label='F_curve')
plt.ylabel('force in N')
plt.legend()
plt.grid()
plt.xlabel('travel in mm')

#plt.subplot(3, 2, 4)
#plt.plot(t_res, F_user_vals, label='F_user')
#plt.plot(t_res, F_target_vals, '--', label='F_target(x)')
#plt.plot(t_res, F_servo_vals, label='F_servo')
#plt.ylabel('Kraft [N]')
#plt.legend()
#plt.grid()
#plt.xlabel('Zeit [s]')

#plt.subplot(3, 2, 6)
#plt.plot(t_res, F_error)
#plt.ylabel('Regelabweichung [N]')
#plt.xlabel('Zeit [s]')
#plt.grid()

plt.tight_layout()
plt.show()
