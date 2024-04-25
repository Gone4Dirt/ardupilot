# Plot out the sensitivities of all of the inputs to the flare height calculation

import numpy as np
import math
from matplotlib import pyplot as plt

SSL_AIR_DENSITY = 1.225
ASSUMED_CD0 = 0.011
AP_ALPHA_TPP = 20.0
GRAVITY_MSS = 9.81

def norm(a,b):
    return np.sqrt(a**2 + b**2)

def deg2rad(a):
    return a*np.pi/180

def safe_asin(f):
    if f >= 1.0:
        return 2 * math.pi
    if f <= -1.0:
        return -2 * math.pi
    return math.asin(f)

def safe_acos(f):
    if f >= 1.0:
        return 0
    if f <= -1.0:
        return -math.pi
    return math.acos(f)

def my_arctanh(x):
    # print(x)
    try:
        # return (math.exp(2*x) + 1) / (math.exp(2*x) - 1)
        return 0.5 * math.log((1 + x)/(1 - x))
    except:
        print('ArcTanException')
        return 0.0

# =====================================================================================
class Flare_Calc_Reworked_Math():
    def __init__(self):
        self.reset_to_defaults()

    def reset_to_defaults(self):
        self.var = {}
        self.var['_param_solidity'] = 0.05
        self.var['_c_l_alpha'] = np.pi*2
        self.var['_param_diameter'] = 1.25
        self.var['blade_pitch_hover_deg'] = 5.0
        self.var['_param_head_speed_set_point'] = 1500
        self.var['_param_target_speed'] = 1100
        self.var['_t_tch'] = 0.55

    def initial_flare_estimate(self):

        blade_pitch_hover_rad = deg2rad(self.var['blade_pitch_hover_deg'])
        blade_pitch_hover_rad = max(blade_pitch_hover_rad, deg2rad(0.1))

        b = self.var['_param_solidity'] * self.var['_c_l_alpha']
        _disc_area = np.pi * 0.25 * self.var['_param_diameter']**2

        # Calculating the equivalent inflow ratio (average across the whole blade)
        lambda_eq = -b / 16.0 + math.sqrt((b**2) / 256.0 + b * blade_pitch_hover_rad / 12.0) # <--------

        # Tip speed = head speed (rpm) / 60 * 2pi * rotor diameter/2. Eq below is simplified.
        tip_speed_auto = self.var['_param_head_speed_set_point'] * np.pi * self.var['_param_diameter'] / 60.0

        # Calc the coefficient of thrust in the hover
        c_t_hover = 0.5 * b * (blade_pitch_hover_rad / 3.0 - lambda_eq / 2.0)
        self._lift_hover = c_t_hover * SSL_AIR_DENSITY * _disc_area * tip_speed_auto**2

        # Estimate rate of descent
        _est_rod = ((0.25 * self.var['_param_solidity'] * ASSUMED_CD0 / c_t_hover) + (c_t_hover**2 / (self.var['_param_solidity'] * ASSUMED_CD0))) * tip_speed_auto

        # Estimate rotor C_d
        # self._c = self._lift_hover / (_est_rod**2)
        self._c = self._lift_hover / (_est_rod**2 * 0.5 * SSL_AIR_DENSITY * _disc_area)

        # Calc flare altitude
        des_spd_fwd = self.var['_param_target_speed'] * 0.01

        flare_alt, delta_t_flare = self.calc_flare_alt(-_est_rod, des_spd_fwd)
        return (flare_alt, self._lift_hover/GRAVITY_MSS, delta_t_flare)

    def calc_flare_alt(self, sink_rate, fwd_speed):

        # Compute speed module and glide path angle during descent
        speed_module = max(norm(sink_rate, fwd_speed), 0.1)
        glide_angle = np.pi / 2 - safe_asin(fwd_speed / speed_module)

        # Estimate inflow velocity at beginning of flare
        entry_inflow = - sink_rate

        z_m = self._lift_hover / GRAVITY_MSS
        print(z_m)

        delta_h = 10


        # Estimate altitude to begin collective pull
        _cushion_alt = (-sink_rate * self.var['_t_tch']) * 100.0

        # Total delta altitude to ground
        _flare_alt_calc = _cushion_alt + delta_h * 100.0

        return _flare_alt_calc*1e-2, delta_t_flare
# =====================================================================================

n_col = 3
n_row = 2
fig_flare_alt, ax_fa = plt.subplots(n_row, n_col, figsize=(15,8))
fig_mass_est, ax_me = plt.subplots(n_row, n_col, figsize=(15,8))
fig_extra_var, ax_ev = plt.subplots(n_row, n_col, figsize=(15,8))
plot_index = 0
def plot_single_var_sensitivity(obj, var_name, var_array):
    global plot_index, n_col, ax_fa, ax_me, ax_ev
    flare_alt = []
    mass_est = []
    other_var = []
    for v in var_array:
        obj.var[var_name] = v
        alt, mass, delta_t_f = obj.initial_flare_estimate()
        flare_alt.append(alt)
        mass_est.append(mass)
        other_var.append(delta_t_f)

    r = math.floor(plot_index / n_col)
    c = plot_index % n_col

    # Plot against flare alt
    ax_fa[r,c].plot(var_array, flare_alt)
    ax_fa[r,c].set_xlabel(var_name)
    ax_fa[r,c].set_ylabel('Flare Alt (m)')
    plt.tight_layout()

    # # Plot against mass estimate
    ax_me[r,c].plot(var_array, mass_est)
    ax_me[r,c].set_xlabel(var_name)
    ax_me[r,c].set_ylabel('Mass Est (kg)')
    plt.tight_layout()

     # Plot against ...
    ax_ev[r,c].plot(var_array, other_var)
    ax_ev[r,c].set_xlabel(var_name)
    ax_ev[r,c].set_ylabel('Flare Time ()')
    # ax_ev[r,c].set_ylabel('Other Var ()')
    plt.tight_layout()

    plot_index+=1

    obj.reset_to_defaults()



if __name__=='__main__':
    # Setup flare object to do calculations
    flare_obj = Flare_Calc_Reworked_Math()

    N_PTS = 100000

    solidity = np.linspace(0.03, 0.10, N_PTS)
    c_l_alpha = np.linspace(np.pi, np.pi*2, N_PTS)
    diameter = np.linspace(0.72, 3.0, N_PTS)
    blade_pitch_hover_deg = np.linspace(0.1, 10.0, N_PTS)
    head_speed = np.linspace(600, 1500, N_PTS)
    target_speed = np.linspace(800, 1600, N_PTS)

    plot_single_var_sensitivity(flare_obj, '_param_solidity', solidity)
    plot_single_var_sensitivity(flare_obj, '_c_l_alpha', c_l_alpha)
    plot_single_var_sensitivity(flare_obj, '_param_diameter', diameter)
    plot_single_var_sensitivity(flare_obj, 'blade_pitch_hover_deg', blade_pitch_hover_deg)
    plot_single_var_sensitivity(flare_obj, '_param_head_speed_set_point', head_speed)
    plot_single_var_sensitivity(flare_obj, '_param_target_speed', target_speed)

    plt.show()
