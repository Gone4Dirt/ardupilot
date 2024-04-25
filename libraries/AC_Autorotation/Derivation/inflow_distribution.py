# A simple script that generates plots that illustrate the distribution of inflow ratio with respect 
# to the non-dimensional radial distance.

import numpy as np
from matplotlib import pyplot as plt
import random

GRID_COLOUR = [0.9,0.9,0.9]
N_PTS = 400 # Number of points used in plotting

c_l_alpha = 2*np.pi
solidity = 0.05
hover_blade_pitch_deg = 3


b = solidity * c_l_alpha
hover_blade_pitch_rad = hover_blade_pitch_deg * np.pi / 180.0


r = np.linspace(0, 1, N_PTS)

inflow_ratio_plus = (- b / 16.0) + np.sqrt((b**2 / 256.0) + (b*hover_blade_pitch_rad*r/8.0))
inflow_ratio_minus = (- b / 16.0) - np.sqrt((b**2 / 256.0) + (b*hover_blade_pitch_rad*r/8.0))

# Generate plot of positive and negative roots
fig, ax = plt.subplots(1,1, figsize=(10,4))
plt.grid(True, which='major', axis='both', color=GRID_COLOUR, linestyle='-', linewidth=1)
ax.plot(r, inflow_ratio_plus, label='+ root')
ax.plot(r, inflow_ratio_minus, label='- root', linestyle='--')
ax.set_xlabel('Nondimensional Radial Position, r')
ax.set_ylabel(r'Local inflow ratio, $\lambda_{i}$')
plt.legend()



# show the change in inflow with respect to cl_alpha
c_l_alpha = np.linspace(np.pi, 2*np.pi, N_PTS)
r = np.linspace(0, 1, 5)

head_speed = 1400 # (RPM)

head_speed_rad = head_speed / 60.0 * 2 * np.pi

# Plot the variation of inflow ratio at different stations of r
# Also, plot the sensitivity of the thrust coefficient to changes in lift curve slope

# Config the inflow plot
fig_lam, ax_lam = plt.subplots(1,1, figsize=(10,4))
plt.grid(True, which='major', axis='both', color=GRID_COLOUR, linestyle='-', linewidth=1)
ax_lam.set_xlabel(r'Lift Curve Slope, $C_{l_{\alpha}}$')
ax_lam.set_ylabel(r'Local inflow ratio, $\lambda_{i}$')

# Config the Ct plot
fig_ct, ax_ct = plt.subplots(1,1, figsize=(10,4))
plt.grid(True, which='major', axis='both', color=GRID_COLOUR, linestyle='-', linewidth=1)
ax_ct.set_xlabel(r'Lift Curve Slope, $C_{l_{\alpha}}$')
ax_ct.set_ylabel(r'Local Thrust Coefficient, $C_{T}(r)$')

for ri in r:
    b = solidity * c_l_alpha
    inflow_ratio = (- b / 16.0) + np.sqrt((b**2 / 256.0) + (b*hover_blade_pitch_rad*ri/8.0))


    plot_colour = [random.random(), random.random(), random.random()]
    ax_lam.plot(c_l_alpha, inflow_ratio, label='r = %.2f' % ri, color=plot_colour)

    # determine best fit line
    par = np.polyfit(c_l_alpha, inflow_ratio, 1, full=True)
    # plot linear regression
    slope=par[0][0]
    intercept=par[0][1]
    xl = [min(c_l_alpha), max(c_l_alpha)]
    yl = [slope*xx + intercept  for xx in xl]
    ax_lam.plot(xl, yl, label='r = %.2f Linear' % ri, linestyle='--', color=plot_colour)

    # Do all thrust coefficient calcs and plotting
    c_t_hover = 0.5 * b * (hover_blade_pitch_rad / 3.0 + inflow_ratio / 2.0)
    ax_ct.plot(c_l_alpha, c_t_hover, label='r = %.2f' % ri, color=plot_colour)


plt.legend()




# Show the change in inflow with respect to blade pitch collective
c_l_alpha = 2*np.pi
r = np.linspace(0, 1, 5)
hover_blade_pitch_deg = np.linspace(0, 12, N_PTS)
hover_blade_pitch_rad = hover_blade_pitch_deg * np.pi / 180.0
b = solidity * c_l_alpha

# plot the variation of inflow ratio at different stations of r
fig, ax = plt.subplots(1,1, figsize=(10,4))
plt.grid(True, which='major', axis='both', color=GRID_COLOUR, linestyle='-', linewidth=1)
for ri in r:
    inflow_ratio = (- b / 16.0) + np.sqrt((b**2 / 256.0) + (b*hover_blade_pitch_rad*ri/8.0))
    ax.plot(hover_blade_pitch_deg, inflow_ratio, label='r = %.2f' % ri)

    # determine best fit line
    par = np.polyfit(hover_blade_pitch_deg, inflow_ratio, 1, full=True)
    # plot linear regression
    slope=par[0][0]
    intercept=par[0][1]
    xl = [min(hover_blade_pitch_deg), max(hover_blade_pitch_deg)]
    yl = [slope*xx + intercept  for xx in xl]
    ax.plot(xl, yl, label='r = %.2f Linear' % ri, linestyle='--')

ax.set_xlabel(r'Blade Pitch Angle, $\theta$')
ax.set_ylabel(r'Local inflow ratio, $\lambda_{i}$')
plt.legend()












plt.show()
