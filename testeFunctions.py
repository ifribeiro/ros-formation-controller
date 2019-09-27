# -*- coding: utf-8 -*-
from __future__ import unicode_literals
import math
import numpy as np
import random as rd 
import matplotlib.pyplot as plt

t = [0,1,2,3,4,5,6,7,8,9,10]
erros_x = []
errorhof = []
erros_y = []
erros_z = []
erros_rho = []
erros_alpha = []
erros_beta = []
for i in range(0,10):
    erros_x.append(rd.randint(0,55))
    erros_y.append(rd.randint(0,55))
    erros_z.append(rd.randint(0,55))
    erros_rho.append(rd.randint(0,55))
    erros_alpha.append(rd.randint(0,55))
    erros_beta.append(rd.randint(0,55))
    errorhof.append(rd.randint(0,55))


fig, ax = plt.subplots(2)
fig.set_figheight(20)
fig.set_figwidth(20)
ax[0].plot(t[:-1], erros_x[:], label="Erro X", )        
"""ax[0].plot(t[:i], erroy[:], label="Erro Y")        
ax[0].plot(t[:i], erroz[:], label="Erro Z")"""
ax[0].set_title("Erros de posicionamento")

ax[1].plot(t[:-1], errorhof[:], '--', label="Erro rhof")        
"""ax[1].plot(t[:i], errobetaf[:], '--', label="Erro betaf")        
ax[1].plot(t[:i], erroalphaf[:], '--', label="Erro alphaf")"""
ax[1].set_title("Erros de formação")
ax[1].set_xlabel("Tempo de execução (s)")
ax[1].set_ylabel("Erro (m)")

fig_x = plt.figure(1)
fig_y = plt.figure(2)
fig_z = plt.figure(3)

fig_rho = plt.figure(4)
fig_alpha = plt.figure(5)
fig_beta = plt.figure(6)

ax_x = fig_x.add_subplot(111)
ax_x.plot(t[:-1], erros_x[:], label="Erro X", color="red")
ax_x.set_title("Erros de posicionamento")
ax_x.set_xlabel("Tempo de execução (s)")
ax_x.set_xlabel("Tempo de execução (s)")
ax_x.set_ylabel("Erro (m)")

ax_y = fig_y.add_subplot(111)
ax_y.plot(t[:-1], erros_y[:], color="orange")
ax_y.set_title("Erros de posicionamento y")
ax_y.set_xlabel("Tempo de execução (s)")
ax_y.set_ylabel("Erro (m)")


ax_z = fig_z.add_subplot(111)
ax_z.plot(t[:-1], erros_z[:], color="green")
ax_z.set_title("Erros de posicionamento z")
ax_z.set_xlabel("Tempo de execução (s)")
ax_z.set_ylabel("Erro (m)")

ax_rho = fig_rho.add_subplot(111)
ax_rho.plot(t[:-1], erros_rho[:], color="magenta")
ax_rho.set_title("Erros de posicionamento rho")
ax_rho.set_xlabel("Tempo de execução (s)")
ax_rho.set_ylabel("Erro (m)")

ax_alpha = fig_alpha.add_subplot(111)
ax_alpha.plot(t[:-1], erros_alpha[:], color="sandybrown")
ax_alpha.set_title("Erros de posicionamento alpha")
ax_alpha.set_xlabel("Tempo de execução (s)")
ax_alpha.set_ylabel("Erro (m)")

ax_beta = fig_beta.add_subplot(111)
ax_beta.plot(t[:-1], erros_beta[:], color="cyan")
ax_beta.set_title("Erros de posicionamento beta")
ax_beta.set_xlabel("Tempo de execução (s)")
ax_beta.set_ylabel("Erro (m)")

print (np.std(erros_beta))

fig.legend()

plt.show()