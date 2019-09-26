# -*- coding: utf-8 -*-
from __future__ import unicode_literals
import math
import numpy as np
import random as rd 
import matplotlib.pyplot as plt

t = [0,1,2,3,4,5,6,7,8,9,10]
errox = []
errorhof = []
for i in range(0,10):
    errox.append(rd.randint(0,55))
    errorhof.append(rd.randint(0,55))


fig, ax = plt.subplots(2)
fig.set_figheight(20)
fig.set_figwidth(20)
ax[0].plot(t[:-1], errox[:], label="Erro X", )        
"""ax[0].plot(t[:i], erroy[:], label="Erro Y")        
ax[0].plot(t[:i], erroz[:], label="Erro Z")"""
ax[0].set_title("Erros de posicionamento")

ax[1].plot(t[:-1], errorhof[:], '--', label="Erro rhof")        
"""ax[1].plot(t[:i], errobetaf[:], '--', label="Erro betaf")        
ax[1].plot(t[:i], erroalphaf[:], '--', label="Erro alphaf")"""
ax[1].set_title("Erros de formação")
ax[1].set_xlabel("Tempo de execução (s)")
ax[1].set_ylabel("Erro (m)")

fig.legend()

plt.show()