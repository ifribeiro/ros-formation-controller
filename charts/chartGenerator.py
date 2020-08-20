# -*- coding: utf-8 -*-
from __future__ import unicode_literals
import math
import numpy as np
import random as rd 
import matplotlib.pyplot as plt
import pandas as pd

df_erro_sim = pd.read_csv("Erros_simulacao.csv")
df_erro_exp1 = pd.read_csv("Erros_Exp1.csv")
df_erro_exp2 = pd.read_csv("Erros_Exp2.csv")

i = len(df_erro_exp1['X'])
t = np.arange(0,20,0.2)
des = np.zeros(len(t))

fig_x = plt.figure(0)
ax_x = fig_x.add_subplot(111)
ax_x.plot(t[:99], df_erro_exp1['X'][:99], label='Experimento 1')
ax_x.plot(t[:99], df_erro_exp2['X'][:99], label='Experimento 2')
ax_x.plot(t[:99], df_erro_sim['X'][:99], label='Simulação')
ax_x.plot(t[:99], des[:99],'--',color='red', label='Desejado')
ax_x.set_title("Erros de posição: X")
ax_x.set_xlabel("Tempo de execução (s)")
ax_x.set_ylabel("Erro (m)")
plt.legend()

fig_y = plt.figure(1)
ax_y = fig_y.add_subplot(111)
ax_y.plot(t[:99], df_erro_exp1['Y'][:99], label='Experimento 1')
ax_y.plot(t[:99], df_erro_exp2['Y'][:99], label='Experimento 2')
ax_y.plot(t[:99], df_erro_sim['Y'][:99], label='Simulação')
ax_y.plot(t[:99], des[:99],'--', color='red', label='Desejado')
ax_y.set_title("Erros de posição: Y")
ax_y.set_xlabel("Tempo de execução (s)")
ax_y.set_ylabel("Erro (m)")
plt.legend()

fig_z = plt.figure(2)
ax_z = fig_z.add_subplot(111)
ax_z.plot(t[:99], df_erro_exp1['Z'][:99], label='Experimento 1')
ax_z.plot(t[:99], df_erro_exp2['Z'][:99], label='Experimento 2')
ax_z.plot(t[:99], df_erro_sim['Z'][:99], label='Simulação')
ax_z.plot(t[:99], des[:99],'--',color='red', label='Desejado')
ax_z.set_title("Erros de posição: Z")
ax_z.set_xlabel("Tempo de execução (s)")
ax_z.set_ylabel("Erro (m)")
plt.legend()

fig_rho = plt.figure(3)
ax_rho = fig_rho.add_subplot(111)
ax_rho.plot(t[:99], df_erro_exp1['Rho'][:99], label='Experimento 1')
ax_rho.plot(t[:99], df_erro_exp2['Rho'][:99], label='Experimento 2')
ax_rho.plot(t[:99], df_erro_sim['Rho'][:99], label='Simulação')
ax_rho.plot(t[:99], des[:99],'--', color='red', label='Desejado')
ax_rho.set_title("Erros de posição: Rho")
ax_rho.set_xlabel("Tempo de execução (s)")
ax_rho.set_ylabel("Erro (m)")
plt.legend()

fig_alpha = plt.figure(4)
ax_alpha = fig_alpha.add_subplot(111)
ax_alpha.plot(t[:99], df_erro_exp1['Alpha'][:99], label='Experimento 1')
ax_alpha.plot(t[:99], df_erro_exp2['Alpha'][:99], label='Experimento 2')
ax_alpha.plot(t[:99], df_erro_sim['Alpha'][:99], label='Simulação')
ax_alpha.plot(t[:99], des[:99],'--', color='red', label='Desejado')
ax_alpha.set_title("Erros de posição: Alpha")
ax_alpha.set_xlabel("Tempo de execução (s)")
ax_alpha.set_ylabel("Erro (m)")
plt.legend()

fig_beta = plt.figure(5)
ax_beta = fig_beta.add_subplot(111)
ax_beta.plot(t[:99], df_erro_exp1['Beta'][:99], label='Experimento 1')
ax_beta.plot(t[:99], df_erro_exp2['Beta'][:99], label='Experimento 2')
ax_beta.plot(t[:99], df_erro_sim['Beta'][:99], label='Simulação')
ax_beta.plot(t[:99], des[:99],'--', color='red', label='Desejado')
ax_beta.set_title("Erros de posição: Beta")
ax_beta.set_xlabel("Tempo de execução (s)")
ax_beta.set_ylabel("Erro (m)")

plt.legend()
plt.show()