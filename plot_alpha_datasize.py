import numpy as np
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, ConstantKernel as C
from sklearn.gaussian_process.kernels import DotProduct, WhiteKernel
from sklearn.preprocessing import StandardScaler
import matplotlib.pyplot as plt
import matplotlib
import datetime
import math
from tqdm import tqdm
from scipy.optimize import minimize
import pickle

font_size = 13
fig_size = (5,3.5)
linesize = 2
matplotlib.rcParams['pdf.fonttype'] = 42

numClients = 10
budget = 5
rounds = 100

name = 'results/results_'+'UE_'+str(numClients)+'budget_'+str(budget)+'_lifi.pkl'

results = pickle.load(open(name, 'rb'))

alpha_RES_N=results["alpha_RES_N"]
D_RES_N=results["D_RES_N"]

####### alpha ########
plt.figure(figsize=fig_size) 
for i in range(numClients):
    plt.plot(alpha_RES_N[:, i],"-" ,linewidth=linesize)

plt.grid(which='both',linestyle='-',axis='both',color='gray')
plt.xlabel('Iterations',fontsize=font_size)
plt.ylabel('Reward Weight',fontsize=font_size)
plt.xticks(list(range(0,rounds+1,int(rounds/5))),fontsize=font_size)
plt.yticks(fontsize=font_size)
plt.tight_layout()
plt.subplots_adjust(left=0.16, bottom=0.18, right=1-0.07, top=1-0.03)
plt.savefig('results/fig_aplha.pdf',format='pdf',dpi=300)
plt.show()


####### data size ########

plt.figure(figsize=fig_size) 
for i in range(numClients):
    plt.plot(D_RES_N[:, i],"-" ,linewidth=linesize)

plt.grid(which='both',linestyle='--',axis='both',color='gray')
plt.xlabel('Iteractions',fontsize=font_size)
plt.ylabel('Dataset size(k)',fontsize=font_size)
plt.xticks(list(range(0,rounds+1,int(rounds/5))),fontsize=font_size)
plt.yticks(fontsize=font_size)
plt.subplots_adjust(left=0.16, bottom=0.18, right=1-0.07, top=1-0.03)
plt.savefig('results/fig_datasize.pdf',format='pdf',dpi=300)
plt.show()