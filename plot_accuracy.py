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

common = 'results/results_'+'UE_'+str(numClients)+'budget_'+str(budget)

def get_all_a(filename):
    result = pickle.load(open(filename, 'rb'))
    all_a = result['accuracy']
    return all_a

results_lifi = get_all_a(common+'_lifi.pkl')
results_baseline = get_all_a(common + '_baseline.pkl')
results_bo = get_all_a(common+'_bo.pkl')

def get_best(inputs):
    best=np.empty(len(inputs))
    temp=-1
    for i in range(len(inputs)):
        if inputs[i]>temp:
            best[i]= inputs[i]
            temp = inputs[i]
        else:
            best[i] = temp
    return best

# results_lifi = get_best(results_lifi)
# results_baseline = get_best(results_baseline)
# results_bo = get_best(results_bo)

plt.figure(figsize=fig_size) 
plt.plot(results_baseline,linewidth=linesize,label="Baseline")
plt.plot(results_lifi,linewidth=linesize,label="LiFi")
plt.plot(results_bo,linewidth=linesize,label="BARA")
plt.grid(which='both',linestyle='--',axis='both',color='gray')
plt.xlabel('Iterations',fontsize=font_size)
plt.ylabel('Accuracy',fontsize=font_size)
plt.legend(prop=dict(size=font_size),loc='lower right')
# plt.xticks([0,20,40,60,80,100],fontsize=font_size)
# plt.yticks([0.155,0.16,0.165,0.17,0.175],fontsize=font_size)
plt.tight_layout()
plt.subplots_adjust(left=0.16, bottom=0.18, right=1-0.07, top=1-0.03)
plt.savefig('results/fig_a.pdf',format='pdf',dpi=300)
plt.show()