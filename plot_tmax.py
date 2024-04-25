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
ranges = list(range(10, 60+1,10))

def get_max_a(filename):
    result = pickle.load(open(filename, 'rb'))
    all_a = result['accuracy']
    return np.max(all_a)

results_lifi, results_baseline, results_bo = [],[], []

for ran in ranges:
    common = 'results/results_'+'UE_'+str(numClients)+'budget_'+str(budget)+'Tmax_'+str(ran)
    results_lifi.append(get_max_a(common+'_lifi.pkl'))
    results_baseline.append(get_max_a(common + '_baseline.pkl'))
    results_bo.append(get_max_a(common+'_bo.pkl'))




plt.figure(figsize=fig_size) 
plt.plot(ranges, results_baseline,linewidth=linesize, marker = 'x',label="Baseline")
plt.plot(ranges, results_lifi,linewidth=linesize, marker = 'o',label="LiFi")
plt.plot(ranges, results_bo,linewidth=linesize, marker = 's',label="BARA")
plt.grid(which='both',linestyle='--',axis='both',color='gray')
plt.xlabel('Latency Requirement (Seconds)',fontsize=font_size)
plt.ylabel('Accuracy',fontsize=font_size)
plt.legend(prop=dict(size=font_size),loc='lower right')
# plt.xticks(ranges,fontsize=font_size)
# plt.yticks([0.155,0.16,0.165,0.17,0.175],fontsize=font_size)
plt.tight_layout()
plt.subplots_adjust(left=0.16, bottom=0.18, right=1-0.07, top=1-0.03)
plt.savefig('results/fig_tmax.pdf',format='pdf',dpi=300)
plt.show()