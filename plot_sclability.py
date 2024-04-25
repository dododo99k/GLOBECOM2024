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

rounds = 100
ranges = [10,50,100,150,200]

def get_max_a(filename):
    result = pickle.load(open(filename, 'rb'))
    all_a = result['accuracy']
    return np.max(all_a), np.min(all_a)

results_lifi, results_baseline, results_bo = [],[], []
results_lifi_scale, results_baseline_scale, results_bo_scale = [],[], []
mins = []
for ran in ranges:
    common = 'results/results_'+'UE_'+str(ran)+'budget_'+str(int(ran/2)) # we set budget to be half in simulation
    lifi, _ = get_max_a(common+'_lifi.pkl')
    baseline, min = get_max_a(common + '_baseline.pkl')
    bo, _ = get_max_a(common+'_bo.pkl')

    results_lifi.append(1)
    results_baseline.append(baseline/lifi)
    results_bo.append(bo/lifi)

    mins.append(min)
    improve = lifi-min
    results_lifi_scale.append(1)
    results_baseline_scale.append((baseline-min)/improve)
    results_bo_scale.append((bo-min)/improve)




#########################################################################################################
ind = np.arange(1,6,1)
width = 0.3

fig, ax = plt.subplots(figsize=fig_size)    

ax.bar(ind,results_lifi,linewidth=linesize, width=width, label="LiFi", linestyle='solid', color='C1')

ax.bar(ind+width,results_baseline,linewidth=linesize, width=width, label="Baseline", linestyle='solid', color='C0')

ax.bar(ind+2*width,results_bo,linewidth=linesize, width=width, label="BARA", linestyle='solid', color='C2')

plt.xticks(ind + width / 2, ('10','50','100','150','200'))

plt.legend(prop=dict(size=font_size),loc='lower left')
plt.xlabel('Number of CAVs',fontsize=font_size)
plt.ylabel('Accuracy relative to LiFi',fontsize=font_size)
plt.grid(which='both',linestyle='--',axis='y',color='gray')
plt.ylim(0.95, 1.001)
plt.tight_layout()
plt.subplots_adjust(left=0.16, bottom=0.18, right=0.980, top=0.970) # adjust when plt.show() and copy to here
# plt.subplots_adjust(left=0.16, bottom=0.18, right=1-0.07, top=1-0.03)
plt.savefig('results/fig_scale.pdf',format='pdf',dpi=300)
plt.show()
print('done')






#########################################################################################################
ind = np.arange(1,6,1)
width = 0.3

fig, ax = plt.subplots(figsize=fig_size)    

ax.bar(ind,results_lifi_scale,linewidth=linesize, width=width, label="LiFi", linestyle='solid', color='C1')

ax.bar(ind+width,results_baseline_scale,linewidth=linesize, width=width, label="Baseline", linestyle='solid', color='C0')

ax.bar(ind+2*width,results_bo_scale,linewidth=linesize, width=width, label="BARA", linestyle='solid', color='C2')

plt.xticks(ind + width / 2, ('10','50','100','150','200'))

plt.legend(prop=dict(size=font_size),loc='lower left')
plt.xlabel('Number of CAVs',fontsize=font_size)
plt.ylabel('Relative accuracy improvement',fontsize=font_size)
plt.grid(which='both',linestyle='--',axis='y',color='gray')
plt.tight_layout()
plt.subplots_adjust(left=0.16, bottom=0.18, right=0.980, top=0.970) # adjust when plt.show() and copy to here
# plt.subplots_adjust(left=0.16, bottom=0.18, right=1-0.07, top=1-0.03)
plt.savefig('results/fig_scale_scale.pdf',format='pdf',dpi=300)
plt.show()
print('done')
