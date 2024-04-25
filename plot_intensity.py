import pickle
import numpy as numpy
import matplotlib.pyplot  as plt
import numpy as np

font_size = 14
fig_size = (4,3)
line_size = 2.0


h_min_list = [0.1, 0.2, 0.3, 0.4]

latency_list = [182.76470588235293, 185.5686274509804, 178.27450980392157, 176.80392156862746, 176.0]
fail_prob_list = [0.14757494459622872, 0.17415357797176304, 0.17236041030471408, 0.26350413305310005, 0.39805223876758683]

fig, ax = plt.subplots(figsize=fig_size)

ax.plot(h_min_list, latency_list[1:], label='Latency', color='C0',marker='o', linewidth=line_size)
ax.set_ylabel('Avg. Latency (ms)', color='C0')
ax.legend(prop=dict(size=font_size-2),loc='upper left')

ax2 = ax.twinx()
ax2.plot(h_min_list, fail_prob_list[1:], label='Failure probability', color='C1',marker='*', linewidth=line_size)
ax2.set_ylabel('Failure probability', color='C1')
ax2.legend(prop=dict(size=font_size-2),loc='lower right')

ax.set_xlabel('Failure threshold',fontsize=font_size-2)
plt.grid(which='both',linestyle='--',axis='y',color='gray') # zorder=0, 
plt.tight_layout()
# plt.subplots_adjust(left=0.19, bottom=0.17, right=0.98, top=0.98) # adjust when plt.show() and copy to here

# plt.show()
plt.savefig("results_/threshold.pdf", format = 'pdf', dpi=300)
print('done')


###############################################################

latency = {'pso': [182.76470588235293, 180.9625, 188.44210526315788, 186.5344827586207, 200.30935251798562], 'fpsomr': [306.11764705882354, 309.8333333333333, 317.80645161290323, 302.1858407079646, 303.6222222222222], 'least': [273.7647058823529, 328.32, 300.2043010752688, 298.9298245614035, 336.1119402985075]}


fig, ax = plt.subplots(figsize=fig_size)

ranges = [10, 15, 20, 25, 30]
ax.plot(ranges,latency['pso'], marker='o',label='CAVE')
ax.plot(ranges,latency['fpsomr'], marker='*',label='FPSO-MR')
ax.plot(ranges,latency['least'], marker='>',label='Baseline')
ax.set_ylabel('Avg. latency (ms)', fontsize=font_size-2)
# ax2.bar(ranges-0.1, usage, align='center', color='C1', width=0.2, alpha=0.8)
# ax2.set_ylim(0.5, 2.0)

# ax.set_axisbelow(True)
ax.set_xlabel('Traffic intensity (tasks/s)',fontsize=font_size-2)
# plt.xticks(ranges, labels)
plt.legend(prop=dict(size=font_size-2),loc='upper left')

plt.grid(which='both',linestyle='--',axis='y',color='gray') # zorder=0, 
plt.tight_layout()
plt.subplots_adjust(left=0.16, bottom=0.17, right=0.98, top=0.98) # adjust when plt.show() and copy to here

# plt.show()
plt.savefig("results_/intensity_latency.pdf", format = 'pdf', dpi=300)
print('done')


###############################################################

fail = {'pso': [0.14757494459622872, 0.14515140325535247, 0.1601896919061106, 0.16090675334066626, 0.19162619867433908], 'fpsomr': [0.5270811941032356, 0.47581799503289307, 0.49395610244104765, 0.47559191751539004, 0.4473939592782638], 'least': [0.5247002344473123, 0.5408543042989344, 0.5537897633237135, 0.5551696664293633, 0.5791179956020467]}

fig, ax = plt.subplots(figsize=fig_size)

ranges = [10, 15, 20, 25, 30]
ax.plot(ranges,fail['pso'], marker='o',label='CAVE')
ax.plot(ranges,fail['fpsomr'], marker='*',label='FPSO-MR')
ax.plot(ranges,fail['least'], marker='>',label='Baseline')
ax.set_ylabel('Failure probability', fontsize=font_size-2)
# ax2.bar(ranges-0.1, usage, align='center', color='C1', width=0.2, alpha=0.8)
# ax2.set_ylim(0.5, 2.0)

# ax.set_axisbelow(True)
ax.set_xlabel('Traffic intensity (tasks/s)',fontsize=font_size-2)
# plt.xticks(ranges, labels)
plt.legend(prop=dict(size=font_size-2),loc='upper left')

plt.grid(which='both',linestyle='--',axis='y',color='gray') # zorder=0, 
plt.tight_layout()
plt.subplots_adjust(left=0.19, bottom=0.17, right=0.98, top=0.98) # adjust when plt.show() and copy to here

# plt.show()
plt.savefig("results_/intensity_failure.pdf", format = 'pdf', dpi=300)
print('done')


