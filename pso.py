import numpy as np
import time,math,random
import itertools
import matplotlib.pyplot as plt
from parameters import *

MU = 1
# lambda_prob = 0.005

def PSO(tasks, fitness_list, N = 100, T = 50 , c1 = 1.5, c2 = 1.5,
        w_max = 0.8 ,w_min = 0.8, v_max = 10, v_min = -10, h_min = 0.1):
    
    # print('fitness_list')
    # print(fitness_list)
    # start_time = time.time()
    def fitness_func(particle , time):
        # adaptive mu
        mu_temp = MU  * (T-time)/T

        for tkid in range(tasks_num):
            if np.sum(particle[ tkid*vehicles_num : (tkid+1)*vehicles_num ]) == 0:
                value = 100000000000000000
                delete_flag = True
                return  [value, delete_flag, None]
            
        tasks_time = np.zeros(tasks_num)
        
        prob = []
        # list of vehicle, store tasks number when multi-tasks are allocated to one vehicle
        particle_count = [0] * vehicles_num
        
        for vid in range(vehicles_num):
            # calculate repeating tasks allocation
            for tkid in range(tasks_num):
                particle_count[vid] += particle[ tkid*vehicles_num + vid ]
                # np.sum(particle[ tkid*vehicles_num : (tkid+1)*vehicles_num ])
        pass
    
        value_s = []
        # for some tasks allocation combination
        # they may not satisify the latency probability restriction
        # thus delete and renew this combination
        delete_flag = False
        for tkid in range(tasks_num):
            # calculate repeating tasks allocation
            # particle_count = np.sum(particle[ tkid*vehicles_num : (tkid+1)*vehicles_num ])
            prob_temp = 1
            for vid in range(vehicles_num):
                if particle[ tkid*vehicles_num + vid ]==1:
                    temp_task_time = 0
                    temp_task_time += tasks[tkid].remain_size_dl/fitness_list[vid][0]\
                        *(fitness_list[vid][3] + particle_count[vid])
                    
                    temp_task_time += tasks[tkid].remain_size_ul/fitness_list[vid][1]\
                        *(fitness_list[vid][4] + particle_count[vid])
                    
                    temp_task_time += tasks[tkid].remain_compute_size/fitness_list[vid][2]\
                        *(fitness_list[vid][5] + particle_count[vid])

                    # prob[tkid] = 1 - edge_vehicles[vid].prob(tasks_time[tkid]) # fail prob
                    # prob[tkid] = 1 - math.exp(- temp_task_time* lambda_prob)
                    prob_temp *= (1 - math.exp(- temp_task_time* lambda_prob)) # np.log(1 + h_min - prob[tkid])
                    tasks_time[tkid] += temp_task_time
            # fitness value with barrier function\
            prob.append(prob_temp)
            barrier = mu_temp * np.log(1 + h_min - prob_temp)
            value_s.append(tasks_time[tkid] - barrier) # fitness value
            value = np.sum(value_s)
            if h_min < prob_temp:
                delete_flag = True
        # print('fitness func time:', time.time()-fitness_func_start_time)
        ave_prob = np.sum(prob)/len(prob)
        return  [value, delete_flag, ave_prob]

    def initialize( pre_best_x = None):
                # PSO main part
        # random sample to initialize partocles
        # x = np.random.randint(0, 2, [N, D ]) # 0-20 represent vehicle_id
        x=[]
        # comb = math.factorial(vehicles_num)//(math.factorial(vehicles_num-2)*math.factorial(2))
        comb = 0
        for i in range(vehicles_num+comb):
            particle_temp = []
            index_list = list(range(vehicles_num+comb))
            
            for j in range(tasks_num):
                # sample_allcation = allocation_space[np.random.randint(vehicles_num)]
                if j != 0:
                    index = random.sample(index_list, 1)[0]
                else:
                    index = i
                index_list.remove(index)
                particle_temp.extend(allocation_space[index])
            x.append(particle_temp)
            
        for _ in range(N-vehicles_num):
            particle_temp = []
            for _ in range(tasks_num):
                # sample_allcation = allocation_space[np.random.randint(vehicles_num)]
                particle_temp.extend(allocation_space[np.random.randint(len(allocation_space))])
            x.append(particle_temp)
        x = np.array(x)

        # 初始化每个粒子的适应度值
        p = x  # 用来存储每个粒子的最佳位置
        p_best = np.ones(N)  # 用来存储每个粒子的适应度值
        
        min_fail_prob = 1
        best_x = []
            
        for i in range(N):
            fitness_value, delete_flag, fail_prob = fitness_func(x[i, :], 0)
            min_fail_prob = min(min_fail_prob, fail_prob)
            if min_fail_prob == fail_prob:  best_x = x[i, :]
            # renew particle which can not satisify the latency probability restriction
            if delete_flag or np.sum(x[i,:])>=tasks_num * max_reapted_num:
                for _ in range(50): # renew maximization times
                    particle_temp =[]
                    for _ in range(tasks_num):
                        particle_temp.extend(allocation_space[np.random.randint(len(allocation_space))])
                    x[i,:] = particle_temp
                    fitness_value, delete_flag, fail_prob = fitness_func(x[j,:], 0)
                    min_fail_prob = min(min_fail_prob, fail_prob)
                    if min_fail_prob == fail_prob:  best_x = x[i, :]
                    if not delete_flag: # once the new particle fulfill restriction
                        break
            if delete_flag:
                p_best[i] = 100000000000000000
                if pre_best_x:
                    x[i,:] = pre_best_x
                    fitness_value, delete_flag, fail_prob = fitness_func(x[i, :], 0)
                    min_fail_prob = min(min_fail_prob, fail_prob)
                    best_x = x[i, :]
            else:
                p_best[i] = fitness_value
        return x , p_best, p, min_fail_prob, best_x
    # c1 = 1.5
    # c2 = 1.5
    # w_max = 0.8
    # w_min = 0.8
    # v_max = 10
    # v_min = -10
    
    vehicles_num = len(fitness_list) # 20 by default
    tasks_num = len(tasks)
    D = tasks_num * vehicles_num
    # build allocation space
    max_reapted_num = 3
    max_reapted_num += 1
    allocation_space = []
    for allocated_v_num in range(1,max_reapted_num):
        sample_list = itertools.combinations(list(range(vehicles_num)),allocated_v_num)
        for sample_item in sample_list:
            particle = [0] * vehicles_num
            for j in sample_item:
                particle[j] = 1
            allocation_space.append(particle)  
    
    v = (v_max - v_min) * np.random.rand(N, D ) + v_min
    vx = np.random.rand(N,D)
    x , p_best, p, min_fail_prob, best_x = initialize()
    
    if np.sum(p_best) == 1e17*N: # all elements are 100000000000000000, means no solution
        h_min = min_fail_prob + 0.05
        print('adjust h min ',h_min)
        x , p_best, p , min_fail_prob, best_x = initialize()
        print('best x',best_x)
        # print(p_best)
        
    
    # 初始化全局最优位置与最优值
    g_best = 100000000000000000
    x_best = np.ones(D)
    for i in range(N):
        if p_best[i] < g_best:
            g_best = p_best[i]
            x_best = x[i, :].copy()

    gb = np.ones(T)  # global best
    min_fail_prob = 1
    for i in range(T):
        for j in range(N):
            # 更新每个个体最优值和最优位置
            fitness_value, delete_flag, fail_prob = fitness_func(x[j,:], i)
            try:
                min_fail_prob = min(min_fail_prob, fail_prob)
            except:
                continue
            # renew particle which can not satisify the latency probability restriction
            if delete_flag or np.sum(x[j,:]) >= tasks_num*max_reapted_num:
                for k in range(10): # renew maximization times
                    particle_temp =[]
                    for _ in range(tasks_num):
                        particle_temp.extend(allocation_space[np.random.randint(len(allocation_space))])
                    x[j,:] = particle_temp
                    fitness_value, delete_flag, fail_prob = fitness_func(x[j,:], i)
                    if not delete_flag: # once the new particle fulfill restriction
                        break
            if delete_flag:
                continue
            
            if p_best[j] > fitness_value:
                p_best[j] = fitness_value
                p[j, :] = x[j, :].copy()
            # 更新全局最优位置和最优值
            if p_best[j] < g_best:
                g_best = p_best[j]
                x_best = x[j, :].copy()
            # 计算动态惯性权重
            w = w_max - (w_max - w_min) * i / T
            # 更新速度, 因为位置需要后面进行概率判断更新
            v[j, :] = w * v[j, :] + c1 * np.random.rand(1) * (p_best[j] - x[j, :])\
                        + c2 * np.random.rand(1) * (x_best - x[j, :])
            
            # 边界条件处理
            for jj in range(D):
                if (v[j, jj] > v_max) or (v[j, jj] < v_min):
                    v[j, jj] = v_min + np.random.rand(1)[0] * (v_max - v_min)
                    
            # 进行概率计算并且更新位置
            vx[j, :] = 1 / (1 + np.exp(-v[j, :]))
            
            for ii in range(D):
                r = np.random.rand(1)
                x[j, ii] = 1 if vx[j, ii] > r else 0
        gb[i] = g_best
    
    allocation =[]
    for i in range(tasks_num):
        x_temp = x_best[i*vehicles_num:(i+1)*vehicles_num]
        vehicle_index = [x+1 for x, y in list(enumerate(x_temp)) if y == 1]
        allocation.append(vehicle_index)
        
    # debug
    
    fitness_value, delete_flag, fail_prob = fitness_func(x_best, T-1)
    # print(allocation, delete_flag, g_best, fail_prob)
    return allocation, delete_flag, fail_prob, gb