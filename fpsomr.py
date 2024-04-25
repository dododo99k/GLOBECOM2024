import numpy as np
import time,math,random
import itertools
import matplotlib.pyplot as plt
from parameters import *

# lambda_prob = 0.005
exterior_penalty_coefficient = 1e4

def FPSOMR(tasks, fitness_list, N = 100, T = 50 , c1 = 1.5, c2 = 1.5,
        w_max = 0.8 ,w_min = 0.8, v_max = 10, v_min = -10, h_min = 0.1):
    
    # start_time = time.time()
    def fitness_func(particle , time):
        # adaptive mu
        
        tasks_time = np.zeros(tasks_num)
        
        prob_list = []
        exterior_penalty = []
        # list of vehicle, store tasks number when multi-tasks are allocated to one vehicle
        particle_count = [0] * vehicles_num
        
        for vid in range(vehicles_num):
            # calculate repeating tasks allocation
            for tkid in range(tasks_num):
                particle_count[vid] += particle[ tkid*vehicles_num + vid ]
                # np.sum(particle[ tkid*vehicles_num : (tkid+1)*vehicles_num ])
        pass
    
        value = 0
        # for some tasks allocation combination
        # they may not satisify the latency probability restriction
        # thus delete and renew this combination
        penalty_flag = False
        for tkid in range(tasks_num):
            if np.sum(particle[ tkid*vehicles_num : (tkid+1)*vehicles_num ])== 0:
                value = 10000000
                penalty_flag = False
                return  [value, penalty_flag, None]
        for tkid in range(tasks_num):
            # calculate repeating tasks allocation
            # particle_count = np.sum(particle[ tkid*vehicles_num : (tkid+1)*vehicles_num ]
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
                        
                    # prob[tkid] = 1 - math.exp(- temp_task_time * lambda_prob)
                    prob_temp *= (1 - math.exp(- temp_task_time * lambda_prob)) # np.log(1 + h_min - prob[tkid])
                    tasks_time[tkid] += temp_task_time
                    pass

            prob_list.append(prob_temp)
            exterior_penalty.append(exterior_penalty_coefficient * (prob_temp - h_min)**2)
            value += tasks_time[tkid] #- mu_temp * np.log(1 + h_min - prob_temp) # fitness value
            if h_min < prob_temp:
                penalty_flag = True
                penalty = np.sum(exterior_penalty)
                pass
                value += penalty
                # value += exterior_penalty
        # print('fitness func time:', time.time()-fitness_func_start_time)
        ave_prob = np.sum(prob_list)/len(prob_list)
        return  [value, penalty_flag , ave_prob]
    
    
    # c1 = 1.5
    # c2 = 1.5
    # w_max = 0.8
    # w_min = 0.8
    # v_max = 10
    # v_min = -10
    
    vehicles_num = len(fitness_list) # 20 by default
    tasks_num = len(tasks)
    D = tasks_num * vehicles_num
    
    # PSO main part
    # random sample to initialize partocles
    allocation_space = []
    for allocated_v_num in range(1,8):
        sample_list = itertools.combinations(list(range(vehicles_num)),allocated_v_num)
        for sample_item in sample_list:
            particle = [0] * vehicles_num
            for j in sample_item:
                particle[j] = 1
            allocation_space.append(particle)

    x=[]
    # for i in range(vehicles_num):
    #     particle_temp = []
    #     index_list = list(range(vehicles_num))
    #     for j in range(tasks_num):
    #         # sample_allcation = allocation_space[np.random.randint(vehicles_num)]
    #         if j != 0:
    #             index = random.sample(index_list, 1)[0]
    #         else:
    #             index = i
    #         index_list.remove(index)
    #         particle_temp.extend(allocation_space[index])
    #     x.append(particle_temp)
    lengthx = len(x)
    for _ in range(N-lengthx):
        particle_temp = []
        for _ in range(tasks_num):
            # sample_allcation = allocation_space[np.random.randint(vehicles_num)]
            particle_temp.extend(allocation_space[np.random.randint(len(allocation_space))])
        x.append(particle_temp)
    x = np.array(x)

    v = (v_max - v_min) * np.random.rand(N, D ) + v_min
    vx = np.random.rand(N,D)

    # 初始化每个粒子的适应度值
    p = x  # 用来存储每个粒子的最佳位置
    p_best = np.ones(N)  # 用来存储每个粒子的适应度值
    min_fail_prob = 1
    for i in range(N):
        p_best[i], penalty_flag, fail_prob = fitness_func(x[i, :], i)
        if penalty_flag:
            min_fail_prob = min(min_fail_prob, fail_prob)
            p_best[i] =  100000000000000
    if not np.std(p_best):
        h_min = min_fail_prob+0.05
    # 初始化全局最优位置与最优值
    g_best = 100000000000000000
    x_best = np.ones(D)
    for i in range(N):
        if p_best[i] < g_best and not penalty_flag:
            g_best = p_best[i]
            x_best = x[i, :].copy()

    gb = np.ones(T)  # global best
    pass
    
    for i in range(T):
        # renew exterior penalty
        fitness_val_list = [0]*N
        penalty_flag_list = [None]*N
        
        for j in range(N):
            # 更新每个个体最优值和最优位置
            
            fitness_val_list[j] , penalty_flag_list[j], fail_prob = fitness_func(x[j,:], i)
            # renew particle
            # if penalty_flag_list[j]:# or np.sum(x[j,:]) > tasks_num*3:
            #     for k in range(10): # renew maximization times
            #         particle_temp =[]
            #         for _ in range(tasks_num):
            #             particle_temp.extend(allocation_space[np.random.randint(len(allocation_space))])
            #         x[j,:] = particle_temp
            #         fitness_val_list[j], penalty_flag_list[j], fail_prob = fitness_func(x[j,:], i)
            #         if not penalty_flag_list[j]: # once the new particle fulfill restriction
            #             break
        pass
            # renew 
        #     if not penalty_flag_list[j] and np.sum(x[j,:]) <= tasks_num*3: # if the allocation has too many vehicle, discard this allocation
        #         local_best_fitness = min(local_best_fitness, fitness_val_list[j])
        #         global_original_best = min(global_original_best, fitness_val_list[j])
                                
        # exterior_penalty = global_original_best - local_best_fitness # should be negative
        for j in range(N):
            # 更新每个个体最优值和最优位置
            # fitness_value, delete_flag = fitness_func(x[j,:], i)
            fitness_value = fitness_val_list[j]
            # if penalty_flag_list[j]:
            #     continue
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
    # print(allocation)
    fitness_value, penalty_flag, fail_prob = fitness_func(x_best, i)
    # print(allocation, penalty_flag, g_best, fail_prob)
    pass
    return allocation, penalty_flag, fail_prob, gb