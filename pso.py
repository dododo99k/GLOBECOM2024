import numpy as np
import copy

H_MIN = 0.1 
MU = 1000

def PSO(edge_vehicles, tasks, fitness_list, N = 100, T=200 , c1 = 1.5, c2 = 1.5,
        w_max = 0.8 ,w_min = 0.8, v_max = 6, v_min = -6):
        
    def fitness_func(particle):
        # x : np.array, vehicles_num * tasks_num
        
        tasks_time = np.zeros_like(particle)
        
        prob = np.zeros_like(particle)
        # list of vehicle, store tasks number when multi-tasks are allocated to one vehicle
        particle_count = [0] * len(vehicles_num) 
        
        
        for vid in range(vehicles_num):
            # calculate repeating tasks allocation
            for tkid in range(tasks_num):
                particle_count[vid] += particle[ tkid*vehicles_num + vid ]
                # np.sum(particle[ tkid*vehicles_num : (tkid+1)*vehicles_num ])
        pass
    
        value = 0
        for tkid in range(tasks_num):
            # calculate repeating tasks allocation
            # particle_count = np.sum(particle[ tkid*vehicles_num : (tkid+1)*vehicles_num ])
            prob_temp = 1
            temp_task_time = 0
            for vid in range(vehicles_num):
                if particle[ tkid*vehicles_num + vid ]==1:
                    tasks_time[tkid] += tasks[tkid].remain_size_dl/fitness_list[vid][0]*particle_count[vid]
                    
                    tasks_time[tkid] += tasks[tkid].remain_size_ul/fitness_list[vid][1]*particle_count[vid]
                    
                    tasks_time[tkid] += tasks[tkid].remain_compute_size/fitness_list[vid][2]\
                        *(fitness_list[vid][3] + particle_count[vid])

                    prob[tkid] = 1 - edge_vehicles[i].prob(tasks_time[tkid]) # fail prob
                    
                    prob_temp *= np.log(1 + H_MIN - prob[tkid])
                    temp_task_time += tasks_time[tkid]
                    
            # fitness value with barrier function
            value = value + temp_task_time - MU * prob_temp # fitness value
        
        return value

    
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
    
    x = np.random.randint(0, vehicles_num, [N, D ]) # 0-20 represent vehicle_id
    v = (v_max - v_min) * np.random.rand(N, D ) + v_min
    vx = np.random.rand(N,D)

    # 初始化每个粒子的适应度值
    p = x  # 用来存储每个粒子的最佳位置
    p_best = np.ones(N)  # 用来存储每个粒子的适应度值
    for i in range(N):
        p_best[i] = fitness_func(x[i, :])

    # 初始化全局最优位置与最优值
    g_best = - 10000000
    x_best = np.ones(D )
    for i in range(N):
        if p_best[i] > g_best:
            g_best = p_best[i]
            x_best = x[i, :].copy()

    gb = np.ones(T)  # global best
    for i in range(T):
        for j in range(N):
            # 更新每个个体最优值和最优位置
            if p_best[j] < fitness_func(x[j,:]):
                p_best[j] = fitness_func(x[j, :])
                p[j, :] = x[j, :].copy()
            # 更新全局最优位置和最优值
            if p_best[j] > g_best:
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
            
            # v_temp = np.round(v[j, :])
              
            # x[j, :] += v_temp.astype(int)
            vx[j, :] = 1 / (1 + np.exp(-v[j, :]))
            
            # 边界条件处理
            for jj in range(D):
                if (v[j, jj] > v_max) or (v[j, jj] < v_min):
                    v[j, jj] = v_min + np.random.rand(1) * (v_max - v_min)
            # 进行概率计算并且更新位置
            vx[j, :] = 1 / (1 + np.exp(-v[j, :]))
            for ii in range(D):
                r = np.random.rand(1)
                x[j, ii] = 1 if vx[j, ii] > r else 0
        gb[i] = g_best
    
    allocation =[]
    for i in len(tasks_num):
        allocation.append(x_best)
    
    
    return x_best
    # plt.plot(range(T), gb)
    # plt.xlabel("Iterations")
    # plt.ylabel("Fitness")
    # plt.title("Fitness evolution")
    # plt.show()
    

# if __name__ == '__main__':
#     N = 100 # particle number
#     D = 10 # task number
#     T = 200
#     vehicles_num = 20
#     PSO(N,D,T, vehicles_num)