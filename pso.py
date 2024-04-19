import numpy as np
import time,math

H_MIN = 0.2
MU = 1
lambda_prob = 0.00005
def PSO(tasks, fitness_list, N = 100, T=100 , c1 = 1.5, c2 = 1.5,
        w_max = 0.8 ,w_min = 0.8, v_max = 6, v_min = -6):
    start_time = time.time()
    def fitness_func(particle):
        # fitness_func_start_time = time.time()
        
        # x : np.array, vehicles_num * tasks_num
        
        tasks_time = np.zeros(tasks_num)
        
        prob = np.zeros(tasks_num)
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
        delete_flag = False
        for tkid in range(tasks_num):
            # calculate repeating tasks allocation
            # particle_count = np.sum(particle[ tkid*vehicles_num : (tkid+1)*vehicles_num ])
            prob_temp = 1
            temp_task_time = 0
            for vid in range(vehicles_num):
                if particle[ tkid*vehicles_num + vid ]==1:
                    tasks_time[tkid] += tasks[tkid].remain_size_dl/fitness_list[vid][0]\
                        *(fitness_list[vid][3] + particle_count[vid])
                    
                    tasks_time[tkid] += tasks[tkid].remain_size_ul/fitness_list[vid][1]\
                        *(fitness_list[vid][4] + particle_count[vid])
                    
                    tasks_time[tkid] += tasks[tkid].remain_compute_size/fitness_list[vid][2]\
                        *(fitness_list[vid][4] + particle_count[vid])

                    # prob[tkid] = 1 - edge_vehicles[vid].prob(tasks_time[tkid]) # fail prob
                    prob[tkid] = 1 - math.exp(- tasks_time[tkid]* lambda_prob)
                    prob_temp *= prob[tkid] # np.log(1 + H_MIN - prob[tkid])
                    temp_task_time += tasks_time[tkid]
                    
            # fitness value with barrier function
            value = value + temp_task_time - MU * np.log(1 + H_MIN - prob_temp) # fitness value
            if H_MIN < prob_temp:
                delete_flag = True
        # print('fitness func time:', time.time()-fitness_func_start_time)
        return  [value, delete_flag]

    
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
    
    # ones = np.eye(vehicles_num)
    # twos = np.eye(vehicles_num) + np.eye(vehicles_num, k=1) + np.eye(vehicles_num, k=1-vehicles_num)
    # sample = np.vstack((ones, twos))
    
    # x = np.zeros([N, D ])
    
    # for i in range(tasks_num):
    #     for j in range(N):
    #         index = np.random.randint(np.shape(sample)[0])
    #         x[j,i* vehicles_num : (i+1)*vehicles_num] = sample[index, :]
    
    
    x = np.random.randint(0, 2, [N, D ]) # 0-20 represent vehicle_id

        
    
    v = (v_max - v_min) * np.random.rand(N, D ) + v_min
    vx = np.random.rand(N,D)

    # 初始化每个粒子的适应度值
    p = x  # 用来存储每个粒子的最佳位置
    p_best = np.ones(N)  # 用来存储每个粒子的适应度值
    for i in range(N):
        p_best[i], delete_flag = fitness_func(x[i, :])

    # 初始化全局最优位置与最优值
    g_best = 100000000000000000
    x_best = np.ones(D)
    for i in range(N):
        if p_best[i] < g_best:
            g_best = p_best[i]
            x_best = x[i, :].copy()

    gb = np.ones(T)  # global best
    for i in range(T):
        for j in range(N):
            # 更新每个个体最优值和最优位置
            fitness_value, delete_flag = fitness_func(x[j,:])
            # renew particle which can not satisify the latency probability restriction
            if delete_flag:
                for k in range(10): # renew maximization times
                    x[j,:] = np.random.randint(0, 2, [1, D ])
                    fitness_value, delete_flag = fitness_func(x[j,:])
                    if not delete_flag: # once the new particle fulfill restriction
                        break
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
    pass
    
    all_time = time.time() - start_time
    # print('time usage:', all_time)
    pass
    return allocation
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