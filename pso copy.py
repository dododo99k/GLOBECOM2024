import numpy as np
import copy

H_MIN = 0.1 

def PSO(edge_vehicles, tasks, fitness_list, N = 100, T=200 , c1 = 1.5, c2 = 1.5,
        w_max = 0.8 ,w_min = 0.8, v_max = 6, v_min = -6):
    
    def task_prob(tk, vehicle_list): # estimate positive prob in terms of time latency
        prob = np.zeros(vehicle_num)
        # vehicle_list: a list store vehicle_id, may
        for vid in vehicle_list:
            task_time = tk.remain_size_dl/fitness_list[vid][0]
                
            task_time += tk.remain_size_ul/fitness_list[vid][1]
            
            task_time += tk.remain_compute_size/fitness_list[vid][4]\
                            *(fitness_list[vid][3]+1)

            prob[vid] = edge_vehicles[vid].prob(task_time)
        return prob
        
    def fitness_func(particle):
        # x : np.array, [vid] * task_num
        
        task_time = np.zeros_like(particle)
        
        prob = np.zeros_like(particle)

        # count the repeating elements in one particle
        # prevent multi-tasks are allocated to the same edge server
        particle_count = np.zeros_like(particle)
        for i in range(len(particle)):
            for j in np.unique(particle):
                if particle[i] == j:
                    particle_count[i] = np.sum(particle==j)
        
        for i, vid in enumerate(particle):
            task_time[i] += tasks[i].remain_size_dl/fitness_list[vid][0]*particle_count[i]
            
            task_time[i] += tasks[i].remain_size_ul/fitness_list[vid][1]*particle_count[i]
            
            task_time[i] += tasks[i].remain_compute_size/fitness_list[vid][4]\
                            *(fitness_list[vid][3]+1)*particle_count[i]

            prob[i] = edge_vehicles[i].prob(task_time[i])
        
        # TODO
        # renew the fitness value
        
        value = np.sum(task_time)
        
        return value

    
    # c1 = 1.5
    # c2 = 1.5
    # w_max = 0.8
    # w_min = 0.8
    # v_max = 10
    # v_min = -10
    D = len(tasks)
    vehicle_num = len(fitness_list) # 20 by default

    task_2_vehicle_num = np.zeros(D) # duplicate number of tasks for repeating allocation
    # calculate which task should be duplicated
    tasks_re = []
    vehicles_temp = copy.deepcopy(edge_vehicles)
    vehicles_list = list(range(vehicle_num))
    
    for i, task in enumerate(tasks):
        prob = task_prob(task, vehicles_list) # calculate all positive prob for one task allcated to all edge vehicles
        prob_sort = np.sort(-prob)[::-1] # sort of positive prob
        prob_sort = np.argsort(-prob)[::-1] # arg sort
        
        prob_temp = 1
        for j in range(len(prob_sort)):
            prob_temp = prob_temp * (1 - prob[j]) # multiply of fail prob
            vehicles_list.remove(prob_sort[j]) # remove current vehicle, current vehicle should not calculate next task
            if prob_temp <  H_MIN:
                break
        task_2_vehicle_num[i] = j+1
        # duplicate tasks
        for _ in range(task_2_vehicle_num[i]):
            tasks_re.append(task)
    
    
    
    
    
    
    # PSO main part
    D = len(tasks) # re-count the task number after duplication
    
    x = np.random.randint(0, vehicle_num, [N, D]) # 0-20 represent vehicle_id
    v = (v_max - v_min) * np.random.rand(N, D) + v_min

    # 初始化每个粒子的适应度值
    p = x  # 用来存储每个粒子的最佳位置
    p_best = np.ones(N)  # 用来存储每个粒子的适应度值
    for i in range(N):
        p_best[i] = fitness_func(x[i, :])

    # 初始化全局最优位置与最优值
    g_best = - 10000000
    x_best = np.ones(D)
    for i in range(N):
        if p_best[i] > g_best:
            g_best = p_best[i]
            x_best = x[i, :].copy()

    gb = np.ones(T)  # 用来存储每依次迭代的最优值
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
            
            v_temp = np.round(v[j, :])
              
            x[j, :] += v_temp.astype(int)
            
            for jj in range(D):
                if x[j, jj] > vehicle_num - 1:
                    x[j, jj] = vehicle_num - 1
                elif x[j, jj] < 0:
                    x[j, jj] = 0
            # # 进行概率计算并且更新位置
            # vx[j, :] = 1 / (1 + np.exp(-v[j, :]))
            # for ii in range(D):
            #     r = np.random.rand(1)
            #     x[j, ii] = 1 if vx[j, ii] > r else 0
        gb[i] = g_best

    # print("Best Value", gb[T - 1], "Best Position", x_best)
    allocation = []
    temp = 0
    for vehicle_num in task_2_vehicle_num:
        tk_allocation = []
        for i in range(len(vehicle_num)):
            tk_allocation.append(x_best[temp + i]) # allocation indicator for one task
        temp += vehicle_num
        allocation.append(tk_allocation)
    
    
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
#     vehicle_num = 20
#     PSO(N,D,T, vehicle_num)