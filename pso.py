# 离散粒子群算法，本质就是将用二进制编码的方式进行解决，将离散问题空间映射到连续粒子运动空间中，其他速度位置更新原则还是和之前连续性粒子群
# 算法保持一致，但是不同的是粒子在状态空间的取值和变化只限于0和1两个值，而速度表示位置取值为 1 的可能性，其表达形式和logistics回归相似
# 连续问题也可以离散化，离散化后收敛速度变快，但是运行时间变长

# 粒子群算法流程
# 1.初始化粒子群，包括群体规模N，每个粒子的位置xi 和速度vi
# 2.计算每个粒子的适应度值fit[i]
# 3.通过适应度值取计算每个个体的适应度值以及全局适应度值
# 4.迭代更新每个粒子的速度和位置
# 5.进行边界条件处理
# 6、判断算法条件是否终止，否则返回步骤2

# 离散粒子群算法

import numpy as np

# 初始化粒子群相关参数
def PSO(tasks, fitness_list, N = 100, T=200 , c1 = 1.5, c2 = 1.5,
        w_max = 0.8 ,w_min = 0.8, v_max = 6, v_min = -6):
    
    def fitness_func(x):
        # x : np.array, [vid] * task_num
        time = 0
        for task_id, vid in enumerate(x):
            time += tasks[task_id].task_size/fitness_list[vid][0]
            time += tasks[task_id].result_size/fitness_list[vid][1]
            time += fitness_list[vid][2]
        return -time # the longer time, the worse fitness value
    
    
    D = len(tasks)
    
    # c1 = 1.5
    # c2 = 1.5
    # w_max = 0.8
    # w_min = 0.8
    # v_max = 10
    # v_min = -10
    # fitness_list :size (vehicle_num,3)
    position_num = len(fitness_list)
    # print_list = []
    # for i in range(position_num):
    #     print_list.append(fitness_list[i][2])
    # print(print_list)
    x = np.random.randint(0, position_num, [N, D]) # 0-20 represent vehicle_id
    v = (v_max - v_min) * np.random.rand(N, D) + v_min
    # vx = np.random.rand(N,D) # 这个是将速度转换成概率的矩阵

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
                if x[j, jj] > position_num - 1:
                    x[j, jj] = position_num - 1
                elif x[j, jj] < 0:
                    x[j, jj] = 0
            # # 进行概率计算并且更新位置
            # vx[j, :] = 1 / (1 + np.exp(-v[j, :]))
            # for ii in range(D):
            #     r = np.random.rand(1)
            #     x[j, ii] = 1 if vx[j, ii] > r else 0
        gb[i] = g_best

    # print("Best Value", gb[T - 1], "Best Position", x_best)
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
#     position_num = 20
#     PSO(N,D,T, position_num)