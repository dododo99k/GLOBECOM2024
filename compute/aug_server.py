from ast import Assert
import os, sys, random
# sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))) # this finds parent dir
import math, copy
import numpy as np
import matplotlib.pyplot as plt
from parameters import *


class ShareCompServer:
    def __init__(self, vid, capacity=6.45, lambda_prob = 0.0001, kkt_allocation = True):
        self.capacity = capacity * np.clip(1+np.random.randn(), 1-RANDOMS, 1+RANDOMS) # ms-level capacity
        self.tasks = [] # not actually queue, but just accommodate users
        self.time = 0
        self.vid = vid
        self.allocations = {} # key is time slots, each dict is vid-to-resource
        self.minimum = 0.1 # self.capacity/num_of_users  # the minimum capacity if no reserved resource
        self.location = np.random.randint(30, 200, 2) # random generate the vehicle location
        self.generated_task_num = 0
        self.generated_task = []
        self.local_computing = False
        self.lambda_prob = lambda_prob
        self.kkt_allocation = kkt_allocation
        
        #######################  ppp look up table (pdf), for fast ppp sample
        # ppp_CDF = []
        # cdf = 0
        # self.ppp_lambda = 2 # maximum: 700, will overflow if larger
        # # n = self.ppp_lambda
        # n = 0 
        # while 1:
        #     prob = self.ppp_lambda**n/math.factorial(n)*math.exp(-self.ppp_lambda)
        #     cdf += prob
        #     ppp_CDF.append(cdf)
        #     # print(n, pdf)
        #     if cdf >= 1 or ( n > self.ppp_lambda and prob <= 1e-16): # float number precision
        #         break
        #     n+=1
        # self.max_tasks = n
        # self.ppp_CDF = ppp_CDF

    
    def get_utilization(self,):
        return 1 if len(self.tasks)>0 else 0

    def do_resource_allocation(self, ):
        
        # sub algorithm of allcation here
        # task.experienced_resource_allocation = 10
        
        # KKT allocation
        if self.kkt_allocation:
            # remain_capacity_sum = 0
            portion = [0]*len(self.tasks)
            # modify = [False]*len(self.tasks)
            for i, task in enumerate(self.tasks):
                portion[i] = math.sqrt(task.remain_compute_size)
            remain_capacity_sum = sum(portion)
            sum_compute_size = 0
            for i, task in enumerate(self.tasks):
                task.experienced_resource_allocation = round(portion[i] / remain_capacity_sum , 8) * self.capacity
                
                sum_compute_size+=task.experienced_resource_allocation
            
            left_compute_resource = self.capacity-sum_compute_size
            if self.tasks and left_compute_resource > 0:
                self.tasks[np.argsort(portion)[0]].experienced_resource_allocation += left_compute_resource
        # average allocation
        else: 
            for task in self.tasks:
                task.experienced_resource_allocation = self.capacity / len(self.tasks)

        # pass ### TODO

    
    def step(self,):
        
        self.local_computing = False
        self.time += 1 # increase simulation time

        self.do_resource_allocation()
        # dequeue tasks if they completed (front task in queue)
        
        tasks_dequeued = []
        tasks_remove_list = []

        # dequeue the task if the task is completed, based on remaining compute size
        vehicle_compute_resource_sum = 0
        debug_flag = False
        for task in self.tasks: # increase compute time and total time for all tasks
            if task.experienced_resource_allocation < MINI_ALLOC_COMP:
                debug_flag = True
            average_rate = np.clip(task.experienced_resource_allocation, MINI_ALLOC_COMP, self.capacity)

            task.total_time += 1 # add time unit every time for total
            task.time_vehicle_compute += 1 # add time unit every time for server processing
            task.remain_compute_size = np.clip(task.remain_compute_size - average_rate, 0, None)
            task.sum_computed_size += average_rate
            vehicle_compute_resource_sum += average_rate
            # remove the task
            # if task.remain_compute_size <= 0:
            #     tasks_dequeued.append(copy.deepcopy(task)) # append the task for task completion
            #     self.tasks.remove(task) # remove it 
            # remove the task by Jiahe Cao
            if task.remain_compute_size <= 0:
                tasks_dequeued.append(copy.deepcopy(task)) # append the task for task completion
                tasks_remove_list.append(task) # remove it
        
        # check if resource is not fully utilized or overflow
        # if debug_flag or (self.tasks and vehicle_compute_resource_sum < self.capacity):
        #     print('warning', vehicle_compute_resource_sum)
        # remove the task by Jiahe Cao
        for remove_tk in tasks_remove_list:
            self.tasks.remove(remove_tk)
            
            
        for task in self.tasks:
            if task.generated_vid == self.vid:
                self.local_computing = True
                break
            # elif self.local_computing: # taks.generated_vid != self.vid AND slef.local_somputing = True
                
        return tasks_dequeued

    def get_num_of_users(self,):
        return len(self.users)

    def enqueue_task(self, task):
        self.tasks += [task]

        return True
    
    def prob(self, time_length = 1000): 
        return math.exp(- time_length* self.lambda_prob)
    
        
    



