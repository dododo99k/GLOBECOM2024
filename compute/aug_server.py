from ast import Assert
import os, sys, random
# sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))) # this finds parent dir
import math, copy
import numpy as np
import matplotlib.pyplot as plt
from parameters import *


class ShareCompServer:
    def __init__(self, vid, capacity=6.45):
        self.capacity = capacity * np.clip(1+np.random.randn(), 1-RANDOMS, 1+RANDOMS) # ms-level capacity
        self.tasks = [] # not actually queue, but just accommodate users
        self.time = 0
        self.vid = vid
        self.allocations = {} # key is time slots, each dict is vid-to-resource
        self.minimum = 0.1 # self.capacity/num_of_users  # the minimum capacity if no reserved resource
        self.location = np.random.randint(30, 500, 2) # random generate the vehicle location
        self.generated_task_num = 0
        self.generated_task = []
        
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

        for task in self.tasks:
            task.experienced_resource_allocation = self.capacity / len(self.tasks)

        # pass ### TODO 

    # def task_generator(self): 
        
    #     # ppp sample
    #     # generate cabin tasks from passenager

    #     sample_pdf = random.random()
    #     # print("sample_pdf",sample_pdf)
    #     # target = sample_pdf
    #     left, right = 0, len(self.ppp_CDF) - 1
    #     while left <= right:
    #         mid = left + (right - left) // 2
    #         if self.ppp_CDF[mid] == sample_pdf:
    #             break
    #         elif self.ppp_CDF[mid] < sample_pdf:
    #             left = mid + 1
    #         elif self.ppp_CDF[mid] > sample_pdf:
    #             right = mid - 1
    #     return mid

    
    def step(self,):

        self.time += 1 # increase simulation time

        self.do_resource_allocation()
        # dequeue tasks if they completed (front task in queue)
        
        tasks_dequeued = []

        # dequeue the task if the task is completed, based on remaining compute size
        for task in self.tasks: # increase compute time and total time for all tasks

            average_rate = np.clip(task.experienced_resource_allocation, MINI_ALLOC_COMP, self.capacity)

            task.total_time += 1 # add time unit every time for total
            task.time_vehicle_compute += 1 # add time unit every time for server processing
            task.remain_compute_size = np.clip(task.remain_compute_size - average_rate, 0, None)
            # remove the task
            if task.remain_compute_size <= 0:
                tasks_dequeued.append(copy.deepcopy(task)) # append the task for task completion
                self.tasks.remove(task) # remove it 

        return tasks_dequeued

    def get_num_of_users(self,):
        return len(self.users)

    def enqueue_task(self, task):
        self.tasks += [task]

        return True




