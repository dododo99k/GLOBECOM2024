import numpy as np
import os, string, sys, copy, random
import matplotlib.pyplot as plt
from parameters import *

#######################################################################################
#  this is the class for the TASK, it records all the info during the simulation
#  including the identification, latency usage in components, counting timer, transition
#######################################################################################
class Task:
    def __init__(self, tid, curr_time, vid=-1, generated_vid=-1):
        self.tid = tid # the id of this task
        self.vid = vid # the id of this task's original vehicle
        self.start_time = curr_time  # the start time of this task
        self.end_time = None
        self.total_time = 0
        # used by on vehicle compute
        self.time_vehicle_compute = 0
        self.time_ul_transmit = 0 #  the time consumed in ul
        self.time_dl_transmit = 0 #  the time consumed in ul
        
        # remain_compute_size_list = [200, 500, 700]  
        remain_compute_size_list = list(range(1000, 2000, 10))
        self.compute_size = random.sample(remain_compute_size_list, 1)[0]# TODO  XXX
        self.remain_compute_size = self.compute_size
        # used by wireless
        task_size_list = list(range(10, 100, 10)) # kb
        self.task_size = random.sample(task_size_list, 1)[0]# TODO  XXX
        self.remain_size_dl = self.task_size
        # used by broadcast
        result_size_list = list(range(10, 100, 10)) # kb
        self.result_size = random.sample(result_size_list, 1)[0]# TODO  XXX
        self.remain_size_ul = self.result_size

        # generated vehicle id
        self.generated_vid = generated_vid

        # expected latency
        # self.avg_experenced_data_rate = 0 # see post process
        # self.avg_experenced_compute_rate = 0  # see post process
        # self.expected_downlink_latency = model_downlink_latency(self.task_size, radio)
        # self.expected_uplink_latency = model_uplink_latency(self.task_size, radio)
        # self.expected_compute_latency = model_compute_latency(self.remain_server_compute_size, compute)
        # self.expected_e2e_latency = model_latency(self.remain_local_compute_size, self.task_size, self.remain_server_compute_size, radio, compute)
        self.sum_computed_size = 0
        self.experienced_resource_allocation = 0

    def post_process(self,):
        try:
            self.avg_experenced_data_rate = self.task_size / (self.time_ul_transmit)
        except: pass
        try:
            self.avg_experenced_compute_rate = self.compute_size / self.time_vehicle_compute
        except: pass
        self.dl_time_percentage = self.time_ul_transmit/self.total_time
        self.ul_time_percentage = self.time_ul_transmit/self.total_time
        self.compute_time_percentage = self.time_vehicle_compute/self.total_time
        
