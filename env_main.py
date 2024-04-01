import os, sys, math, time, random
from tqdm import tqdm
import numpy as np
import pickle
import time
import matplotlib.pyplot as plt
from parameters import *
# from functions import *
import copy


from utils import Task # task
from uplink.uplink import Uplink
from downlink.downlink import Downlink
from compute.aug_server import ShareCompServer  # vehicle server

# todo: vehicle location geenration once only
# todo: traffic generator, ppp maybe or trace
# todo:

class env_main():
    def __init__(self, num_vehicles=20, capacity_vehicle=465, bandwidth_ul=1.0, bandwidth_dl=1.0):
        self.num_vehicles = num_vehicles
        print("number of simulated vehicles ", num_vehicles)
        np.random.seed(int(time.time()*1000000)%1000)

        ##resources INTERFACE for changing resource in different modules, XXX ONLY take effect after reset() XXX 
        self.res_bw_ul = bandwidth_ul
        self.res_bw_dl = bandwidth_dl
        self.res_cap_vehicle = capacity_vehicle
        self.time = 0 # reset the decision time
        self.Vehicles = []

        self.stats = {'latency':[],}
        ###################################### generate vehicles ################################################ 

        for vid in range(self.num_vehicles):
            self.Vehicles.append(ShareCompServer(vid, capacity=self.res_cap_vehicle)) # initialize the vehicle entity

        ###################################### generate wireless ################################################

        self.downlink = Downlink(bandwidth=self.res_bw_dl)

        for vid in range(self.num_vehicles):
            self.downlink.add_in_receiver(vid, self.Vehicles[vid].location) # initial coords is bs location
        self.downlink.update_receiver_path_loss() # update path loss for all receivers

        ###################################### generate wireless ################################################

        self.uplink = Uplink(bandwidth=self.res_bw_ul)

        for vid in range(self.num_vehicles):
            self.uplink.add_in_receiver(vid, self.Vehicles[vid].location) # initial coords is bs location
        self.uplink.update_receiver_path_loss() # update path loss for all receivers

        ######################################  ################################################
       
        self.TASKS = []

        #######################  ppp look up table (pdf), for fast ppp sample
        ppp_CDF = []
        cdf = 0
        # self.ppp_lambda = self.num_vehicles//2 # maximum: 700, will overflow if larger
        self.ppp_lambda = 0.5 # maximum: 700, will overflow if larger
        # n = self.ppp_lambda
        n = 0 
        while 1:
            prob = self.ppp_lambda**n/math.factorial(n)*math.exp(-self.ppp_lambda)
            cdf += prob
            ppp_CDF.append(cdf)
            # print(n, pdf)
            if cdf >= 1 or ( n > self.ppp_lambda and prob <= 1e-16): # float number precision
                break
            n+=1
        self.max_tasks = n
        self.ppp_CDF = ppp_CDF
        # print("self.ppp_pdf",self.ppp_pdf)


    def task_generator(self): # assuming allow max number of tasks within one time slot
        
        sample_pdf = random.random()
        # print("sample_pdf",sample_pdf)
        # target = sample_pdf
        left, right = 0, len(self.ppp_CDF) - 1
        while left < right:
            mid = left + (right - left) // 2
            if self.ppp_CDF[mid] < sample_pdf:
                left = mid + 1
            elif self.ppp_CDF[mid] > sample_pdf:
                right = mid - 1
        mid = left + (right - left) // 2

        if self.ppp_CDF[mid] < sample_pdf and sample_pdf <= self.ppp_CDF[mid+1]:
            mid+=1
        elif self.ppp_CDF[mid+1] < sample_pdf:
            mid+=2
        
        return mid
        
        
    def run_algorithm(self, tasks):
        for task in tasks:
            # np.random.seed(int(time.time()*1000000)%1000)
            # task.vid = np.random.randint(self.num_vehicles)
            task.vid = task.generated_vid

    def step(self, ):
        
        #################################     create tasks      ###########################################
        # create task based on traffic generator from PPP or trace
        # traffic = self.task_generator() # PPP generator or trace from users

        current_task_num = 0
        tasks = []
        for vehicle in self.Vehicles:
            in_vehicle_tasks = self.task_generator() # assign genertaed tasks number for each vehicle
            current_task_num += in_vehicle_tasks
            vehicle.generated_task_num = in_vehicle_tasks

            generated_task = [Task(tid=len(self.TASKS)+1+k, curr_time=self.time, generated_vid = vehicle.vid) for k in range(vehicle.generated_task_num)]
            tasks += copy.deepcopy(generated_task)
            self.TASKS += copy.deepcopy(generated_task)
            pass

        # tasks = [Task(tid=len(self.TASKS)+1+k, curr_time=self.time) for k in range(current_task_num)] # vid=vid to be added

        ################################# algrotihm execution #################################

        self.run_algorithm(tasks) # generate the optimized vid for each task

        # self.TASKS += copy.deepcopy(tasks) # append the created task into all task list
        local_tasks = []
        remove_list = []
        for tk in tasks:
            if tk.vid == tk.generated_vid:
                local_tasks.append(copy.deepcopy(tk)) # local computing task without transmitting
                remove_list.append(tk) # tasks need to be tansmitted
        
        # remove local computing tasks
        for tk in remove_list:
            tasks.remove(tk)
        
        pass
    
        for tk in tasks:
            self.downlink.enqueue_task(tk)

        ############################### run whole end-to-end process #######################################
            
        ############ run downlink wireless ######################

        complete_downlink = self.downlink.step()
        
        # put task into vehicle compute if wireless transmission is completed,
        for tk in complete_downlink:
            self.Vehicles[tk.vid].enqueue_task(tk)
        
        # put local tasks into vehicle
        for tk in local_tasks:
            self.Vehicles[tk.vid].enqueue_task(tk)
        
        ############ run vehicle #######################

        for vehicle in self.Vehicles:
            # run computation on vehicle
            complete_compute = vehicle.step()
            
            for tk in complete_compute:
                if tk.vid != tk.generated_vid: # upload tasks
                    self.uplink.enqueue_task(tk)
                else: # local tasks time
                    tk.end_time = self.time # assign the end time of this task
                    # print(task.total_time)
                    self.stats['latency'].append(tk.total_time)
                    tk.post_process() # calculate avg data rate and compute rate
                    


        ############ run uplink wireless ######################

        complete_uplink = self.uplink.step()
        
        # put task into server compute if wireless transmission is completed
        for task in complete_uplink:
            # determine when the vehicle is available for starting task again
            task.end_time = self.time # assign the end time of this task
            # print(task.total_time)
            self.stats['latency'].append(task.total_time)
            task.post_process() # calculate avg data rate and compute rate

        ##################################################################
        self.time += 1 # increase decision time 

        return None


################################################################################################
# this is the main env, which is the simulator (time driven)
# for each time_unit we update all the tasks in each modules
# NOT all the time we have an action to take, i.e., not return back to the DRL agent
# as a result, this simualtion can be slow if too long delay, because of no action
# You can increase the TIME_UNIT to accelerate, but it may not very accurate due to large granularity
# The task is with its transition, which is retrived when finished
################################################################################################

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--length', type=int, default=100)
    parser.add_argument('--exp_name', type=str, default='env_main')
    parser.add_argument('--car_num', type=int, default=20)
    parser.add_argument('--traffic', type=int, default=10)
    parser.add_argument('--mode', type=str, default="base")
    args = parser.parse_args()

    env = env_main(num_vehicles=args.car_num, mode = args.mode) 

    start_time = time.time()
    for i in tqdm(range(args.length)):
        # random select vid and action
        initial_state = env.step()
    print("time usage:", time.time()-start_time)

    # plot the CDF of the achieved all user latency
    fig, ax = plt.subplots()
    plt.hist(np.array(env.stats['latency']), bins=100,cumulative=True, density=True, histtype='step',  color='C0',)
    fix_hist_step_vertical_line_at_end(ax)
    plt.show()
    fig.savefig('CDF of latency.png')
    print('done')
