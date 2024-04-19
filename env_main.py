import os, sys, math, time, random, statistics
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

from pso import PSO

# todo: vehicle location geenration once only
# todo: traffic generator, ppp maybe or trace
# todo:

class env_main():
    def __init__(self, num_vehicles=20, capacity_vehicle=10, bandwidth_ul=10, bandwidth_dl=10,
                 poisson_density = 0.015, ego_poisson_density = 0.02, length = 100 , mode = 'base'):
        self.num_vehicles = num_vehicles + 1 # first vheicle is ego vehicleç
        print("number of simulated vehicles ", num_vehicles)
        print("poisson density ", poisson_density)
        
        np.random.seed(1000) # debug
        random.seed(1000) # debug
        # np.random.seed(int(time.time()*1000000)%1000)

        ##resources INTERFACE for changing resource in different modules, XXX ONLY take effect after reset() XXX 
        self.res_bw_ul = bandwidth_ul
        self.res_bw_dl = bandwidth_dl
        self.res_cap_vehicle = capacity_vehicle
        self.time = 0 # reset the decision time
        self.length = length
        self.Vehicles = []
        self.mode = mode
        self.stats = {'latency':[], 'ego_v_latency':[]}
        self.free_vehicle_num = []
        self.vehicle_tasks_num = []
        # self.ego_vid = 100
        ###################################### generate vehicles ################################################ 
        
        # ego vehicle vid = 0
        for vid in range(self.num_vehicles):
            self.Vehicles.append(ShareCompServer(vid, capacity=self.res_cap_vehicle)) # initialize the vehicle entity
        self.Vehicles[0].location = np.array((0,0))
        
        ###################################### generate wireless ################################################

        self.downlink = Downlink(bandwidth=self.res_bw_dl)

        for vid in range(1,self.num_vehicles):
            self.downlink.add_in_receiver(vid, self.Vehicles[vid].location) # initial coords is bs location
        self.downlink.update_receiver_path_loss() # update path loss for all receivers
        self.downlink.initialize_receiver()
        ###################################### generate wireless ################################################

        self.uplink = Uplink(bandwidth=self.res_bw_ul)

        for vid in range(1,self.num_vehicles):
            self.uplink.add_in_receiver(vid, self.Vehicles[vid].location) # initial coords is bs location
        self.uplink.update_receiver_path_loss() # update path loss for all receivers
        self.uplink.initialize_receiver()
        ######################################  ################################################
       
        self.TASKS = []
        self.egoTASKS = []
        self.finishedTASKS = []
        self.repeated_task_num = []

        #######################  ppp look up table (pdf), for fast ppp sample
        ppp_CDF = []
        cdf = 0
        self.ppp_lambda = poisson_density # 0.015 by deault
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

        #######################  ego vehicle ppp look up table (pdf), for fast ppp sample
        ego_ppp_CDF = []
        cdf = 0
        self.ego_ppp_lambda = ego_poisson_density # 0.02 by deault
        n = 0 
        while 1:
            prob = self.ego_ppp_lambda**n/math.factorial(n)*math.exp(-self.ego_ppp_lambda)
            cdf += prob
            ego_ppp_CDF.append(cdf)
            # print(n, pdf)
            if cdf >= 1 or ( n > self.ego_ppp_lambda and prob <= 1e-16): # float number precision
                break
            n+=1
        self.ego_ppp_CDF = ego_ppp_CDF
        # print("self.ppp_pdf",self.ppp_pdf)


    def task_generator(self, ppp_CDF): # assuming allow max number of tasks within one time slot
        sample_pdf = random.random()
        left, right = 0, len(ppp_CDF) - 1
        while left < right:
            mid = left + (right - left) // 2
            if ppp_CDF[mid] < sample_pdf:
                left = mid + 1
            elif ppp_CDF[mid] > sample_pdf:
                right = mid - 1
        mid = left + (right - left) // 2

        if ppp_CDF[mid] < sample_pdf and sample_pdf <= ppp_CDF[mid+1]:
            mid+=1
        elif ppp_CDF[mid+1] < sample_pdf:
            mid+=2
        
        return mid
        
        
    def run_algorithm(self, tasks):
        # def local_computing(self, task):
        #     task.vid = task.generated_vid
        #     self.Vehicle[task.generated_vid].local_computing = True
        #     try:
        #         free_vehicle_id.remove(tasks.vid)
        #     except:
        #         pass
        
        ############################# local computing mode ####################
        if self.mode == 'base':
            free_vehicle_id = list(range(self.num_vehicles))
            
            for task in tasks:
                task.vid = task.generated_vid
                self.Vehicles[task.generated_vid].local_computing = True
                try:
                    free_vehicle_id.remove(task.vid)
                except:
                    pass
            
            for v in self.Vehicles:
                if v.tasks: # no tasks in vehicle
                    try:
                        free_vehicle_id.remove(v.vid)
                    except:
                        pass
            
            # record free vehicle number, incoming tasks could be allocated to the free vehicles
            self.free_vehicle_num.append(len(free_vehicle_id))
            
        ############################# edge computing mode ####################
        elif self.mode == "pso":
            free_vehicle_id = list(range(self.num_vehicles))
            local_computing_vehicle_id = [] #list(range(self.num_vehicles))
            allocated_tasks = []
            
            for v in self.Vehicles[1:]:
                if v.tasks: # no tasks in vehicle
                    try:
                        free_vehicle_id.remove(v.vid)
                    except:
                        pass
                    if v.local_computing: # local computing tasks in vehicle
                        local_computing_vehicle_id.append(v.vid)
            
            # record free vehicle number, incoming tasks could be allocated to the free vehicles
            self.free_vehicle_num.append(len(free_vehicle_id))
            
            
            # size : (number of edge vehicles, 3), downlink speed, uplink speed, remaining tasks computing time

            # fitness_list[:][0] : wireless dl transmitting speed
            # fitness_list[:][1] : wireless ul transmitting speed
            # fitness_list[:][2] : vehicle computing capacity
            # fitness_list[:][3] : remaining dl tasks number
            # fitness_list[:][4] : remaining ul tasks number
            # fitness_list[:][5] : remaining computing tasks number
            
            fitness_list = np.zeros(((self.num_vehicles - 1),6))
            
            # record wireless transmitting speed
            for receiver_id, receiver in enumerate(self.downlink.receivers): # receiver 1-21
                #if receiver_id>0:
                fitness_list[receiver_id][0] = self.downlink.receivers[receiver].capacity
                fitness_list[receiver_id][3] = self.downlink.receivers[receiver].task_num
                #print('dl id:',receiver_id, fitness_list[receiver_id][0], fitness_list[receiver_id][3])
            for receiver_id, receiver in enumerate(self.uplink.receivers): # receiver 1-21
                #if receiver_id>0:
                fitness_list[receiver_id][1] = self.uplink.receivers[receiver].capacity
                fitness_list[receiver_id][4] = self.uplink.receivers[receiver].task_num
                #print('ul id:',receiver_id, fitness_list[receiver_id][1], fitness_list[receiver_id][4])
            pass
            # edge server local computing
            for task in tasks:
                if task.generated_vid==0:
                    allocated_tasks.append(task)
                elif task.generated_vid > 0:
                    task.vid = task.generated_vid
                    self.Vehicles[task.generated_vid].local_computing = True
                    # fitness_list[task.generated_vid-1][2] += \
                    #     task.remain_compute_size/self.Vehicles[task.generated_vid].capacity
                    try:
                        free_vehicle_id.remove(task.vid)
                    except:
                        pass
            
            if not allocated_tasks: # if there is no ego vehicle tasks, no need to do PSO
                return None

            
            # record edge server remaining computing time
            for vehicle in self.Vehicles[1:]:
                task_sum = 0
                for task in vehicle.tasks:
                    task_sum += task.remain_compute_size
                
                fitness_list[vehicle.vid-1][2] = vehicle.capacity
                fitness_list[vehicle.vid-1][5] = len(vehicle.tasks)
                # fitness_list[vehicle.vid-1][4] = self.Vehicles[task.generated_vid].capacity
                

            result = PSO( allocated_tasks, fitness_list )
            
            repeated_task_num = 0
            for i, task in enumerate(allocated_tasks):
                task.vid = result[i] # maybe one list or INT
                repeated_task_num += len(result[i])
                
            self.repeated_task_num.append(repeated_task_num/len(allocated_tasks))
                
            # if not self.Vehicles[0].tasks:
            #     allocated_tasks[0].vid = allocated_tasks[0].generated_vid
                
            for tk in tasks:
                if tk.generated_vid==0:
                    for allocated_tk in allocated_tasks:
                        if allocated_tk.tid == tk.tid:
                            tk.vid = allocated_tk.vid
        
        pass

        

    def step(self, ):
        
        #################################     create tasks      ###########################################
        # create task based on traffic generator from PPP or trace
        # traffic = self.task_generator() # PPP generator or trace from users

        current_task_num = 0
        tasks = []
        
        if self.time<=self.length:
            for vehicle in self.Vehicles:
                if vehicle.vid == 0: # ego vehicle
                    # assign genertaed tasks number for each vehicle
                    ego_vehicle_task_num = self.task_generator(self.ego_ppp_CDF)
                    # if ego_vehicle_task_num>=2:
                    #     print("time {}, ego vehicle task number:{}".format(self.time, ego_vehicle_task_num,))
                    vehicle.generated_task_num = ego_vehicle_task_num
                    # create tasks
                    generated_task = [Task(tid=len(self.TASKS)+1+k, curr_time=self.time, 
                                           generated_vid = vehicle.vid) for k in range(ego_vehicle_task_num)]
                    # copy tasks 
                    tasks += copy.deepcopy(generated_task)
                    self.TASKS += copy.deepcopy(generated_task)
                    
                elif self.ppp_lambda: # other vehicles
                    in_vehicle_tasks = self.task_generator(self.ppp_CDF)
                    current_task_num += in_vehicle_tasks # record other vhicle tasks num
                    vehicle.generated_task_num = in_vehicle_tasks
                    generated_task = [Task(tid=len(self.TASKS)+1+k, curr_time=self.time, 
                                           generated_vid = vehicle.vid) for k in range(vehicle.generated_task_num)]
                    # copy tasks 
                    tasks += copy.deepcopy(generated_task)
                    self.TASKS += copy.deepcopy(generated_task)

        # tasks = [Task(tid=len(self.TASKS)+1+k, curr_time=self.time) for k in range(current_task_num)]
        # # vid=vid to be added

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
        
        # allocation
        for tk in tasks:
            if isinstance(tk.vid,list):
                # repeated allocation
                vid_list = tk.vid
                for vid in vid_list:
                    new_tk =copy.copy(tk)
                    new_tk.vid = vid
                    self.downlink.enqueue_task(new_tk)
                    pass

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
        vehicle_tasks_num = 0
        for vehicle in self.Vehicles:
            # run computation on vehicle
            vehicle_tasks_num += len(vehicle.tasks)
            complete_compute = vehicle.step()
            
            for tk in complete_compute:
                if tk.vid != tk.generated_vid: # upload tasks
                    self.uplink.enqueue_task(tk)
                else: # local tasks time
                    tk.end_time = self.time # assign the end time of this task
                    # print(task.total_time)
                    self.stats['latency'].append(tk.total_time)
                    if tk.generated_vid == 0:
                        self.stats['ego_v_latency'].append(tk.total_time)
                    tk.post_process() # calculate avg data rate and compute rate
        
        self.vehicle_tasks_num.append(vehicle_tasks_num/len(self.Vehicles))

        ############ run uplink wireless ######################

        complete_uplink = self.uplink.step()
        
        # put task into server compute if wireless transmission is completed
        for task in complete_uplink:
            # determine when the vehicle is available for starting task again
            task.end_time = self.time # assign the end time of this task
            # print(task.total_time)
            self.stats['latency'].append(task.total_time)
            if task.generated_vid == 0:
                self.stats['ego_v_latency'].append(task.total_time)
            task.post_process() # calculate avg data rate and compute rate
            self.finishedTASKS.append(task)
            # with open('task_log.pkl','rb') as file:
                
        ##################################################################
        self.time += 1 # increase decision time 
        if self.time % 500 == 1:
            pass
        # print('env time', self.time)
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
    parser.add_argument('--length', type=int, default=1000) # ms
    parser.add_argument('--exp_name', type=str, default='env_main')
    parser.add_argument('--car_num', type=int, default=20)
    parser.add_argument('--traffic', type=int, default=10)
    parser.add_argument('-pd','--poisson_density', type=float, default=0.000)
    parser.add_argument('-epd','--ego_poisson_density', type=float, default=0.03)
    parser.add_argument('--mode', type=str, default="pso")
    parser.add_argument('--show_figure', action="store_true") #
    args = parser.parse_args()

    env = env_main(num_vehicles=args.car_num, poisson_density = args.poisson_density, 
                   ego_poisson_density = args.ego_poisson_density,
                   length = args.length, mode = args.mode)

    start_time = time.time()
    for i in tqdm(range(int(2*args.length))):
        # random select vid and action
        initial_state = env.step()
    print("time usage:", time.time()-start_time)

    output_hal = open('finished tasks, ego V density %s, time length %s, mode %s.pkl'\
                %(args.ego_poisson_density, args.length, args.mode), 'wb')
    # for tk in env.finishedTASKS:
    str = pickle.dumps(env.finishedTASKS)
    output_hal.write(str)
    output_hal.close()

    output_hal = open('all tasks, ego V density %s, time length %s, mode %s.pkl'\
                %(args.ego_poisson_density, args.length, args.mode), 'wb')
    # for tk in env.finishedTASKS:
    str = pickle.dumps(env.TASKS)
    output_hal.write(str)
    output_hal.close()
    
    # plot the CDF of the achieved all user latency
    
    # fig, ax = plt.subplots()
    # plt.hist(np.array(env.stats['latency']), bins=100,cumulative=True, density=True, histtype='step',  color='C0',)
    # fix_hist_step_vertical_line_at_end(ax)
    # plt.title('Possion Distribution density = %s' %(args.poisson_density))
    # plt.xlabel('latency')
    # plt.ylabel("CDF")
    # plt.show()
    # fig.savefig('CDF of latency on PP density %s.png' %(args.poisson_density))
    plt_size = 8
    fig, ax = plt.subplots(2, 2, figsize=(2*plt_size,plt_size))
    plt.suptitle('Possion Distribution density = %s' %(args.ego_poisson_density))
    
    ax1 = ax[0,0]
    ax1.hist(np.array(env.stats['ego_v_latency']), bins=100,cumulative=True, density=True, histtype='step',  color='C0',)
    fix_hist_step_vertical_line_at_end(ax1)
    ax1.set_title('CDF of time latency')
    ax1.set_xlabel('latency')
    ax1.set_ylabel("CDF")

    # ax2 = ax[0,1]
    # ax2.hist(np.array(env.stats['ego_v_latency']), bins=100,cumulative=True, density=True, histtype='step',  color='C0',)
    # fix_hist_step_vertical_line_at_end(ax2)
    # ax2.set_title('CDF of ego v time latency')
    # ax2.set_xlabel('latency')
    # ax2.set_ylabel("CDF")
    
    ax2 = ax[0,1]
    ax2.plot(env.repeated_task_num)
    ax2.set_title('average duplicated task number')
    ax2.set_xlabel('time')
    ax2.set_ylabel("duplicated num")  
     
    ax3 = ax[1,0]
    ax3.plot(env.free_vehicle_num)
    ax3.set_title('free vehicle number, average = %s' %(statistics.mean(env.free_vehicle_num[:args.length])))
    ax3.set_xlabel('time')
    ax3.set_ylabel("number")
    
    ax4 = ax[1,1]
    ax4.plot(env.vehicle_tasks_num)
    ax4.set_title('average task number, average = %s' %(statistics.mean(env.vehicle_tasks_num[:args.length])))
    ax4.set_xlabel('time')
    ax4.set_ylabel("tasks number")
    
    # ax4 = ax[1,1]
    # ax4.hist(np.array(env.stats['ego_v_latency']), bins=100,cumulative=True, density=True, histtype='step',  color='C0',)
    # fix_hist_step_vertical_line_at_end(ax4)
    # ax4.set_title('CDF of ego v time latency')
    # ax4.set_xlabel('latency')
    # ax4.set_ylabel("CDF")

    if args.show_figure:
       plt.show()
    
    fig.savefig('ego V density %s and time length %s, mode %s.png'\
                %(args.ego_poisson_density, args.length, args.mode))
    # save_subfig(fig, ax1, 'test CDF of latency on PP density %s.png' %(args.poisson_density))
    print('done')
    np.save("ego_v_latency.npy",env.stats['ego_v_latency'])