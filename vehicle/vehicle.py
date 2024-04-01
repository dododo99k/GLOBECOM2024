import os, sys, copy
import math, time
import numpy as np
import matplotlib.pyplot as plt
from parameters import *
# from utils import *


class ShareCompVehicle_Server:
    def __init__(self, vehicle_id, time_unit=1, capacity=465, ram=8, speed=40):
        self.vid = vehicle_id
        self.time = 0
        self.time_unit = time_unit
        # the incoming_tasks should a list, each element is a dict of task
        self.task = None
        self.available = True # whether the previous task is done
        # location of this vehicle
        self.location = [0, 0]
        self.rotation = [0,0,0,0]
        np.random.seed(int(time.time()*1000000)%1000)
        self.capacity = capacity * np.clip(1+np.random.randn(), 1-RANDOMS, 1+RANDOMS) # the compute capability of this vehicle random generated
        self.ram = ram # the compute capability of this vehicle
        self.speed = speed # the compute capability of this vehicle


    def add_in_task(self, task):
        assert(self.vid == task.vid) # make sure the task is installed in its vehicle
        self.task = task
        self.task.time_local_compute = 0 # add in task, reset the time local compute, even if it is reseted when it is created
        self.available = False # mark the vehicle's task is onging (not complete)

    def update_loc(self, location=[0,0]):
        assert(len(location) == 2) # make sure the is xy coords
        self.location = location

    def update_rot(self, rotation=[0,0,0,0]):
        assert(len(rotation) == 4) # make sure the is xy coords
        self.rotation = rotation

    def get_utilization(self,):
        if self.task is not None:
            return 1.0
        else:
            return 0.0

    def step(self,):

        self.time += self.time_unit # increase simulation time

        utilization = self.get_utilization()

        capacity = self.capacity

        if self.task is not None: # this vehicle may not have task yet.

            self.task.total_time += self.time_unit # total time increases
            self.task.time_local_compute += self.time_unit # local compute time increases

            self.task.remain_local_compute_size = np.clip(self.task.remain_local_compute_size - self.time_unit * capacity, 0, None) # the remaining time decreases

            if self.task.remain_local_compute_size <= 0:
                done_task = self.task # task is done, no task any more on vehicle
                self.task = None # task is done, no task any more on vehicle
                return done_task, utilization # if this task is done, return task

        return None, utilization # if this task is not done, return False


# ##################################################################################################################

# vehicle_ids = generate_vehicle_ids(NUM_VEHICLES)

# Vehicles = []

# for idx in range(NUM_VEHICLES):
#     Vehicles.append(Vehicle_Server(vehicle_ids[idx])) # initialize the vehicle entity

# completed_tasks = []

# for itime in range(1000):

#     compute_idxs = np.random.randint(0,len(COMPUTATION_TIME), np.random.randint(0,5))

#     tasks = []
#     for idx in range(len(compute_idxs)):
#         tasks.append(initialize_task(vehicle_ids[idx],itime,COMPUTATION_TIME[str(compute_idxs[idx])]))

#     tasks_dequeued = server.run_computation(tasks) # run computation for a time unit

#     completed_tasks += tasks_dequeued

#     print("avg queue length: ",np.mean(server.get_queue_length()))
#     print("dequeued tasks: ", len(tasks_dequeued))

# print("done")
# service_times = [task["used_time"] for task in completed_tasks]
# queuing_times = [task["used_time"] - task["compute_time"] for task in completed_tasks]
# plt.plot(service_times)
# plt.show()
# plt.plot(queuing_times)
# plt.show()









