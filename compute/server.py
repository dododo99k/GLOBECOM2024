from ast import Assert
import os, sys
# sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))) # this finds parent dir
import math
import numpy as np
import queue
import matplotlib.pyplot as plt
from parameters import *

class MultiQueueServer:
    def __init__(self, num_of_server=10, capacity=1.0, time_unit=1, ):
        self.num_of_server = num_of_server
        self.capacity = capacity
        # initialize queue of this server
        self.service_queues = []
        for _ in range(num_of_server):
            self.service_queues.append(queue.Queue()) # basic FIFO queue
        
        self.time = 0
        self.time_unit = time_unit

    # def run_server_computation(self, incoming_tasks=None):
    #     tasks_dequeued = [] 
    #     # the incoming_tasks should a list, each element is a dict of task

    #     if incoming_tasks not None:

    #         # add the incoming tasks into the queues
    #         self.enqueue_tasks(incoming_tasks)

    #         self.time += self.time_unit # increase simulation time

    #         # dequeue tasks if they completed (front task in queue)
    #         tasks_dequeued = self.dequeue_tasks()

    #     return tasks_dequeued
    def get_utilization(self,):
        occupation = [True for service_queue in self.service_queues if service_queue.qsize()>0]
        utilization = np.sum(occupation) / self.num_of_server
        return utilization

    def step(self,):

        self.time += self.time_unit # increase simulation time

        utilization = self.get_utilization()

        # dequeue tasks if they completed (front task in queue)
        tasks_dequeued = self.dequeue_tasks()

        return tasks_dequeued, utilization

    def get_queue_length(self,):
        length = [service_queue.qsize() for service_queue in self.service_queues]
        return np.array(length)

    def enqueue_tasks(self, tasks):
        if not isinstance(tasks, list):
            raise ValueError("incoming tasks should be a list!") 

        # the scheduling of enqueuing is RR
        while len(tasks) > 0:
            task = tasks.pop() # pop out the first task
            # scale the compute time of this task according to the current server capacity
            task.remain_time_server = task.remain_time_server/self.capacity
            task.time_server_compute = task.remain_time_server # update real computing time at server
            curr_queue_len = [queue.qsize() for queue in self.service_queues] # get current queue load
            sche_idx = np.argmin(curr_queue_len) # find the minimum load queue, i.e., min-load scheduling 
            self.service_queues[sche_idx].put_nowait(task) # put the task into a queue
        
        return True


    def dequeue_tasks(self,):

        tasks_dequeued = []
        # dequeue the task if the task is completed, based on the compute time
        for service_queue in self.service_queues:
            # change the values in the queue without get()/popout
            if service_queue.qsize() > 0: 
                # decrease remianing time for the front task
                service_queue.queue[0].total_time += self.time_unit # add time unit every time for total
                service_queue.queue[0].remain_time_server = np.clip(service_queue.queue[0].remain_time_server - self.time_unit, 0, None)

                for idx in range(1, service_queue.qsize()): # increase queue time and total time for the other queued tasks
                    service_queue.queue[idx].total_time += self.time_unit # add time unit every time for total
                    service_queue.queue[idx].time_server_queue += self.time_unit # add time unit every time for queue             

                # pop out the front task if it is completed
                if service_queue.queue[0].remain_time_server <= 0:
                    front_task = service_queue.get_nowait() # get the front task out
                    # front_task.end_time = self.time  # get the current time for end time
                    tasks_dequeued.append(front_task) # append the task
            else:
                pass # no tasks in this queue

        return tasks_dequeued


class ShareCompServer:
    def __init__(self, capacity=6450, time_unit=1, ):
        self.capacity = capacity
        self.users = [] # not actually queue, but just accommodate users
        self.time = 0
        self.time_unit = time_unit
        self.last_computed_size = 0
        self.historical_num_of_users = []

    def get_utilization(self,):
        return 1 if len(self.users)>0 else 0

    def step(self,):

        self.time += self.time_unit # increase simulation time

        utilization = self.get_utilization()

        # dequeue tasks if they completed (front task in queue)
        tasks_dequeued = self.dequeue_tasks()

        return tasks_dequeued, utilization

    def get_num_of_users(self,):
        return len(self.users)

    def enqueue_tasks(self, tasks):
        if not isinstance(tasks, list):
            raise ValueError("incoming tasks should be a list!") 
        else:
            self.users += tasks

        return True

    def dequeue_tasks(self,):

        tasks_dequeued = []
        num_of_active_users = len(self.users)
        
        average_capacity = self.capacity / num_of_active_users if num_of_active_users>0 else self.capacity
        average_capacity = average_capacity * np.clip(1+np.random.randn(), 1-RANDOMS, 1+RANDOMS)
        self.last_computed_size = self.time_unit * average_capacity # this applies to all users/tasks in this edge server
        self.historical_num_of_users.append(self.get_num_of_users())

        # dequeue the task if the task is completed, based on remaining compute size
        for task in self.users: # increase compute time and total time for all tasks
            task.total_time += self.time_unit # add time unit every time for total
            task.time_server_compute += self.time_unit # add time unit every time for server processing
            task.remain_server_compute_size = np.clip(task.remain_server_compute_size - self.time_unit * average_capacity, 0, None)
            # task.last_edge_computed_size = self.time_unit * average_capacity #  add last computed size
            # remove the task
            if task.remain_server_compute_size <= 0:
                tasks_dequeued.append(task) # append the task for task completion
                self.users.remove(task) # remove it 

        return tasks_dequeued


