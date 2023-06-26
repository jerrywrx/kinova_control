#!/usr/bin/env python3

import subprocess
import GPUtil
from multiprocessing import Pool, Manager
from queue import Empty
import time
import concurrent.futures
from itertools import product

# Define hyperparameters
# Best parameters for push: mse, 0.1, 0.1
# random seed: 4

hyperparameters = {
    'dynamic_staic_edge': [1, 0],
    'auxiliary_gripper_loss': [0, 1],

}

default_parameters = {
    # 'dynamic_staic_edge': 0,
    # 'auxiliary_gripper_loss': 0,
}


# Define other parameters
other_parameters = {
    'skill_name': 'push',
    'stage': 'dy',
    'data_type': 'gt',
    "neighbor_radius": 0.1,
    "tool_neighbor_radius": 0.1,
    'debug': 0,
    'n_his': 1,
    'sequence_length': 1,
    'time_gap': 1,
    'rigid_motion': 1,
    'attn': 0,
    'chamfer_weight': 0.1,
    'emd_weight': 0.9,
    "loss_type": 'mse',
    "check_tool_touching": 0,
    "tool_next_edge": 0,
    "pstep": 2,
    "random_seed": 4,
    'eval': 1,
    "loss_type": 'mse',

}


# Prepare the list of experiments
vary_one_param = False
if vary_one_param:
    experiments = [(hyperparameter_name, hyperparameter_value) 
                for hyperparameter_name, hyperparameter_values in hyperparameters.items() 
                for hyperparameter_value in hyperparameter_values]
else:
    experiments = [dict(zip(hyperparameters.keys(), values)) for values in product(*hyperparameters.values())]

# Assume each job will need 2MB of memory (you should adjust this according to your tasks)
memory_needed = 20000

def run_experiment(job_queue):
    while True:
        try:
            # Block and wait for a new job to become available
            args = job_queue.get(block=True, timeout=1)
        except Empty:
            # If no new job is available, exit the worker function
            break

        # Start with default parameters
        parameters = {**other_parameters, **default_parameters, "skill_name": other_parameters['skill_name']}

        # Modify the hyperparameters
        if isinstance(args, tuple):  # for vary_one_param=True
            hyperparameter_name, hyperparameter_value = args
            parameters[hyperparameter_name] = hyperparameter_value
        else:  # for vary_one_param=False
            parameters.update(args)
            
        # Prepare the command
        command = ['python', 'dynamics/train.py'] + [f'--{k}={v}' for k, v in parameters.items()]

        # Run the command
        subprocess.run(command, shell=False)

        # Mark this job as done so that .join() on the main process knows when all jobs are finished
        job_queue.task_done()


# Use a process pool to run experiments in parallel for each skill name
with Pool() as p, Manager() as manager:
    # Create a shared job queue
    job_queue = manager.Queue()

    # Submit all jobs to the queue
    for experiment in experiments:
        job_queue.put(experiment)

    while not job_queue.empty():
        # Get the first GPU's memory status
        gpu = GPUtil.getGPUs()[0]
        available_memory = gpu.memoryFree

        # If there is enough memory for another job, start it on a new process
        if available_memory >= memory_needed:
            p.apply_async(run_experiment, args=(job_queue,))

        # Sleep for a while to prevent this loop from using too much CPU
        time.sleep(30)

    # Wait for all jobs to finish
    job_queue.join()
