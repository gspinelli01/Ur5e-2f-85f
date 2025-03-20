#!/bin/bash

model_folder='/home/gianl/catkin_ws/src/Ur5e-2f-85f/ur5e_2f_85_controller/src/ai_controller/checkpoint/RT1__pick_place__real__90_epochs__5e-4_lr__bs_16__fixed_orig_range-Batch32'
model_name='model_save-210600.pt'
context_path='/home/gianl/catkin_ws/src/Ur5e-2f-85f/ur5e_2f_85_controller/src/ai_controller/context'
task_name='pick_place'
context_robot_name='panda'
variation_number='0'
trj_number='0'

python3 RT1_controller.py --model_folder ${model_folder} \
                          --model_name ${model_name} \
                          --context_path ${context_path} \
                          --task_name ${task_name} \
                          --context_robot_name ${context_robot_name} \
                          --variation_number ${variation_number} \
                          --trj_number ${trj_number} \