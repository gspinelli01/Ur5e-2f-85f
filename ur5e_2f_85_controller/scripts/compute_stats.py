from argparse import ArgumentParser
import glob
import os
from collections import OrderedDict
import json
import numpy as np

RES_DICT = OrderedDict()

if __name__ == '__main__':
    parser = ArgumentParser(
                    prog='Real World test compute Stats')
    parser.add_argument('--task_path', type=str, default="/media/ciccio/Sandisk/test_res/model_save_state_no_finetuned_no_val-65970/pick_place")           
    parser.add_argument('--debug', action='store_true') 
    args = parser.parse_args()
         
    tasks_folder = glob.glob(os.path.join(args.task_path, "task_*"))
    
    
    RES_DICT["overall"] = OrderedDict()
    RES_DICT["overall"]['reached'] = []
    RES_DICT["overall"]['picked'] = []
    RES_DICT["overall"]['success'] = []
    for task_folder in tasks_folder:
        # print(f"Considering task {task_folder.split('/')[-1]}")
        task_id = task_folder.split('/')[-1]
        json_files = glob.glob(os.path.join(task_folder, "*.json"))
        
        RES_DICT[task_id] = OrderedDict()
        RES_DICT[task_id]['reached'] = []
        RES_DICT[task_id]['picked'] = []
        RES_DICT[task_id]['success'] = []
        
        for json_file in json_files:
            with open(json_file, 'r') as file:
                data = json.load(file)
                
            RES_DICT[task_id]['reached'].append(data['reached'])
            RES_DICT[task_id]['picked'].append(data['picked'])
            RES_DICT[task_id]['success'].append(data['success'])
            
            RES_DICT["overall"]['reached'].append(data['reached'])
            RES_DICT["overall"]['picked'].append(data['picked'])
            RES_DICT["overall"]['success'].append(data['success'])
                    
    
        RES_DICT[task_id]['avg_reached'] = np.mean(RES_DICT[task_id]['reached'])
        RES_DICT[task_id]['avg_picked'] = np.mean(RES_DICT[task_id]['picked'])
        RES_DICT[task_id]['avg_success'] = np.mean(RES_DICT[task_id]['success'])
        print(f"Result task {task_id}\n\tAvg reached {RES_DICT[task_id]['avg_reached']}\n\tAvg picked {RES_DICT[task_id]['avg_picked']}\n\tAvg success {RES_DICT[task_id]['avg_success']}")
        
    RES_DICT["overall"]['avg_reached'] = np.mean(RES_DICT["overall"]['reached'])
    RES_DICT["overall"]['avg_picked'] = np.mean(RES_DICT["overall"]['picked'])
    RES_DICT["overall"]['avg_success'] = np.mean(RES_DICT["overall"]['success'])
    print(f"Result task overall\n\tAvg reached {RES_DICT['overall']['avg_reached']}\n\tAvg picked {RES_DICT['overall']['avg_picked']}\n\tAvg success {RES_DICT['overall']['avg_success']}")
        
    
    
