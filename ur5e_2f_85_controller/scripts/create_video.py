import os
import pickle
import cv2
import re
import glob
from PIL import Image
import numpy as np
# img=cv2.imread('/home/ciccio/Pictures/conf_1_v3.png')
# cv2.imshow('Window',img)
# cv2.destroyAllWindows()
import torch
from torchvision.transforms import Normalize
import json
from tqdm import tqdm
from torchvision import transforms
from torchvision.transforms.functional import resized_crop
import multi_task_il
from torchvision.ops import box_iou
from multi_task_il.datasets.savers import Trajectory

STATISTICS_CNTRS = {'reach_correct_obj': 0,
                    'reach_wrong_obj': 0,
                    'pick_correct_obj': 0,
                    'pick_wrong_obj': 0,
                    'pick_correct_obj_correct_place': 0,
                    'pick_correct_obj_wrong_place': 0,
                    'pick_wrong_obj_correct_place': 0,
                    'pick_wrong_obj_wrong_place': 0,
                    }


def find_number(name):
    # return int(re.search(r"\d+", name).group())
    # regex = r'(\d+)_(\d+)'
    regex = r'(\d+)'
    res = re.search(regex, name)
    return res.group()


def sort_key(file_name):
    # Extract the number X from the file name using a regular expression
    pkl_name = file_name.split('/')[-1].split('.')[0]
    match = find_number(pkl_name)
    if match:
        return match
    else:
        return 0  # Return 0 if the file name doesn't contain a number


def sample_command(context):
    demo_t = 4
    selected_frames = list()
    for i in range(demo_t):
        # get first frame
        if i == 0:
            n = 1
        # get the last frame
        elif i == demo_t - 1:
            n = len(context) - 1
        elif i == 1:
            obj_in_hand = 0
            # get the first frame with obj_in_hand and the gripper is closed
            for t in range(1, len(context)):
                state = context.get(t)['info']['status']
                trj_t = context.get(t)
                gripper_act = trj_t['action'][-1]
                if state == 'obj_in_hand' and gripper_act == 1:
                    obj_in_hand = t
                    n = t
                    break
        elif i == 2:
            # get the middle moving frame
            start_moving = 0
            end_moving = 0
            for t in range(obj_in_hand, len(context)):
                state = context.get(t)['info']['status']
                if state == 'moving' and start_moving == 0:
                    start_moving = t
                elif state != 'moving' and start_moving != 0 and end_moving == 0:
                    end_moving = t
                    break
            n = start_moving + int((end_moving-start_moving)/2)
        selected_frames.append(n)

    if isinstance(context, (list, tuple)):
        return [context[i] for i in selected_frames]
    elif isinstance(context, Trajectory):
        return [context[i]['obs'][f"camera_front_image"] for i in selected_frames]


def create_video_for_each_trj(base_path="/", task_name="pick_place"):
    from omegaconf import DictConfig, OmegaConf

    results_folder = f"results_{task_name}"

    # Load config
    config_path = "../src/ai_controller/checkpoint/mosaic_ctod/config.yaml"

    # config_path = "/user/frosa/multi_task_lfd/checkpoint_save_folder/2Task-Pick-Place-Nut-Assembly-Mosaic-100-180-Target-Obj-Detector-BB-Batch50/config.yaml"
    config = OmegaConf.load(config_path)

    # step_pattern = os.path.join(base_path, results_folder, "step-*")
    step_pattern = base_path
    adjust = False if "Real" in base_path else True
    flip_channels = False if "Real" in base_path else True
    for step_path in glob.glob(step_pattern):

        step = step_path.split("-")[-1]
        print(f"---- Step {step} ----")
        context_files = glob.glob(os.path.join(step_path, "context*.pkl"))
        context_files.sort(key=sort_key)
        traj_files = glob.glob(os.path.join(step_path, "traj*.pkl"))
        traj_files.sort(key=sort_key)
        print(context_files)

        try:
            print("Creating folder {}".format(
                os.path.join(step_path, "video")))
            video_path = os.path.join(step_path, "video")
            os.makedirs(video_path)
        except:
            pass
        if len(context_files) != 0:
            for context_file, traj_file in zip(context_files, traj_files):

                # open json file
                try:
                    json_file = traj_file.split('.')[-2]
                    with open(f"{json_file}.json", "rb") as f:
                        traj_result = json.load(f)
                except:
                    pass

                if traj_result.get('success', 0) == 1:
                    STATISTICS_CNTRS['pick_correct_obj_correct_place'] += 1
                if traj_result.get('reached', 0) == 1:
                    STATISTICS_CNTRS['reach_correct_obj'] += 1
                if traj_result.get('picked', 0) == 1:
                    STATISTICS_CNTRS['pick_correct_obj'] += 1

                print(context_file, traj_file)
                with open(context_file, "rb") as f:
                    context_data = pickle.load(f)
                with open(traj_file, "rb") as f:
                    traj_data = pickle.load(f)
                # open json file
                traj_result = None
                try:
                    json_file = traj_file.split('.')[-2]
                    with open(f"{json_file}.json", "rb") as f:
                        traj_result = json.load(f)
                except:
                    pass

                context_frames = torch.Tensor.numpy(context_data)[0]
                context_frames_list = list()
                for indx, frame in enumerate(context_frames):
                    context_frames_list.append(np.array(np.moveaxis(
                        frame*255, 0, -1), dtype=np.uint8))
                context_frames = np.array(context_frames_list)

                traj_frames = []
                bb_frames = []
                gt_bb = []
                activation_map = []
                predicted_conf_score = []
                iou = []
                predicted_bb = False
                for t, step in enumerate(traj_data):

                    if 'camera_front_image' in traj_data.get(t)["obs"].keys():
                        traj_frames.append(step["obs"]['camera_front_image'])
                    else:
                        traj_frames.append(step["obs"]['image'])

                    if 'predicted_bb' in traj_data.get(t)["obs"].keys():
                        predicted_bb = True
                        if isinstance(step["obs"]['predicted_bb'], np.ndarray):
                            bb_frames.append(
                                step["obs"]['predicted_bb'].tolist())
                        else:
                            bb_frames.append(step["obs"]['predicted_bb'])
                        predicted_conf_score.append(
                            step["obs"].get('predicted_score', -1))

                number_of_context_frames = len(context_frames)
                demo_height, demo_width, _ = context_frames[0].shape
                traj_height, traj_width, _ = traj_frames[0].shape

                # Determine the number of columns and rows to create the grid of frames
                num_cols = 2  # Example value, adjust as needed
                num_rows = (number_of_context_frames +
                            num_cols - 1) // num_cols

                # Create the grid of frames
                frames = []
                for i in range(num_rows):
                    row_frames = []
                    for j in range(num_cols):
                        index = i * num_cols + j
                        if index < number_of_context_frames:
                            frame = context_frames[index]
                            # cv2.imwrite(f"context_{index}", context_frames[index])
                            row_frames.append(frame)
                    row = cv2.hconcat(row_frames)
                    frames.append(row)

                new_image = np.array(cv2.resize(cv2.vconcat(
                    frames), (traj_width, traj_height)), np.uint8)

                context_number = find_number(
                    context_file.split('/')[-1].split('.')[0])
                trj_number = find_number(
                    traj_file.split('/')[-1].split('.')[0])
                out = None
                if len(traj_data) >= 3:
                    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                    output_width = 2*traj_width
                    output_height = traj_height
                    # out_path = f"{task_name}_step_{step}_demo_{context_number}_traj_{trj_number}.mp4"
                    out_path = f"demo_{context_number}_traj_{trj_number}.mp4"
                    print(video_path)
                    print(out_path)
                    out = cv2.VideoWriter(os.path.join(
                        video_path, out_path), fourcc, 30, (output_width, output_height))

                else:
                    out_path = os.path.join(
                        video_path, f"demo_{context_number}_traj_{trj_number}.png")

                # create the string to put on each frame
                if traj_result:
                    # res_string = f"Task {traj_result['variation_id']} - Reached {traj_result['reached']} - Picked {traj_result['picked']} - Success {traj_result['success']}"
                    # res_string = f"Reached {traj_result['reached']} - Picked {traj_result['picked']} - Success {traj_result['success']}"
                    # res_string = ""
                    # if predicted_bb:
                    #     res_string = f"Step {step} - Task {traj_result['variation_id']}"
                    res_string = None
                else:
                    res_string = f"Sample index {step}"
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.35
                thickness = 1
                for i, traj_frame in enumerate(traj_frames):
                    # and len(bb_frames) >= i+1:
                    if len(bb_frames) != 0 and i > 0 and len(bb_frames) >= i+1:

                        bb = bb_frames[i -
                                       1]['camera_front'][0].cpu().numpy()

                        traj_frame = np.array(cv2.rectangle(
                            traj_frame,
                            (int(bb[0]),
                             int(bb[1])),
                            (int(bb[2]),
                             int(bb[3])),
                            (0, 0, 255), 1))

                        if len(activation_map) != 0:
                            activation_map_t = activation_map[i-1]

                    output_frame = cv2.hconcat(
                        [new_image, traj_frame])
                    cv2.putText(output_frame,  res_string, (0, 99), font,
                                font_scale, (255, 0, 0), thickness, cv2.LINE_AA)
                    cv2.imwrite("frame.png", output_frame)
                    if out is not None:
                        out.write(output_frame)
                    else:
                        cv2.imwrite(out_path, output_frame)
                if out is not None:
                    out.release()


def read_results(base_path="/", task_name="pick_place"):
    results_folder = f"results_{task_name}"
    # step_pattern = os.path.join(base_path, results_folder, "step-*")
    step_pattern = base_path
    avg_iou = 0
    for step_path in glob.glob(step_pattern):

        step = step_path.split("-")[-1]
        print(f"---- Step {step} ----")
        context_files = glob.glob(os.path.join(step_path, "context*.pkl"))
        context_files.sort(key=sort_key)
        traj_files = glob.glob(os.path.join(step_path, "traj*.pkl"))
        traj_files.sort(key=sort_key)

        try:
            print("Creating folder {}".format(
                os.path.join(step_path, "video")))
            video_path = os.path.join(step_path, "video")
            os.makedirs(video_path)
        except:
            pass

        success_cnt = 0
        reached_cnt = 0
        picked_cnt = 0
        file_cnt = 0
        mean_iou = 0.0
        tp_avg = 0.0
        fp_avg = 0.0
        number_frames = 0.0
        tp = 0.0
        fp = 0.0
        fp_pre_picking = 0.0
        fp_post_picking = 0.0
        fn = 0.0
        fn_pre_picking = 0.0
        fn_post_picking = 0.0
        OPEN_GRIPPER = np.array([-0.02, -0.25, -0.2, -0.02, -0.25, - 0.2])
        for context_file, traj_file in zip(context_files, traj_files):
            print(context_file, traj_file)
            with open(context_file, "rb") as f:
                context_data = pickle.load(f)
            with open(traj_file, "rb") as f:
                traj_data = pickle.load(f)

            try:
                print(len(traj_data))
                for t in range(len(traj_data)):
                    if t != 0:
                        iou = traj_data.get(t)['obs']['iou']
                        number_frames += 1
                        if iou > 0.5:
                            tp += 1
                        else:
                            if traj_data.get(
                                    t)['obs'].get('predicted_bb') is not None:
                                fp += 1
                                # check for gripper open or close
                                gripper_state = traj_data.get(
                                    t)['obs'].get('gripper_qpos')
                                if np.array_equal(OPEN_GRIPPER, np.around(gripper_state, 2)):
                                    fp_pre_picking += 1
                                else:
                                    fp_post_picking += 1

                                bb = adjust_bb(
                                    bb=traj_data.get(
                                        t)['obs']['predicted_bb'][0],
                                    crop_params=None)
                                gt_bb_t = adjust_bb(
                                    bb=traj_data.get(t)['obs']['gt_bb'][0],
                                    crop_params=None)
                                traj_frame = np.array(traj_data.get(
                                    t)['obs']['camera_front_image'])  # [:, :, ::-1])
                                traj_frame = np.array(cv2.rectangle(
                                    traj_frame,
                                    (int(bb[0]),
                                     int(bb[1])),
                                    (int(bb[2]),
                                     int(bb[3])),
                                    (0, 0, 255), 1))
                                traj_frame = np.array(cv2.rectangle(
                                    traj_frame,
                                    (int(gt_bb_t[0]),
                                     int(gt_bb_t[1])),
                                    (int(gt_bb_t[2]),
                                     int(gt_bb_t[3])),
                                    (0, 255, 0), 1))
                                cv2.imwrite(
                                    f"debug/{traj_file.split('/')[-1].split('.')[0]}_{t}.png", traj_frame)
                            else:
                                fn += 1
                                # check for gripper open or close
                                gripper_state = traj_data.get(
                                    t)['obs'].get('gripper_qpos')
                                if np.array_equal(OPEN_GRIPPER, np.around(gripper_state, 2)):
                                    fn_pre_picking += 1
                                else:
                                    fn_post_picking += 1
                                traj_frame = np.array(traj_data.get(
                                    t)['obs']['camera_front_image'][:, :, ::-1])
                                cv2.imwrite(
                                    f"debug/{traj_file.split('/')[-1].split('.')[0]}_{t}.png", traj_frame)
            except:
                pass
            # open json file
            traj_result = None
            try:
                json_file = traj_file.split('.')[-2]
                with open(f"{json_file}.json", "rb") as f:
                    traj_result = json.load(f)
                    file_cnt += 1
            except:
                pass

            if traj_result['success'] == 1:
                success_cnt += 1
            if traj_result['reached'] == 1:
                reached_cnt += 1
            if traj_result['picked'] == 1:
                picked_cnt += 1

            if 'avg_iou' in traj_result.keys():
                mean_iou += traj_result['avg_iou']
                tp_avg += traj_result['avg_tp']

                if traj_result['avg_iou'] < 0.50:
                    fp_avg += traj_result['avg_fp']
                else:
                    tp_avg += traj_result['avg_tp']

            try:
                avg_iou += traj_result['avg_iou']
            except:
                pass

        print(f"Success rate {success_cnt/file_cnt}")
        print(f"Reached rate {reached_cnt/file_cnt}")
        print(f"Picked rate {picked_cnt/file_cnt}")
        print(f"MeanIoU {mean_iou/file_cnt}")
        print(f"MeanFP {fp_avg/file_cnt}")
        print(f"Total Number frames {number_frames}")
        assert number_frames == (
            tp+fp+fn), "Number of frames must be equal to tp+fp+fn"
        print(f"Total Number tp {tp}")
        print(f"Total Number fp {fp}")
        print(f"Total Number fp-pre-picking {fp_pre_picking}")
        print(f"Total Number fp-post-picking {fp_post_picking}")
        print(f"Total Number fn {fn}")
        print(f"Total Number fn-pre-picking {fn_pre_picking}")
        print(f"Total Number fn-post-picking {fn_post_picking}")


if __name__ == '__main__':
    import argparse
    import debugpy
    parser = argparse.ArgumentParser()
    parser.add_argument('--base_path', type=str, default="/",
                        help="Path to checkpoint folder")
    parser.add_argument('--task', type=str,
                        default="pick_place", help="Task name")
    parser.add_argument('--metric', type=str,
                        default="results")
    parser.add_argument('--debug', action='store_true')
    args = parser.parse_args()
    if args.debug:
        debugpy.listen(('0.0.0.0', 5678))
        print("Waiting for debugger attach")
        debugpy.wait_for_client()
    if args.metric != "results":
        # 1. create video
        create_video_for_each_trj(
            base_path=args.base_path, task_name=args.task)
    else:
        import time
        read_results(base_path=args.base_path, task_name=args.task)
        time.sleep(3)
