import hydra
from omegaconf import OmegaConf
import torch
import os
import pickle as pkl
from multi_task_il.datasets.savers import Trajectory
import numpy as np
import copy
import cv2
from torchvision.transforms import ToTensor, Normalize
from torchvision.transforms.functional import resized_crop
from multi_task_il.models.cond_target_obj_detector.utils import project_bboxes
from colorama import Fore, Back, Style
print(f"cv2 file ctod controller {cv2.__file__}")
from multi_task_il.models.command_encoder.cond_module import CondModule
import math
import torchvision

def axisangle2quat(vec):
    """
    Converts scaled axis-angle to quat.

    Args:
        vec (np.array): (ax,ay,az) axis-angle exponential coordinates

    Returns:
        np.array: (x,y,z,w) vec4 float angles
    """
    # Grab angle
    angle = np.linalg.norm(vec)

    # handle zero-rotation case
    if math.isclose(angle, 0.):
        return np.array([0., 0., 0., 1.])

    # make sure that axis is a unit vector
    axis = vec / angle

    q = np.zeros(4)
    q[3] = np.cos(angle / 2.)
    q[:3] = axis * np.sin(angle / 2.)
    return q

def euler2mat(euler):
    """
    Converts euler angles into rotation matrix form

    Args:
        euler (np.array): (r,p,y) angles

    Returns:
        np.array: 3x3 rotation matrix

    Raises:
        AssertionError: [Invalid input shape]
    """

    euler = np.asarray(euler, dtype=np.float64)
    assert euler.shape[-1] == 3, "Invalid shaped euler {}".format(euler)

    ai, aj, ak = -euler[..., 2], -euler[..., 1], -euler[..., 0]
    si, sj, sk = np.sin(ai), np.sin(aj), np.sin(ak)
    ci, cj, ck = np.cos(ai), np.cos(aj), np.cos(ak)
    cc, cs = ci * ck, ci * sk
    sc, ss = si * ck, si * sk

    mat = np.empty(euler.shape[:-1] + (3, 3), dtype=np.float64)
    mat[..., 2, 2] = cj * ck
    mat[..., 2, 1] = sj * sc - cs
    mat[..., 2, 0] = sj * cc + ss
    mat[..., 1, 2] = cj * sk
    mat[..., 1, 1] = sj * ss + cc
    mat[..., 1, 0] = sj * cs - sc
    mat[..., 0, 2] = -sj
    mat[..., 0, 1] = cj * si
    mat[..., 0, 0] = cj * ci
    return mat

def mat2quat(rmat):
    """
    Converts given rotation matrix to quaternion.

    Args:
        rmat (np.array): 3x3 rotation matrix

    Returns:
        np.array: (x,y,z,w) float quaternion angles
    """
    M = np.asarray(rmat).astype(np.float32)[:3, :3]

    m00 = M[0, 0]
    m01 = M[0, 1]
    m02 = M[0, 2]
    m10 = M[1, 0]
    m11 = M[1, 1]
    m12 = M[1, 2]
    m20 = M[2, 0]
    m21 = M[2, 1]
    m22 = M[2, 2]
    # symmetric matrix K
    K = np.array(
        [
            [m00 - m11 - m22, np.float32(0.0), np.float32(0.0), np.float32(0.0)],
            [m01 + m10, m11 - m00 - m22, np.float32(0.0), np.float32(0.0)],
            [m02 + m20, m12 + m21, m22 - m00 - m11, np.float32(0.0)],
            [m21 - m12, m02 - m20, m10 - m01, m00 + m11 + m22],
        ]
    )
    K /= 3.0
    # quaternion is Eigen vector of K that corresponds to largest eigenvalue
    w, V = np.linalg.eigh(K)
    inds = np.array([3, 0, 1, 2])
    q1 = V[inds, np.argmax(w)]
    if q1[0] < 0.0:
        np.negative(q1, q1)
    inds = np.array([1, 2, 3, 0])
    return q1[inds]


class RT1Controller():

    def __init__(self,
                 conf_file_path: str,
                 model_file_path: str,
                 context_path: str,
                 context_robot_name: str,
                 task_name: str,
                 variation_number: int,
                 trj_number: int,
                 camera_name: str) -> None:

        super().__init__()
        # 1. Load configuration file
        self._config = OmegaConf.load(conf_file_path)
        # # modify path for CTOD
        # self._config.policy["target_obj_detector_path"] = os.path.join(
        #     conf_file_path.split("/config")[0], f"../ctod")
        # self._config.policy["model_char"] = conf_file_path.split(
        #     "/")[-1].split("config_")[-1].split('.')[0]
        # 2. Load model
        self._model, self._cond_module = self.load_model_and_conditioner(model_path=model_file_path)
        self._model = self._model.cuda(0)
        self._model.eval()

        for name, module in self._model.named_modules():
            #print(name, type(module))
            if 'dropout' in name and module.training:
                print(module.training)
            if isinstance(module, torchvision.ops.StochasticDepth):
                print("Stochastic Depth")
                if module.training:
                    print("Stochastic depth in training")

        # 3. Get context
        self._context_path = context_path
        self._context_robot_name = context_robot_name
        self._task_name = task_name
        self._variation_number = variation_number
        self._trj_number = trj_number
        self._camera_name = camera_name

        # self._max_z = 0.05  # 5 cm
        # self._prev_action = np.array([0.0, 0.0, 0.0])
        self._t = 0
        print('RT1 controller initialized')

        # self._action_ranges = np.array(
        #     self._config.dataset_cfg.normalization_ranges)
        # print(f"Action ranges {self._action_ranges}")
        # self._context = self._load_context(context_path,
        #                             context_robot_name,
        #                             task_name,
        #                             variation_number,
        #                             trj_number)
        # self.show_context()
        # self.pre_process_context()

    def show_context(self):
        print("Current task to execute")
        img_h = self._context[0].shape[0]
        img_w = self._context[0].shape[1]
        row = 0
        col = 0
        background_color = (255, 255, 255)  # White color
        image = np.full((2 * img_h, 2 * img_w, 3),
                        background_color, dtype=np.uint8)
        for t in range(4):
            row = t // 2
            col = t % 2
            image[row * img_h:(row + 1) * img_h, col *
                  img_w:(col + 1) * img_w, :] = self._context[t][:, :, ::-1]

        cv2.imshow("Image", image)
        cv2.setWindowProperty("Image", cv2.WND_PROP_TOPMOST, 1)
        cv2.waitKey(2000)

    def pre_process_context(self):
        # 4. Pre-process context frames

        # if BGR
        # self._context = [self.pre_process_input(
        #     i[:, :, ::-1])[0][None] for i in self._context] ###############

        # if RGB
        self._context = [self.pre_process_input(
            i)[0][None] for i in self._context]

        if isinstance(self._context[0], np.ndarray):
            self._context = torch.from_numpy(
                np.concatenate(self._context, 0))[None]
        else:
            self._context = torch.cat(self._context, dim=0)[None]

    def modify_context(self, variation_number, trj_number):
        self._context = self._load_context(self._context_path,
                                           self._context_robot_name,
                                           self._task_name,
                                           variation_number,
                                           trj_number)
        self.show_context()
        self.pre_process_context()

    def reset_timer(self):
        self._t = 0

    def _load_context(self, context_path, context_robot_name, task_name, variation_number, trj_number):
        # 1. Load pkl file
        with open(os.path.join(context_path, task_name, f"{context_robot_name}_{task_name}", "task_{0:02d}".format(variation_number), "traj{0:03d}.pkl".format(trj_number)), "rb") as f:
            sample = pkl.load(f)

        traj = sample['traj']

        demo_t = self._config.dataset_cfg.demo_T
        frames = []
        selected_frames = []

        for i in range(demo_t):
            # get first frame
            if i == 0:
                n = 1
            # get the last frame
            elif i == demo_t - 1:
                n = len(traj) - 1
            elif i == 1:
                obj_in_hand = 0
                # get the first frame with obj_in_hand and the gripper is closed
                for t in range(1, len(traj)):
                    state = traj.get(t)['info']['status']
                    trj_t = traj.get(t)
                    gripper_act = trj_t['action'][-1]
                    if state == 'obj_in_hand' and gripper_act == 1:
                        obj_in_hand = t
                        n = t
                        break
            elif i == 2:
                # get the middle moving frame
                start_moving = 0
                end_moving = 0
                for t in range(obj_in_hand, len(traj)):
                    state = traj.get(t)['info']['status']
                    if state == 'moving' and start_moving == 0:
                        start_moving = t
                    elif state != 'moving' and start_moving != 0 and end_moving == 0:
                        end_moving = t
                        break
                n = start_moving + int((end_moving-start_moving)/2)
            selected_frames.append(n)

        if isinstance(traj, (list, tuple)):
            return [traj[i] for i in selected_frames]
        elif isinstance(traj, Trajectory):
            return [traj[i]['obs'][f"{self._camera_name}_image"] for i in selected_frames]
        

    def init_cond_module(self):
        ## loading model
        # cond_module = CondModule(model_name='r2plus1d_18', demo_linear_dim=[512, 512, 512], pretrained=True).to(device)
        cond_module = CondModule(model_name='r2plus1d_18', demo_linear_dim=[512, 512, 512], pretrained=True)


        cond_module_model_path = '/home/gianl/catkin_ws/src/Ur5e-2f-85f/ur5e_2f_85_controller/src/ai_controller/cond_module_ALLBUTDROID_20epochs_RGB_weak_aug-Batch32/model_save-1012.pt'
        try:
            weights = torch.load(cond_module_model_path, weights_only=True)
        except Exception:
            weights = torch.load(cond_module_model_path, weights_only=True, map_location='cuda:0') # this is when you load the cond module on your pc when testing


        cond_module.load_state_dict(weights)
        cond_module = cond_module.eval()

        model_parameters = filter(lambda p: p.requires_grad, cond_module.parameters())
        params = sum([np.prod(p.size()) for p in model_parameters])
        # print(cond_module)
        print('Total params in cond module before freezing:', params)

        # freeze cond module
        for p in cond_module.parameters():
            p.requires_grad = False
            
        model_parameters = filter(lambda p: p.requires_grad, cond_module.parameters())
        params = sum([np.prod(p.size()) for p in model_parameters])
        # print(cond_module)
        print('Total params in cond module after freezing:', params)
        
        return cond_module

    def load_model_and_conditioner(self, model_path=None):

        if model_path:
            # 1. Create the model starting from configuration
            self._config.device = 0 # 'switch to gpu 0'
            model = hydra.utils.instantiate(self._config.policy)
            # 2. Load weights
            weights = torch.load(model_path, map_location=torch.device(0))
            model.load_state_dict(weights)
            # model_weights_print = '/'.join(self._config.policy.cond_module_model_path.split('/')[-2:])

            cond_module = self.init_cond_module().cuda(0)
            # cond_module_weights_print = '/'.join(model_path.split('/')[-2:])
            # print(f'loading model with: \n cond_module: {model_weights_print} \n rt1: {cond_module_weights_print}')

            return model, cond_module
        else:
            raise ValueError("Model path cannot be None")

    def adjust_bb(self, bb, original, cropped_img, obs, img_width=360, img_height=200, top=0, left=0, box_w=360, box_h=200):
        # For each bounding box
        for obj_indx, obj_bb_name in enumerate(bb):
            obj_bb = np.concatenate(
                (bb[obj_bb_name]['bottom_right_corner'], bb[obj_bb_name]['upper_left_corner']))
            # Convert normalized bounding box coordinates to actual coordinates
            x1_old, y1_old, x2_old, y2_old = obj_bb
            x1_old = int(x1_old)
            y1_old = int(y1_old)
            x2_old = int(x2_old)
            y2_old = int(y2_old)

            # Modify bb based on computed resized-crop
            # 1. Take into account crop and resize
            x_scale = obs.shape[1]/cropped_img.shape[1]
            y_scale = obs.shape[0]/cropped_img.shape[0]
            x1 = int(np.round((x1_old - left) * x_scale))
            x2 = int(np.round((x2_old - left) * x_scale))
            y1 = int(np.round((y1_old - top) * y_scale))
            y2 = int(np.round((y2_old - top) * y_scale))

            # image = cv2.rectangle(original,
            #                       (x1_old,
            #                        y1_old),
            #                       (x2_old,
            #                        y2_old),
            #                       color=(0, 0, 255),
            #                       thickness=1)
            # cv2.imwrite("bb_original.png", image)

            # image = cv2.rectangle(cropped_img,
            #                       (int((x1_old - left)),
            #                        int((y1_old - top))),
            #                       (int((x2_old - left)),
            #                        int((y2_old - top))),
            #                       color=(0, 0, 255),
            #                       thickness=1)
            # cv2.imwrite("bb_cropped.png", image)

            # image = cv2.rectangle(obs,
            #                       (x1,
            #                        y1),
            #                       (x2,
            #                        y2),
            #                       color=(0, 0, 255),
            #                       thickness=1)
            # cv2.imwrite("bb_cropped_resize.png", image)

            # replace with new bb
            bb[obj_bb_name]['bottom_right_corner'] = np.array([x2, y2])
            bb[obj_bb_name]['upper_left_corner'] = np.array([x1, y1])
            bb[obj_bb_name]['center'] = np.array(
                [int((x2-x1)/2), int((y2-y1)/2)])
        return bb

    def pre_process_input(self, obs: np.array, bb: np.array = None):
        """Perform preprocess on input image obs

        Args:
            obs (np.array): RGB image
        """

        """applies to every timestep's RGB obs['camera_front_image']"""
        img_height, img_width = obs.shape[:2]
        """applies to every timestep's RGB obs['camera_front_image']"""
        crop_params = self._config.tasks_cfgs[self._task_name].get('demo_crop', [
            0, 0, 0, 0])
        crop_params = [20, 25, 80, 75] #TODO:
        top, left = crop_params[0], crop_params[2]
        img_height, img_width = obs.shape[0], obs.shape[1]
        box_h, box_w = img_height - top - \
            crop_params[1], img_width - left - crop_params[3]

        obs = ToTensor()(obs.copy())
        # ---- Resized crop ----#
        img_res = resized_crop(obs, top=top, left=left, height=box_h,
                               width=box_w, size=(100, 180))
        cv2.imwrite(os.path.join(os.path.dirname(
            os.path.abspath(__file__)), "resize_cropped.png"), np.moveaxis(
            img_res.numpy()*255, 0, -1))
        
        # cv2.imwrite(os.path.join(os.path.dirname(
        #     os.path.abspath(__file__)), "ai_controller_context.png"), np.moveaxis(
        #     ai_controller._context[0][0].numpy()*255, 0, -1))


        adj_bb = None
        # if bb is not None:
        #     adj_bb = self.adjust_bb(bb,
        #                             obs,
        #                             cropped_img,
        #                             img_res,
        #                             img_width=img_width,
        #                             img_height=img_height,
        #                             top=top,
        #                             left=left,
        #                             box_w=box_w,
        #                             box_h=box_h)
        #     cv2.imwrite("cropped.png", obs)
        return img_res, adj_bb

    def pre_process_obs(self, obs: np.array, bb: np.array = None):

        # make RGB
        obs = copy.deepcopy(obs[:,:,::-1])        

        crop_params = self._config.tasks_cfgs[self._task_name].get('agent_crop', [
            0, 0, 0, 0])
        crop_params = [0, 30, 120, 120]

        top, left = crop_params[0], crop_params[2]
        img_height, img_width = obs.shape[0], obs.shape[1]
        box_h, box_w = img_height - top - \
            crop_params[1], img_width - left - crop_params[3]

        # cropped_img = obs[top:box_h, left:box_w]
        # cv2.imwrite(os.path.join(os.path.dirname(
        #     os.path.abspath(__file__)), "cropped.jpg"), cropped_img)

        # img_res = cv2.resize(cropped_img, (180, 100))

        # img_res_scaled = ToTensor()(img_res.copy())

        top, left = crop_params[0], crop_params[2]
        img_height, img_width = obs.shape[0], obs.shape[1]
        box_h, box_w = img_height - top - \
            crop_params[1], img_width - left - crop_params[3]
        
        # bla bla bla

        img_res_scaled = ToTensor()(obs)
        # ---- Resized crop ----#
        img_res_scaled = resized_crop(img_res_scaled, top=top, left=left, height=box_h,
                                      width=box_w, size=(100, 180))

        
        cv2.imwrite(os.path.join(os.path.dirname(
            os.path.abspath(__file__)), "camera_obs_resized.png"), np.array(np.moveaxis(
                copy.deepcopy(img_res_scaled).cpu().numpy()*255, 0, -1), dtype=np.uint8))
        adj_bb = None
        # if bb is not None:
        #     adj_bb = self.adjust_bb(bb,
        #                             obs,
        #                             cropped_img,
        #                             img_res,
        #                             img_width=img_width,
        #                             img_height=img_height,
        #                             top=top,
        #                             left=left,
        #                             box_w=box_w,
        #                             box_h=box_h)
        #     cv2.imwrite("cropped.png", obs)
        
        return img_res_scaled, adj_bb
    

    def _produce_action_vector(self, out: np.array):

        temp_action_dict = out
        temp_action_list = []
        for k in temp_action_dict.keys():
            # print(temp_action_dict[k].shape)
            if temp_action_dict[k].shape[1] != 1:
                temp_action = temp_action_dict[k].squeeze()
            else:
                temp_action = temp_action_dict[k].squeeze(1)
            temp_action_list.append(temp_action)
            
        action = torch.cat(temp_action_list).cpu().numpy()

        return action


    def post_process_output(self, action):
        """Perform post-process on generated output

        Args:
            action (dict): rt1 model output in dict format
        """

        return self._produce_action_vector(action)

    def get_action(self, obs: np.array, robot_state: np.array):
        """_summary_

        Args:
            obs (np.array): _description_
            robot_state (np.array): _description_
        """

        # 1. Pre-process input
        obs, bb = self.pre_process_obs(obs)
        cv2.imwrite(os.path.join(os.path.dirname(
            os.path.abspath(__file__)), "camera_obs.png"), np.array(np.moveaxis(
                copy.deepcopy(obs).cpu().numpy()*255, 0, -1), dtype=np.uint8))
        s_t = torch.from_numpy(robot_state.astype(np.float32))[
            None][None].cuda(0)
        # 2. Run inference
        obs = obs[None][None].cuda(0)
        if bb is not None:
            bb = bb[None].cuda(0)
        elif bb is None:
            bb = torch.from_numpy(
                np.array([[-1, -1, -1, -1]]))[None][None].cuda(0)
            gt_classes = torch.from_numpy(
                np.array([1]))[None][None].cuda(0)
        # 3. Put context on GPU
        context = self._context.float().cuda(0)
        s_t, i_t = s_t.float().cuda(0), obs.float().cuda(0)
        with torch.no_grad():

            if self._t == 0:
                # reset memory at the start of every subtask
                self._model.rt1_memory = None

            ## debug embedding
            # for step_emb in range(context.shape[1]):
            #     cv2.imwrite(f'context_step_{step_emb}.png', np.array(np.moveaxis(context[0][step_emb].detach().cpu().numpy()*255, 0, -1), dtype=np.uint8))

            ######### call cond_module
            embedding = self._cond_module(context)

            # call the forward method
            # RT1 inference (states is not used)
            # i_t = i_t.squeeze(1) # remove batch dimension
            
            ## debug current observation
            # cv2.imwrite(f'observation_0_camera.png', np.array(np.moveaxis(i_t[0][0].detach().cpu().numpy()*255, 0, -1), dtype=np.uint8))

            # image
            out, _ = self._model(images=i_t,
                            states=s_t,
                            cond_embedding=embedding,
                            actions=None,
                            bsize=1,
                            )
            
        # deltas are produced wr the real base link frame
        rt1_action = self.post_process_output(action=out)
        rt1_action[-1] = 0.0 if rt1_action[-1] == 1.0 else 1.0 # invert gripper rt1_action

        converted_action = np.array([0.0]*8)
        converted_action[:3] = rt1_action[:3]
        converted_action[3:-1] = axisangle2quat(rt1_action[3:-1])
        converted_action[-1] = rt1_action[-1]

        cv2.imshow("Image", np.array(np.moveaxis(
                    obs[0][0][:, :, :].cpu().numpy()*255, 0, -1), dtype=np.uint8)[:,:,::-1])
        cv2.setWindowProperty("Image", cv2.WND_PROP_TOPMOST, 1)
        cv2.waitKey(50)

        self._t += 1
        return converted_action, np.ascontiguousarray(np.array(np.moveaxis(
            obs[0][0][:, :, :].cpu().numpy()*255, 0, -1), dtype=np.uint8))

# def _denormalize_action(self, norm_action):
#     action = np.clip(norm_action.copy(), -1, 1)
#     for d in range(self._action_ranges.shape[0]):
#         action[d] = (0.5 * (action[d] + 1) *
#                      (self._action_ranges[d, 1] - self._action_ranges[d, 0])) + self._action_ranges[d, 0]
#     return action



if __name__ == '__main__':
    pass

    # import argparse
    # parser = argparse.ArgumentParser()
    # parser.add_argument('--model_folder', type=str, default=None)
    # parser.add_argument("--model_name", type=str, default=None)
    # parser.add_argument("--context_path", type=str, default=None)
    # parser.add_argument("--task_name", type=str, default=None)
    # parser.add_argument("--context_robot_name", type=str, default=None)
    # parser.add_argument("--variation_number", type=int, default=None)
    # parser.add_argument("--trj_number", type=int, default=None)
    # args = parser.parse_args()

    # import debugpy
    # debugpy.listen(('0.0.0.0', 5678))
    # print("Waiting for debugger attach")
    # debugpy.wait_for_client()

    # # 1. Load Model
    # current_file_path = os.path.dirname(os.path.abspath(__file__))
    # model_folder = args.model_folder
    # model_name = args.model_name
    # conf_file_path = os.path.join(
    #     current_file_path, model_folder, "config.yaml")
    # model_file_path = os.path.join(
    #     current_file_path, model_folder, model_name)
    # # context path
    # context_path = args.context_path
    # context_robot_name = args.context_robot_name
    # task_name = args.task_name
    # variation_number = args.variation_number
    # trj_number = args.trj_number

    # rt1_controller = RT1Controller(
    #     conf_file_path=conf_file_path,
    #     model_file_path=model_file_path,
    #     context_path=context_path,
    #     context_robot_name=context_robot_name,
    #     task_name=task_name,
    #     variation_number=variation_number,
    #     trj_number=trj_number,
    #     camera_name='camera_front')

    # robot_state = state = np.concatenate(
    #     ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [3]))
    # obs = np.zeros((100, 180, 3), dtype=np.uint8)
    # rt1_controller.modify_context(variation_number=0,
    #                                  trj_number=0)
    # rt1_controller.get_action(obs=obs,
    #                              robot_state=robot_state)
