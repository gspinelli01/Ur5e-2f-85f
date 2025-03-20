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


class KPController():

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
        # modify path for CTOD
        self._config.policy["target_obj_detector_path"] = os.path.join(
            conf_file_path.split("/config")[0], f"../ctod")
        self._config.policy["model_char"] = conf_file_path.split(
            "/")[-1].split("config_")[-1].split('.')[0]
        # 2. Load model
        self._model = self.load_model(model_path=model_file_path).cuda(0)
        self._model.eval()
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
        self._action_ranges = np.array(
            self._config.dataset_cfg.normalization_ranges)
        print(f"Action ranges {self._action_ranges}")
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

        while True:
            cv2.imshow("Image", image)
            cv2.setWindowProperty("Image", cv2.WND_PROP_TOPMOST, 1)
            key = cv2.waitKey(0)
            if key == 27:
                break
    def pre_process_context(self):
        # 4. Pre-process context frames
        self._context = [self.pre_process_input(
            i[:, :, ::-1])[0][None] for i in self._context]

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

    def load_model(self, model_path=None):

        if model_path:
            # 1. Create the model starting from configuration
            model = hydra.utils.instantiate(self._config.policy)
            # 2. Load weights
            weights = torch.load(model_path, map_location=torch.device(0))
            weights_copy = copy.deepcopy(weights)
            for key in weights_copy.keys():
                if 'object_detector' in key:
                    del weights[key]
            model.load_state_dict(weights, strict=False)
            return model
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
        crop_params = self._config.tasks_cfgs[self._task_name].get('agent_crop', [
            0, 0, 0, 0])

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

    def _denormalize_action(self, norm_action):
        action = np.clip(norm_action.copy(), -1, 1)
        for d in range(self._action_ranges.shape[0]):
            action[d] = (0.5 * (action[d] + 1) *
                         (self._action_ranges[d, 1] - self._action_ranges[d, 0])) + self._action_ranges[d, 0]
        return action

    def post_process_output(self, action: np.array):
        """Perform post-process on generated output

        Args:
            action (np.array): numpy array representing the normalized  action
        """

        denorm_action = self._denormalize_action(action)
        # if self._t != 0:
        #     if abs(self._prev_action[2] - denorm_action[2]) > self._max_z:
        #         # gripper is going down
        #         if self._prev_action[2] > denorm_action[2]:
        #             denorm_action[2] = denorm_action[2] - self._max_z
        #         else:
        #             denorm_action[2] = denorm_action[2] + self._max_z
        #         if action[-1] == 255:
        #             action[-1] = int(0)

        # self._prev_action = denorm_action
        return denorm_action

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
            # torch.save(
            #     i_t, f'/home/ciccio/Desktop/catkin_ws/src/Ur5e-2f-85f/ur5e_2f_85_controller/src/ai_controller/image_obs_msi.pt')
            # torch.save(
            #     context, f'/home/ciccio/Desktop/catkin_ws/src/Ur5e-2f-85f/ur5e_2f_85_controller/src/ai_controller/context_msi.pt')
            # i_t = torch.load(
            #     "/home/ciccio/Desktop/catkin_ws/src/Ur5e-2f-85f/ur5e_2f_85_controller/src/ai_controller/image_obs_quadro.pt")
            # context = torch.load(
            #     "/home/ciccio/Desktop/catkin_ws/src/Ur5e-2f-85f/ur5e_2f_85_controller/src/ai_controller/context_quadro.pt")
            out = self._model(states=s_t,
                              images=i_t,
                              context=context,
                              bb=bb,
                              gt_classes=gt_classes,
                              predict_gt_bb=False,
                              eval=True,
                              target_obj_embedding=None,
                              compute_activation_map=True,
                              t=-1)

            action = out['bc_distrib'].sample()[0, -1].cpu().numpy()

            # Project bb over image
            prediction = out['target_obj_prediction']
            # 1. Get the index with target class
            target_indx_flags = prediction['classes_final'][0] == 1
            place_indx_flags = prediction['classes_final'][0] == 2
            predicted_bb = []
            max_score_target = prediction['conf_scores_final'][0]
            
            if torch.count_nonzero(target_indx_flags) != 0:
                # 2. Get the confidence scores for the target predictions and the the max
                target_max_score_indx = torch.argmax(
                    prediction['conf_scores_final'][0][target_indx_flags])  # prediction['conf_scores_final'][0][target_indx_flags]
            
            if torch.count_nonzero(place_indx_flags) != 0:
                place_max_score_indx = torch.argmax(
                    prediction['conf_scores_final'][0][place_indx_flags])

            image = np.array(np.moveaxis(
                    obs[0][0][:, :, :].cpu().numpy()*255, 0, -1), dtype=np.uint8)
            scale_factor = self._model._object_detector.get_scale_factors()
            projected_bb = project_bboxes(bboxes=prediction['proposals'][0][None][None],
                                              width_scale_factor=scale_factor[0],
                                              height_scale_factor=scale_factor[1],
                                              mode='a2p')[0]
            if torch.count_nonzero(target_indx_flags) != 0:
                
                predicted_bb.append( projected_bb[target_indx_flags][target_max_score_indx])
            
            if torch.count_nonzero(place_indx_flags) != 0:
                predicted_bb.append( projected_bb[place_indx_flags][place_max_score_indx])


            for indx, bb in enumerate(predicted_bb):
                color = (255, 0, 0)
                image = cv2.rectangle(np.ascontiguousarray(image),
                                        (int(bb[0]),
                                        int(bb[1])),
                                        (int(bb[2]),
                                        int(bb[3])),
                                        color=color, thickness=1)
                # image = cv2.putText(image, "Score {:.2f}".format(max_score_target[indx]),
                #                     (int(bb[0]),
                #                     int(bb[1])),
                #                     cv2.FONT_HERSHEY_SIMPLEX,
                #                     0.3,
                #                     (0, 0, 255),
                #                     1,
                #                     cv2.LINE_AA)
                        
            cv2.imwrite(os.path.join(os.path.dirname(
                os.path.abspath(__file__)), "predicted_bb.png"), image)
            cv2.imshow("Image", image)
            cv2.setWindowProperty("Image", cv2.WND_PROP_TOPMOST, 1)
            cv2.waitKey(50)
            

        action = self.post_process_output(action=action)
        self._t += 1
        if action[-1] > 0.5:
            self._model.first_phase = False
        return action, predicted_bb, np.ascontiguousarray(np.array(np.moveaxis(
            obs[0][0][:, :, :].cpu().numpy()*255, 0, -1), dtype=np.uint8))

    def change_phase(self, first_phase: bool = True):
        self._model.first_phase = first_phase

if __name__ == '__main__':

    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--model_folder', type=str, default=None)
    parser.add_argument("--model_name", type=str, default=None)
    parser.add_argument("--context_path", type=str, default=None)
    parser.add_argument("--task_name", type=str, default=None)
    parser.add_argument("--context_robot_name", type=str, default=None)
    parser.add_argument("--variation_number", type=int, default=None)
    parser.add_argument("--trj_number", type=int, default=None)
    args = parser.parse_args()

    import debugpy
    debugpy.listen(('0.0.0.0', 5678))
    print("Waiting for debugger attach")
    debugpy.wait_for_client()

    # 1. Load Model
    current_file_path = os.path.dirname(os.path.abspath(__file__))
    model_folder = args.model_folder
    model_name = args.model_name
    conf_file_path = os.path.join(
        current_file_path, model_folder, "config.yaml")
    model_file_path = os.path.join(
        current_file_path, model_folder, model_name)
    # context path
    context_path = args.context_path
    context_robot_name = args.context_robot_name
    task_name = args.task_name
    variation_number = args.variation_number
    trj_number = args.trj_number

    mosaic_controller = CTODController(
        conf_file_path=conf_file_path,
        model_file_path=model_file_path,
        context_path=context_path,
        context_robot_name=context_robot_name,
        task_name=task_name,
        variation_number=variation_number,
        trj_number=trj_number,
        camera_name='camera_front')

    robot_state = state = np.concatenate(
        ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [3]))
    obs = np.zeros((100, 180, 3), dtype=np.uint8)
    mosaic_controller.modify_context(variation_number=0,
                                     trj_number=0)
    mosaic_controller.get_action(obs=obs,
                                 robot_state=robot_state)
