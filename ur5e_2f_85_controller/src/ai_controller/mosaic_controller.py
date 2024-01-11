from ai_controller.model_controller import ModelControllerInterface
import hydra
from omegaconf import DictConfig, OmegaConf
import torch
import os
import multi_task_il
import pickle as pkl
from multi_task_il.datasets.savers import Trajectory
import numpy as np
import copy
from torchvision.transforms import ToTensor, Normalize
import cv2


class MosaicController():

    ACTION_RANGES = np.array([[-0.35, 0.35],
                              [0.2, 0.9],
                              [-0.1, 0.2],
                              [-3.14911766, 3.14911766],
                              [-3.14911766, 3.14911766],
                              [-3.14911766, 3.14911766]])

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
        # 2. Load model
        self._model = self.load_model(model_path=model_file_path).cuda(0)
        # 3. Get context
        self._context_path = context_path
        self._context_robot_name = context_robot_name
        self._task_name = task_name
        self._variation_number = variation_number
        self._trj_number = trj_number
        self._camera_name = camera_name
        self._context = self._load_context(context_path,
                                           context_robot_name,
                                           task_name,
                                           variation_number,
                                           trj_number)

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

        cv2.imshow("command", image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        # 4. Pre-process context frames
        self._context = [self.pre_process_input(
            i[:, :, ::-1]/255)[None] for i in self._context]

        if isinstance(self._context[0], np.ndarray):
            self._context = torch.from_numpy(
                np.concatenate(self._context, 0))[None]
        else:
            self._context = torch.cat(self._context, dim=0)[None]

        self._t = 0

    def modify_context(trj_number):
        pass

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
                n = 0
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
            weights = torch.load(model_path, map_location=torch.device('cpu'))
            model.load_state_dict(weights)
            return model
        else:
            raise ValueError("Model path cannot be None")

    def pre_process_input(self, obs: np.array):
        """Perform preprocess on input image obs

        Args:
            obs (np.array): RGB image
        """
        crop_params = self._config.tasks_cfgs[self._task_name].get('crop', [
            0, 0, 0, 0])
        top, left = crop_params[0], crop_params[2]
        img_height, img_width = obs.shape[0], obs.shape[1]
        box_h, box_w = img_height - top - \
            crop_params[1], img_width - left - crop_params[3]

        cropped_img = obs[top:box_h, left:box_w]
        cv2.imwrite(os.path.join(os.path.dirname(
            os.path.abspath(__file__)), "cropped.png"), cropped_img)

        img_res = cv2.resize(cropped_img, (180, 100))
        cv2.imwrite(os.path.join(os.path.dirname(
            os.path.abspath(__file__)), "resized.png"), img_res)

        img_res = ToTensor()(copy.copy(img_res))
        return img_res

    def _denormalize_action(self, action):
        action = np.clip(action.copy(), -1, 1)
        for d in range(MosaicController.ACTION_RANGES.shape[0]):
            action[d] = (0.5 * (action[d] + 1) *
                         (MosaicController.ACTION_RANGES[d, 1] - MosaicController.ACTION_RANGES[d, 0])) + MosaicController.ACTION_RANGES[d, 0]
        return action

    def post_process_output(self, action: np.array):
        """Perform post-process on generated output

        Args:
            action (np.array): numpy array representing the normalized  action
        """

        return self._denormalize_action(action)

    def get_action(self, obs: np.array, robot_state: np.array):
        """_summary_

        Args:
            obs (np.array): _description_
            robot_state (np.array): _description_
        """

        # 1. Pre-process input
        # 1. Pre-process input
        obs = self.pre_process_input(obs)
        s_t = torch.from_numpy(robot_state.astype(np.float32))[
            None][None].cuda(0)
        # 2. Run inference
        obs = obs[None][None].cuda(0)
        # 3. Put context on GPU
        context = self._context.cuda(0)
        with torch.no_grad():
            out = self._model(context=context,
                              images=obs,
                              states=s_t,
                              eval=True,
                              )

            action = out['bc_distrib'].sample()[0, -1].cpu().numpy()

        action = self.post_process_output(action=action)
        self._t += 1
        image = np.array(np.moveaxis(
            obs[0][0][:, :, :].cpu().numpy()*255, 0, -1), dtype=np.uint8)
        return action, image


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

    mosaic_controller = MosaicController(
        conf_file_path=conf_file_path,
        model_file_path=model_file_path,
        context_path=context_path,
        context_robot_name=context_robot_name,
        task_name=task_name,
        variation_number=variation_number,
        trj_number=trj_number,
        camera_name='camera_front')
