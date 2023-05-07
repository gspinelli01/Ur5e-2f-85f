from ai_controller.model_controller import ModelControllerInterface
import hydra
from omegaconf import DictConfig, OmegaConf
import torch
import os
import mosaic
import pickle as pkl
from mosaic.datasets.savers import Trajectory
import numpy as np
import copy
from torchvision.transforms import ToTensor, Normalize


class MosaicController(ModelControllerInterface):

    ACTION_RANGES = [[-0.35,  0.25],
                     [-0.30,  0.30],
                     [0.60,  1.20],
                     [-3.14,  3.14911766],
                     [-3.14911766, 3.14911766],
                     [-3.14911766,  3.14911766]]

    def __init__(self,
                 conf_file_path: str,
                 model_file_path: str,
                 context_path: str,
                 context_robot_name: str,
                 task_name: str,
                 variation_number: int,
                 trj_number: int) -> None:

        super().__init__()
        # 1. Load configuration file
        self._config = OmegaConf.load(conf_file_path)
        # 2. Load model
        self._model = self.load_model(model_path=model_file_path)
        # 3. Get context
        self._context_path = context_path
        self._context_robot_name = context_robot_name
        self._task_name = task_name
        self._variation_number = variation_number
        self._trj_number = trj_number
        self._context = self._load_context(context_path,
                                           context_robot_name,
                                           task_name,
                                           variation_number,
                                           trj_number)
        # 4. Pre-process context frames
        self._context = [[self.pre_process_input(
            i[:, :, ::-1]/255)[None].float().cuda(0) for i in self._context]]

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
            return [traj[i]['obs']['image'] for i in selected_frames]

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
        # 1. Normalize image between 0,1
        obs = copy.deepcopy(obs)/255
        # 2. Perform Normalization
        obs = ToTensor()(obs.copy())

        obs = Normalize(mean=[0.485, 0.456, 0.406],
                        std=[0.229, 0.224, 0.225])(obs)
        return obs

    def _denormalize_action(action):
        action = np.clip(action.copy(), -1, 1)
        for d in range(action_ranges.shape[0]):
            action[d] = (0.5 * (action[d] + 1) *
                         (action_ranges[d, 1] - action_ranges[d, 0])) + action_ranges[d, 0]

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
        obs = self.pre_process_input(obs)[None].float().cuda(0)
        s_t = torch.from_numpy(robot_state.astype(np.float32))[None]
        # 2. Run inference
        with torch.no_grad():
            out = self._model(states=s_t, images=obs, context=self._context, eval=True,
                              target_obj_embedding=target_obj_embedding)  # to avoid computing ATC loss
            try:
                target_obj_embedding = out['target_obj_embedding']
            except:
                pass

            action = out['bc_distrib'].sample()[0, -1].cpu().numpy()

        return self.post_process_output(action)


if __name__ == '__main__':

    import debugpy
    debugpy.listen(('0.0.0.0', 5678))
    print("Waiting for debugger attach")
    debugpy.wait_for_client()

    current_file_path = os.path.abspath(os.getcwd())
    model_folder = "checkpoint/mosaic"
    model_name = "model_save-29348.pt"

    conf_file_path = os.path.join(
        current_file_path, model_folder, "config.yaml")
    model_file_path = os.path.join(
        current_file_path, model_folder, model_name)

    # context path
    context_path = "/media/ciccio/Sandisk/multitask_dataset_baseline"
    context_robot_name = "sawyer"
    task_name = "pick_place"
    variation_number = 0
    trj_number = 0

    mosaic_controller = MosaicController(
        conf_file_path=conf_file_path,
        model_file_path=model_file_path,
        context_path=context_path,
        context_robot_name=context_robot_name,
        task_name=task_name,
        variation_number=variation_number,
        trj_number=trj_number)
