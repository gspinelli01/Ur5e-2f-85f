import torch
import numpy as np
import cv2


def load_tensors(file_paths):
    tensors = []
    for file_path in file_paths:
        try:
            tensor = torch.load(file_path)
            tensors.append(tensor)
        except Exception as e:
            print(f"Error loading tensor from {file_path}: {e}")
    return tensors


def compute_tensor_differences(tensors):
    differences = []
    for i in range(len(tensors) - 1):
        diff = tensors[i] - tensors[i+1]
        differences.append(diff)
    return differences


if __name__ == "__main__":
    file_paths = ["/home/ciccio/Desktop/catkin_ws/src/Ur5e-2f-85f/ur5e_2f_85_controller/src/ai_controller/image_obs_msi.pt",
                  "/home/ciccio/Desktop/catkin_ws/src/Ur5e-2f-85f/ur5e_2f_85_controller/src/ai_controller/image_obs_quadro.pt",
                  "/home/ciccio/Desktop/catkin_ws/src/Ur5e-2f-85f/ur5e_2f_85_controller/src/ai_controller/context_msi.pt",
                  "/home/ciccio/Desktop/catkin_ws/src/Ur5e-2f-85f/ur5e_2f_85_controller/src/ai_controller/context_quadro.pt"]
    loaded_tensors = load_tensors(file_paths)

    print(loaded_tensors)

    image_obs_msi = loaded_tensors[0]
    image_obs_quadro = loaded_tensors[1]

    print(image_obs_msi-image_obs_quadro)
    print(torch.count_nonzero(image_obs_msi-image_obs_quadro))

    context_msi = loaded_tensors[2][0]
    context_quadro = loaded_tensors[3][0]

    print(context_msi.shape)
    print(context_quadro.shape)
    for i in range(context_quadro.shape[0]):
        context_msi_i = context_msi[i]
        context_quadro_i = context_quadro[i]
        context_msi_i = np.array(np.moveaxis(
            context_msi_i[:, :, :].cpu().numpy()*255, 0, -1), dtype=np.uint8)
        context_quadro_i = np.array(np.moveaxis(
            context_quadro_i[:, :, :].cpu().numpy()*255, 0, -1), dtype=np.uint8)
        cv2.imshow(f"context_msi_{i}", context_msi_i)
        cv2.imshow(f"context_quadro_{i}", context_quadro_i)
        cv2.imshow(f"difference_{i}", context_msi_i-context_quadro_i)

        cv2.waitKey(0)
        cv2.destroyAllWindows()
    print(torch.count_nonzero(context_msi-context_quadro))
