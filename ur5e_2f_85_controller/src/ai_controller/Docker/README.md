```bash
# Build the docker
export GID=$(id -g ciccio) && sudo docker build -t ai-controller --build-arg USER_UID=$UID --build-arg USERNAME=$USER --build-arg USER_GID=$GID --build-arg GIT_PERSONAL_TOKEN=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 .

sudo docker run -it --rm --privileged --name="ai-controller_container" --network="host" --runtime nvidia -v /dev:/dev -v /home/ciccio/Desktop/catkin_ws/src/Ur5e-2f-85f:/catkin_ws/src/Ur5e-2f-85f  -v /media/ciccio/Sandisk/multitask_dataset_ur/multitask_dataset_language_command:/media/ciccio/Sandisk/multitask_dataset_ur/multitask_dataset_language_command -p 5678:5678 -e NVIDIA_DRIVER_CAPABILITIES=all ai-controller

# Update clock
```bash
sudo date -s "$(wget -qSO- --max-redirect=0 google.com 2>&1 | grep Date: | cut -d' ' -f5-8)Z"
```