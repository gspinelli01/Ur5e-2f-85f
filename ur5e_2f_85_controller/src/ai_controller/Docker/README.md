```bash
# Build the docker
export GID=$(id -g ciccio)
sudo docker build -t ai-controller --build-arg USER_UID=$UID --build-arg USERNAME=$USER --build-arg USER_GID=$GID .

sudo docker run -it --rm --privileged --name="ai-controller_container" --network="host" --gpus 'all' ai-controller


# Update clock
```bash
sudo date -s "$(wget -qSO- --max-redirect=0 google.com 2>&1 | grep Date: | cut -d' ' -f5-8)Z"
```