```bash
# Build the docker
export GID=$(id -g mivia)
sudo docker build --ssh default=C:/Users/franc/.ssh/id_rsa -t jetson_xavier_nx --build-arg USER_UID=$UID --build-arg USERNAME=$USER --build-arg USER_GID=$GID --build-arg GIT_PERSONAL_TOKEN=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 .

# Run the docker 
docker run -rm jetson_xavier_nx
```