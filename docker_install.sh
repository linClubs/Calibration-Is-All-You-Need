#-----------1. install docker dependent----------
sudo apt-get update
sudo apt-get --no-install-recommends install -y apt-transport-https ca-certificates curl gnupg-agent software-properties-common


#-----------2. add docker gpg list----------使用中科大的
# curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
curl -fsSL https://mirrors.ustc.edu.cn/docker-ce/linux/ubuntu/gpg | sudo apt-key add -


# ----------3. add agent docker source list-----------使用中科大的
# sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
sudo add-apt-repository "deb [arch=amd64] https://mirrors.ustc.edu.cn/docker-ce/linux/ubuntu $(lsb_release -cs) stable"

#-----------4. update docker source list----------
sudo apt-get update

#-----------5. install docker----------
sudo apt-get --no-install-recommends install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin

# ----------6. add docker domestic image source 国内容器源----------
sudo touch /etc/docker/daemon.json

echo "{
    \"registry-mirrors\":[
        \"https://9cpn8tt6.mirror.aliyuncs.com\",
        \"https://registry.docker-cn.com\"
    ]
}" > /etc/docker/daemon.json


# ----------7. add root permissions----------
sudo groupadd docker
sudo gpasswd -a $USER docker
newgrp docker

# ----------8. restart docker----------
sudo systemctl daemon-reload
sudo systemctl restart docker

# ----------9. Host Visualization----------
sudo apt-get --no-install-recommends install -y x11-xserver-utils
# Host run xhost +