sudo apt-get install  -y curl openssh-server ca-certificates apt-transport-https perl
curl https://packages.gitlab.com/gpg.key | sudo tee /etc/apt/trusted.gpg.d/gitlab.asc

sudo apt-get install -y postfix

sudo curl -sS https://packages.gitlab.com/install/repositories/gitlab/raspberry-pi2/script.deb.sh | sudo bash

sudo EXTERNAL_URL="http://192.168.86.3" apt-get install gitlab-ce


