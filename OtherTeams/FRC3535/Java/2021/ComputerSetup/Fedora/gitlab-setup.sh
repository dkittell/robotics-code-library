sudo dnf install -y curl policycoreutils openssh-server perl
# Enable OpenSSH server daemon if not enabled: sudo systemctl status sshd
sudo systemctl enable sshd
sudo systemctl start sshd

# Check if opening the firewall is needed with: sudo systemctl status firewalld
sudo firewall-cmd --permanent --add-service=http
sudo firewall-cmd --permanent --add-service=https
sudo systemctl reload firewalld

sudo dnf install postfix
sudo systemctl enable postfix
sudo systemctl start postfix

# curl https://packages.gitlab.com/install/repositories/gitlab/gitlab-ee/script.rpm.sh | sudo bash
curl -sS https://packages.gitlab.com/install/repositories/gitlab/gitlab-ce/script.rpm.sh | sudo bash

# sudo EXTERNAL_URL="https://gitlab.example.com" dnf install -y gitlab-ee
# sudo EXTERNAL_URL="https://gitlab.example.com" dnf remove -y gitlab-ee
sudo EXTERNAL_URL="https://rhel-parallels.local" dnf install -y gitlab-ce
# sudo EXTERNAL_URL="https://rhel-parallels.local" dnf remove -y gitlab-ce

sudo cat /etc/gitlab/initial_root_password
# 10.211.55.7

# sudo gitlab-ctl restart
# sudo gitlab-ctl reconfigure

# sudo sed -i 's|external_url "https://rhel-parallels.local"|external_url "https://10.211.55.7"|' /etc/gitlab/gitlab.rb
