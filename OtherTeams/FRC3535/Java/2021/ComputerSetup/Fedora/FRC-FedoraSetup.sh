# https://gist.github.com/cha55son/6042560

# sudo systemctl enable sshd && sudo systemctl start sshd

sudo dnf update -y

sudo dnf swap -y @gnome-desktop @kde-desktop

sudo dnf install -y switchdesk
sudo switchdesk kde

sudo dnf install -y net-tools bind-utils nano wget tcl unzip telnet
sudo timedatectl set-timezone America/Detroit

# sudo shutdown -h now # Shutdown
# sudo shutdown -r now # Restart

sudo adduser mentor
sudo passwd mentor

sudo usermod -aG wheel mentor
groups frc3535

# sudo cat /etc/ssh/sshd_config | grep PrintMotd
sudo sed -i "s|#PrintMotd yes|PrintMotd no|" /etc/ssh/sshd_config && cat /etc/ssh/sshd_config | grep PrintMotd
# sudo sed -i "s|PrintMotd yes|PrintMotd no|" /etc/ssh/sshd_config && cat /etc/ssh/sshd_config | grep PrintMotd
sudo sed -i "s|#Banner none|Banner /etc/login.warn|" /etc/ssh/sshd_config && cat /etc/ssh/sshd_config | grep Banner
sudo sed -i "s|#UsePAM no|UsePAM yes|" /etc/ssh/sshd_config && cat /etc/ssh/sshd_config | grep UsePAM
echo "session optional pam_motd.so motd=/run/motd.dynamic" | sudo tee -a /etc/pam.d/sshd
echo "session optional pam_motd.so" | sudo tee -a /etc/pam.d/login
echo "//etc/update-motd.d/10daily-motd" | sudo tee -a /etc/profile
sudo mkdir -p /etc/update-motd.d

clear
sudo cp /etc/selinux/config /etc/selinux/config.bk
sudo sed -i '/^#/d' /etc/selinux/config
sudo sed -i '/^$/d' /etc/selinux/config

SELinuxStatus=$(cat /etc/selinux/config | grep SELINUX= | cut -d'=' -f2)
echo Current Status: $SELinuxStatus

clear
# SELinux - Permissive
sudo sed -i "s|SELINUX=$SELinuxStatus|SELINUX=permissive|" /etc/selinux/config
SELinuxStatus=$(cat /etc/selinux/config | grep SELINUX= | cut -d'=' -f2)
echo New Status: $SELinuxStatus

# sudo visudo # Uncomment the wheel group

# Switch to root user
# sudo -s

sudo rm -f /etc/motd
echo -e "\033[01m\e[4mType your desired hostname for the server, followed by [ENTER]:\e[0m\033[0m"
read hostname
sudo hostnamectl set-hostname --static "$hostname"
sudo hostnamectl set-hostname "$hostname"
hostnamectl status

companyname="FRC3535"

# sudo yum install https://github.com/gdubicki/centos-pam-with-update-motd/releases/download/1.1.8-1023.1/pam-1.1.8-1023.el7.x86_64.rpm --skip-broken

cat /etc/update-motd.d/10daily-motd
sudo chmod -c +x /etc/update-motd.d/*

sudo chmod -c +x /home/frc3535/Desktop/FRC\ VS\ Code\ 2021.desktop

companyPolicy="
     ##          ##
       ##      ##
     ##############       FRC3535 GalakTech Invaders
   ####  ######  ####     Programming Computer
 ######################   $(hostname)
 ##  ##############  ##   $(cat /etc/fedora-release)
 ##  ##          ##  ##
       ####  ####         Authorized Use Only - All use is randomly monitored

$(echo $companyname)\n\r\n\rWARNING\n\rThis computer system is the property of $(echo $companyname). It may be accessed and used only for authorized $(echo $companyname) business by authorized personnel. Unauthorized access or use of this computer system may subject violators to criminal, civil and/or administrative disciplinary action.\n\r\n\r$(echo $companyname) may monitor or log any activity or communication on the system and retrieve any information stored within the system.  By accessing and using this computer, you are consenting to such monitoring and information retrieval for law enforcement and other purposes. All information accessed via this system should be considered confidential unless otherwise indicated. Access or use of this computer system by any person, whether authorized or unauthorized, constitutes consent to these terms. There is no right of privacy in this system.\n\r\n\rNOTE: By logging into this system you indicate your awareness of and consent to these terms and conditions of use. LOG OFF IMMEDIATELY if you do not agree to the conditions stated in this warning.\n"
clear
echo -e "$companyPolicy" | sudo tee /etc/login.warn

sudo systemctl restart sshd.service

sudo gpasswd -d frc3535 wheel

sudo dnf install -y java-latest-openjdk.x86_64

cd ~/Downloads
wget https://github.com/wpilibsuite/allwpilib/releases/download/v2021.3.1/WPILib_Linux-2021.3.1.tar.gz
tar -xf WPILib_Linux-*.tar.gz
cd WPILib_Linux-*/
./WPILibInstaller
