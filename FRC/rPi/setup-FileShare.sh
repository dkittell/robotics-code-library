sudo apt update
sudo apt upgrade -y
# wget -O - https://raw.githubusercontent.com/OpenMediaVault-Plugin-Developers/installScript/master/install | sudo bash

sudo apt-get install samba samba-common-bin -y

echo -e "[NBR-FRC]\n" | sudo tee -a /etc/samba/smb.conf

sudo reboot 