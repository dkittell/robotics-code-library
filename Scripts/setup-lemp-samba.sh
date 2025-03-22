# First make sure RHEL is properly registered

# Switch to root user
sudo -s

if [ "$(subscription-manager status | grep "Overall Status")" != "Overall Status: Unknown" ]; then
    echo "Properly Registered"
else
    echo -e "\033[32mRed Hat Registration - Start\033[0m"
    if [ -z ${1+x} ]; then
        echo -e "\033[01m\e[4mType your username for RedHat.com, followed by [ENTER]:\e[0m\033[0m"
        read rhUser
    else
        #echo "rhUser is set to '$1'"
        declare rhUser=$1
    fi
    if [ -z ${2+x} ]; then
        echo -e "\033[01m\e[4mType your password for RedHat.com, followed by [ENTER]:\e[0m\033[0m"
        read -s rhPass
    else
        #echo "rhPass is set to '$2'"
        declare rhPass=$2
    fi
    echo -e "\033[32mSet Server Hostname - Start\033[0m"
    if [ -z ${3+x} ]; then
        echo -e "\033[01m\e[4mType your desired hostname for the server, followed by [ENTER]:\e[0m\033[0m"
        read hostname
        sudo hostnamectl set-hostname --static "$hostname"
        sudo hostnamectl set-hostname "$hostname"
        hostnamectl status
    else
        declare hostname=$3
        sudo hostnamectl set-hostname --static "$hostname"
        sudo hostnamectl set-hostname "$hostname"
        hostnamectl status
    fi
    echo -e "\033[32mSet Server Hostname - Stop\033[0m"
    # Register Red Hat Server - Start
    subscription-manager register --username $rhUser --password $rhPass --auto-attach
    subscription-manager refresh
    history -c
    subscription-manager identity
    # Register Red Hat Server - Stop
    echo -e "\033[32mRed Hat Registration - Stop\033[0m"
fi

# Remove nginx if exists
sudo yum remove nginx -y
sudo yum remove php php-mysqlnd php-fpm -y
sudo rm -Rfv /usr/share/nginx/

# Get updates then install some basics
sudo yum update -y
sudo yum install -y nano curl wget unzip yum-utils bash-completion policycoreutils-python-utils mlocate bzip2
sudo yum install -y php-gd php-mbstring php-intl php-pecl-apcu php-mysqlnd php-opcache php-json php-zip ipcalc

companyname="North Branch Robotics"

declare OSVer=$(cat /etc/redhat-release)

declare sCPU=$(grep -c ^processor /proc/cpuinfo)
# echo "CPU: $sCPU"
declare sRamGB=$(cat /proc/meminfo | grep MemTotal | cut -d ":" -f 2 | tr -d '[:space:]' | sed 's/.\{2\}$//' | awk '{$1=$1/(1024^2); print int($1+0.5),"GB";}')

if [ "$sRamGB" == "0 GB" ]; then
    sRamGB=$(cat /proc/meminfo | grep MemTotal | cut -d ":" -f 2 | tr -d '[:space:]' | sed 's/.\{2\}$//' | awk '{ foo = $1 / 1024 ; print foo " MB" }')
fi
echo "Memory (RAM): $sRamGB"

declare netAdapter=$(nmcli device status | grep en | cut -d " " -f1)
if [ -z "$netAdapter" ]; then
    netAdapter=$(nmcli device status | grep eth | cut -d " " -f1)
fi
declare netIP=$(/sbin/ip -o -4 addr list $netAdapter | awk '{print $4}' | cut -d/ -f1)
# declare netIPv6=$(ifconfig $val | grep 'inet6' | grep 'global' | awk '{print $2}' | cut -d/ -f1 | tr '\n' ' ')
#declare netCIDR=$(/sbin/ip -o -4 addr list $netAdapter | cut -d ' ' -f7)
declare netMask=$(ipcalc -m $netIP | cut -d '=' -f2)
declare netCIDR=$(ipcalc -p $netIP $netMask | cut -d '=' -f2)
declare netWork=$(ipcalc -n $netIP $netMask | cut -d '=' -f2)
declare banner=$(
    cat <<EOF
$OSVer
       CPU: $sCPU
    Memory: $sRamGB
  Hostname: $(hostname)
 
Network Information
   Adapter: $netAdapter
     IP v4: $netIP
   Netmask: $netMask
      CIDR: $netWork/$netCIDR
 
 
EOF
)
echo "$banner"
echo -e "$banner" | sudo tee /etc/motd
clear
cat /etc/motd

sudo cp /etc/ssh/sshd_config /etc/ssh/sshd_config.original
sudo sed -i "s|#Banner none|Banner /etc/banner|" /etc/ssh/sshd_config
sudo sed -i "s|#Banner /etc/issue.net|Banner /etc/banner|" /etc/ssh/sshd_config

sudo systemctl restart sshd.service

companyPolicy="${companyname}\n\r\n\rWARNING\n\rThis computer system is the property of ${companyname}. It may be accessed and used only for authorized ${companyname} business by authorized personnel. Unauthorized access or use of this computer system may subject violators to criminal, civil and/or administrative disciplinary action.\n\r\n\r${companyname} may monitor or log any activity or communication on the system and retrieve any information stored within the system.  By accessing and using this computer, you are consenting to such monitoring and information retrieval for law enforcement and other purposes. All information accessed via this system should be considered confidential unless otherwise indicated. Access or use of this computer system by any person, whether authorized or unauthorized, constitutes consent to these terms. There is no right of privacy in this system.\n\r\n\rNOTE: By logging into this system you indicate your awareness of and consent to these terms and conditions of use. LOG OFF IMMEDIATELY if you do not agree to the conditions stated in this warning.\n"
echo -e "${companyPolicy}" | sudo tee /etc/banner

# Install/Verify Apache is installed
sudo yum install httpd -y
sudo systemctl start httpd
sudo systemctl enable httpd

# Install PHP
sudo yum install php php-cli php-common php-gd php-mysqlnd php-pdo -y
# sudo nano /etc/php.ini
sudo sed -i "s|memory_limit = 128M|memory_limit = 256M|" /etc/php.ini
sudo sed -i "s|upload_max_filesize = 2M|upload_max_filesize = 128M|" /etc/php.ini
sudo sed -i "s|post_max_size = 8M|post_max_size = 128M|" /etc/php.ini
echo "<?php phpinfo();" | sudo tee /var/www/html/info.php

sudo systemctl restart httpd

# Install, enable, and configure mariadb
sudo yum install mariadb-server mariadb -y
sudo systemctl start mariadb
sudo systemctl enable mariadb
sudo mysql_secure_installation

sudo firewall-cmd --permanent --zone=public --add-service=http
sudo firewall-cmd --permanent --zone=public --add-service=https
sudo firewall-cmd --permanent --zone=public --add-service=mysql
sudo firewall-cmd --reload

# sudo yum install samba samba-common samba-client -y
# sudo systemctl start smb
# sudo systemctl enable smb
# sudo systemctl start nmb
# sudo systemctl enable nmb
# sudo firewall-cmd --permanent --add-service=samba
# sudo firewall-cmd --reload
# sudo firewall-cmd --list-services

sudo systemctl enable --now cockpit.socket
sudo firewall-cmd --add-service=cockpit --permanent
sudo firewall-cmd --reload

nic=$(nmcli connection show | grep ethernet | cut -d ' ' -f1 | sed 's/^[ \t]*//;s/[ \t]*$//')
sudo nmcli con modify "${nic}" ifname ${nic} ipv4.method manual ipv4.addresses 10.87.67.10/24 gw4 10.87.67.1
sudo nmcli con modify "${nic}" ipv4.dns 172.16.1.7
sudo nmcli con down "${nic}"
sudo nmcli con up "${nic}"

wget -O nextcloud.zip https://download.nextcloud.com/server/releases/latest.zip
sudo unzip nextcloud.zip -d /var/www/html
sudo mkdir /var/www/html/nextcloud/data
sudo chown -R apache:apache /var/www/html/nextcloud
sudo systemctl restart httpd
sudo firewall-cmd --permanent --add-port=80/tcp
sudo firewall-cmd --reload

sudo semanage fcontext -a -t httpd_sys_rw_content_t '/var/www/html/nextcloud/data(/.*)?'
sudo semanage fcontext -a -t httpd_sys_rw_content_t '/var/www/html/nextcloud/config(/.*)?'
sudo semanage fcontext -a -t httpd_sys_rw_content_t '/var/www/html/nextcloud/apps(/.*)?'
sudo semanage fcontext -a -t httpd_sys_rw_content_t '/var/www/html/nextcloud/.htaccess'
sudo semanage fcontext -a -t httpd_sys_rw_content_t '/var/www/html/nextcloud/.user.ini'
sudo semanage fcontext -a -t httpd_sys_rw_content_t '/var/www/html/nextcloud/3rdparty/aws/aws-sdk-php/src/data/logs(/.*)?'

sudo restorecon -R '/var/www/html/nextcloud/'
sudo setsebool -P httpd_can_network_connect on

sudo mysql -u root -p
CREATE USER 'nextcloud'@'localhost' IDENTIFIED BY '3%2F@b0#L2iaBhw9d!4AF@';
CREATE DATABASE IF NOT EXISTS nextcloud CHARACTER SET utf8mb4 COLLATE utf8mb4_general_ci;
GRANT ALL PRIVILEGES on nextcloud.* to 'nextcloud'@'localhost';
FLUSH privileges;
quit;

