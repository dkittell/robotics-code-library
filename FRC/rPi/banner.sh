# Switch to root user
su
 
companyname="North Branch Robotics"
 
# Get Hardware Version
RPiVersion=$(cat /proc/device-tree/model)
echo $RPiVersion
 
#lsb_release -a
OS=$(lsb_release -i | cut -d ":" -f2 | tr -d '[:space:]')
OSCode=$(lsb_release -c | cut -d ":" -f2 | tr -d '[:space:]')
OSVer=$(lsb_release -r | cut -d ":" -f2 | tr -d '[:space:]')
echo "$OS $OSCode $OSVer"
 
declare sCPU=$(grep -c ^processor /proc/cpuinfo )
#echo "CPU: $sCPU"
declare sRamGB=$(cat /proc/meminfo | grep MemTotal | cut -d ":" -f 2 |  tr -d '[:space:]' | sed 's/.\{2\}$//'  | awk '{$1=$1/(1024^2); print int($1+0.5),"GB";}')
 
if [ "$sRamGB" == "0 GB" ]; then
sRamGB=$(cat /proc/meminfo | grep MemTotal | cut -d ":" -f 2 |  tr -d '[:space:]' | sed 's/.\{2\}$//' | awk '{ foo = $1 / 1024 ; print foo " MB" }')
fi
 
apt install -y ipcalc
 
echo "Memory (RAM): $sRamGB"
 
# Network Variables - Start
# All Network Adapters
NetworkPorts=$(ip link show | grep '^[a-z0-9]' | awk -F : '{print $2}')
echo $NetworkPorts
 
/sbin/ip -o -4 addr list $netAdapter | awk '{print $4}' | cut -d/ -f1 > /tmp/netIP.txt
netIPFiltered=$(cat /tmp/netIP.txt | sed -n -e 'H;${x;s/\n/,/g;s/^,//;p;}')
echo $netIPFiltered
 
banner=$(echo "$RPiVersion\n")
banner=${banner}$(echo "    OS:              $OS $OSCode $OSVer\n")
banner=${banner}$(echo "    Hostname:        $(hostname)\n")
banner=${banner}$(echo "    CPU:             $sCPU\n")
banner=${banner}$(echo "    Memory (RAM):    $sRamGB\n\n")
banner=${banner}$(echo "Network Information\n")
 
for val in $(echo $NetworkPorts); do   # Get for all available hardware ports their status
echo "Current Interface: $val"
#netActive=$(ifconfig $val | grep "inet.*broadcast" -B1 | grep "Link" | cut -d " " -f1)
netActive=$(ifconfig $val | grep "inet.*broadcast" -B1 | grep "mtu" | cut -d ":" -f1)
#echo $netActive
 
if [ "$netActive" == "$val" ]
then
netIP=$(/sbin/ip -o -4 addr list $val | awk '{print $4}' | cut -d/ -f1|tr '\n' ' ')
netIPv6=$(ifconfig $val | grep 'inet6' | grep 'global'  |awk '{print $2}' | cut -d/ -f1|tr '\n' ' ')
#declare netMask=$(ipcalc -m $netIP | cut -d '=' -f2)
#netMask=$(ifconfig "$netAdapter" | sed -rn '2s/ .*:(.*)$/\1/p') # Debian 8
#netMask=$(ifconfig "$val" | grep "inet.*broadcast" | cut -d " " -f13) # Debian 9
netMask=$(ifconfig "$val" | grep netmask | cut -d ' ' -f13) # Debian 9
 
netCIDR=$(ipcalc $netIP/$val | grep "Netmask:" | cut -d "=" -f2 | cut -d " " -f2 | tr -d '[:space:]')
netWork=$(ipcalc $netIP/$val | grep "Network:" | cut -d "/" -f1 | cut -d " " -f4 | tr -d '[:space:]')
 
banner=${banner}$(echo "    Adapter:         $val\n")
banner=${banner}$(echo "    IP v6:           $netIPv6\n")
banner=${banner}$(echo "    IP v4:           $netIP\n")
banner=${banner}$(echo "    Netmask:         $netMask\n")
banner=${banner}$(echo "    CIDR:            $netWork/$netCIDR\n\n")
fi
done
 
echo -e $banner
#sudo rm /etc/banner
echo -e "$banner"|sudo tee /etc/motd
clear
cat /etc/motd
 
#sudo touch /etc/banner
sudo cp /etc/ssh/sshd_config /etc/ssh/sshd_config.original
sudo sed -i "s|#Banner none|Banner /etc/banner|" /etc/ssh/sshd_config
sudo sed -i "s|#Banner /etc/issue.net|Banner /etc/banner|" /etc/ssh/sshd_config
 
sudo /etc/init.d/ssh restart
 
companyPolicy="$(echo $companyname)\n\r\n\rWARNING\n\rThis computer system is the property of $(echo $companyname). It may be accessed and used only for authorized $(echo $companyname) business by authorized personnel. Unauthorized access or use of this computer system may subject violators to criminal, civil and/or administrative disciplinary action.\n\r\n\r$(echo $companyname) may monitor or log any activity or communication on the system and retrieve any information stored within the system.  By accessing and using this computer, you are consenting to such monitoring and information retrieval for law enforcement and other purposes. All information accessed via this system should be considered confidential unless otherwise indicated. Access or use of this computer system by any person, whether authorized or unauthorized, constitutes consent to these terms. There is no right of privacy in this system.\n\r\n\rNOTE: By logging into this system you indicate your awareness of and consent to these terms and conditions of use. LOG OFF IMMEDIATELY if you do not agree to the conditions stated in this warning.\n"
clear
echo -e "$companyPolicy" |  tee /etc/banner
cat /etc/banner
