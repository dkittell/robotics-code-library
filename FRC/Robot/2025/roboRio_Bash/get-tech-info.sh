#!/bin/bash

# NOTE: This script is to run on the roboRIO while connected via SSH

#region roboRIO
# roboRIO - Basic Connection
# ssh admin@10.87.67.2

# Send File From VM to roboRIO
# scp get-tech-info.sh admin@10.87.67.2:/home/lvuser/

# Run File From roboRIO
# bash /home/lvuser/get-tech-info.sh > /home/lvuser/rio_info.txt

# Get File From roboRIO to VM
# scp admin@10.87.67.2:/home/lvuser/rio_info.txt .
#endregion roboRIO

OSID=$(cat /etc/*release | grep -i id | grep -vi "_id\|id_" | cut -d '"' -f2)
# echo $OSID
connection=""

OSVer=$(cat /etc/*release | grep -i "pretty_name" | cut -d '"' -f2)
if [ -z "$OSVer" ]; then
	OSVer=$(cat /etc/*release | grep -i "DISTRIB_DESCRIPTION" | cut -d '"' -f2)
fi
# echo $OSVer

sCPU=$(grep -c ^processor /proc/cpuinfo)
sRAM=$(cat /proc/meminfo | grep MemTotal | cut -d ':' -f2 | tr -d '[:space:]' | sed 's/.\{2\}$//' | awk '{$1=$1/(1024^2); print int($1+0.5),"GB";}')

if [ "$sRAM" == "0 GB" ]; then
	sRAM=$(cat /proc/meminfo | grep MemTotal | cut -d ':' -f2 | tr -d '[:space:]' | sed 's/.\{2\}$//' | awk '{$1=$1/1024; print $1 " MB"}')
fi

if [ $(uname -m) == "x86_64" ]; then
	sARCH="64-Bit ($(uname -m))"
else
	sARCH="32-Bit ($(uname -m))"
fi
# echo $sARCH
#sARCH=$(dpkg --print-architecture)

netAdapter=$(ip -br -c link show | grep -vi 'loopback\|unknown\|down' | awk '{ print $1 }' | sed 's/^[ \t]*//;s/[ \t]*$//')
# echo $netAdapter
netInfomation=""

if [ ! -z "$netAdapter" ]; then
	netInfomation=${netInfomation}$(echo "\n\nNetwork Information\n")

	val="eth0"
	#echo $val

	netIP=$(/sbin/ip -o -4 addr list "$val" | awk '{print $4}' | cut -d '/' -f1)
	# echo $netIP

	netMask="$(ifconfig "$val" | sed -rn '2s/ .*:(.*)$/\1/p')"
	if [ -z "$netMask" ]; then
		netMask="$(ifconfig $val | grep netmask | awk '{print $4}')"
	fi
	if [[ ! "$netMask" == *"255"* ]]; then
		netMask=""
	fi
	# echo $netMask

	if [[ $netIP == *"172"* ]]; then
		radioIP=$netIP
	elif [[ $netIP == *"192"* ]]; then
		cableIP=$netIP
	fi

	netInfomation=${netInfomation}$(echo "\tNetwork Adapter:\t$val\n\tNetwork IP:\t\t$netIP\n\tNetwork Mask:\t\t$netMask\n\n")

fi

report="Hostname:\t\t$(hostname)"
report=${report}"\nOS:\t\t\t${OSVer}"
report=${report}"\nCPU:\t\t\t${sCPU}"
report=${report}"\nArchitecture:\t\t${sARCH}"
report=${report}"\nMemory:\t\t\t${sRAM}"
report=${report}"\nIP - Radio:\t\t${netIP}"

report=${report}"$netInfomation"

echo -e $report

echo -e "Connected Devices"
ip neigh >/tmp/arp.txt
while IFS="" read -r p || [ -n "$p" ]; do
	# printf '%s\n' "$p" | awk '{print $1,"|",$3,"|",$5,"|",$6}' | sed 's| ||g'
	echo -e "\t\t$p" | awk '{print $1,"|",$3,"|",$5,"|",$6}' | sed 's| ||g'
done </tmp/arp.txt
rm -f /tmp/arp.txt

echo -e "\n\nNetwork Details"
ifconfig

echo -e "\n\nPartition Details"
lsblk

echo -e "\n\nUser List"
# awk -F: '$3 >= 1000 && $3 < 65534 {print $1}' /etc/passwd
getent group everyone network tty | cut -d: -f4 | sed 's|,|\n|g' | sort | uniq

# echo -e "\n\nSudo User List"
# getent group wheel sudo | cut -d: -f4 | sed 's|,|\n|g'
