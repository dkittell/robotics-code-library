#!/bin/bash
#  Place this script within /etc/profile.d/setPS1.sh
#
Red='\e[1;31m'
Yellow='\e[1;33m'
White='\e[0;37m'
Green='\e[0;32m'
Normal='\e[m'

if [ $EUID = 0 ]; then
    export PS1="[${Red}\u${White}@${Yellow}\h${Normal}] \w ${Red}#${Normal} "
else
    export PS1="[${Green}\u${White}@${Yellow}\h${Normal}] \w $ "
fi
