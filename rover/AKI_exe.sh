#!/bin/sh
#
THOUR=$(date +%H:%M:%S)
echo $THOUR
TDAY=$(date -u +%m%d%H%M%Y)
echo $TDAY
#
MC1=192.168.201.11
ID=temp
ID1=guest
PWD1=guest
ID2=root
PWD2=peke
#
expect -c "
set timeout -1
spawn telnet $MC1
expect login:\ ; send \"$ID1\r\"
expect sword:\ ; send \"$PWD1\r\"
expect \"$ \" ; send \"ls -l\r\"
expect \"$ \" ; send \"su\r\"
expect sword:\ ; send \"$PWD2\r\"
expect \"# \" ; send \"cd /home/guest\r\"
expect \"# \" ; send \"chmod 777 *\r\"
expect \"# \" ; send \"chmod 777 /dev/ttyUSB0\r\"
expect \"# \" ; send \"chmod 777 /dev/ttyS*\r\"
expect \"# \" ; send \"chmod 777 /dev/mem\r\"
expect \"# \" ; send \"cd ./rover\r\"
expect \"# \" ; send \"./exe_main\r\"
interact
"
