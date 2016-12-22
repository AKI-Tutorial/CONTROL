#!/bin/sh
#
THOUR=$(date +%H:%M:%S)
echo $THOUR
#TDAY=$(date +%m%d%H%M%Y.%S)
#TDAY=$(date -u +%Y.%m.%d-%H:%M:%S)
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
expect \"# \" ; send \"date -u $TDAY\r\"
expect \"# \" ; send \"cd /home/guest\r\"
expect \"# \" ; send \"chmod 777 *\r\"
expect \"# \" ; send \"chmod 777 /dev/ttyUSB0\r\"
expect \"# \" ; send \"chmod 777 /dev/ttyS*\r\"
expect \"# \" ; send \"chmod 777 /dev/mem\r\"
expect \"# \" ; send \"tar xvf exe_main.tar\r\"
expect \"# \" ; send \"cd ./rover\r\"
expect \"# \" ; send \"make clean\r\"
expect \"# \" ; send \"make\r\"
interact
"
