#!/bin/sh
ntpsv=192.168.201.12
/usr/sbin/ntpclient -h $ntpsv -s
#/usr/sbin/ntpdate $ntpsv
#set menuconfig