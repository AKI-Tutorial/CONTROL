#!/bin/sh -f
rm exe_main.tar -f
tar cvf exe_main.tar ../rover
#
echo "FTP connection..."
ftp -n <<EOF
open 192.168.201.11
user guest guest
binary
cd /home/guest
delete exe_main.tar
put exe_main.tar
put em_shell.sh
bye
EOF
#
rm exe_main.tar -f
