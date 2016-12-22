#!/bin/sh
rm -r rover -f
tar xvf exe_main.tar
cd ./rover
make clean
make
chmod 777 exe_main
