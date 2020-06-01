gcc -I/usr/include/lua5.1 -Wl,-E -fPIC -shared readline.c -o readline.so -lreadline -llua5.1
cp readline.so ../../work/
