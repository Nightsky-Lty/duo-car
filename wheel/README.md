duo-pinmux -w GP0/GP0 &&
duo-pinmux -w GP1/GP1 &&
duo-pinmux -w GP2/GP2 &&
duo-pinmux -w GP3/GP3 &&
duo-pinmux -w GP7/GP7 &&
duo-pinmux -w GP6/GP6 &&
duo-pinmux -w GP4/PWM_5 &&
duo-pinmux -w GP5/PWM_6 &&
duo-pinmux -w GP10/GP10 &&
duo-pinmux -w GP11/GP11 &&
duo-pinmux -w GP12/GP12 &&
duo-pinmux -w GP13/GP13 &&
duo-pinmux -w GP18/GP18 &&
duo-pinmux -w GP19/GP19 &&
duo-pinmux -w GP20/GP20 &&
duo-pinmux -w GP21/GP21

ssh root@192.168.42.1
cd duo-examples && source envsetup.sh
cd wheel
make && scp -O wheel root@192.168.42.1:/root/