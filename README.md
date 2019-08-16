# moxa-hid-ft260-driver

## Moxa FT260 HID USB to SMBus master bridge driver
the hid driver based on
https://elixir.bootlin.com/linux/v4.9.168/source/drivers/hid/hid-cp2112.c

### Compile & install the driver

#### Install build-essential packages
```
sudo apt-get install build-essential linux-headers-4.9.0-9-amd64=4.9.168-1+deb9u4
```

#### Compile the driver
```
make
```

#### Compile and install the driver
```
make install
```

#### Load the watchdog driver
```
modprobe hid-ft260
```
