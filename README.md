# moxa-hid-ft260-driver

## Moxa FT260 HID USB to SMBus master bridge driver
Reference:
- https://github.com/MichaelZaidman/hid-ft260
- https://lkml.org/lkml/2021/2/19/484

### Compile & install the driver

#### Install build-essential packages
```
apt-get install build-essential linux-headers-`uname -r`
```

#### Compile the driver
```
make
```

#### Compile and install the driver
```
make install
depmod -a
```

#### Load the driver
```
modprobe hid-ft260
```
