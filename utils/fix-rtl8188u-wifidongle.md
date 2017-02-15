This was an incredibly tricky one. Basically, the rpi linux kernel uses proprietary realtek drivers instead of the 
opensource ones because they work better. The problem is that they don't support monitor mode. So to fix this you download

backports-4.4.2-1.tar.gz 

or the appropriate newer version. You then have to make sure to install the rtl8xxxu driver. To check what drivers your
network card uses you can use `lshw -c network`.

!!! Beware !!!
The backports `make install` does not always copy the drivers to the correct kernel folder. Make sure that 
`/lib/modules/$(uname -r)` contains an `updates` folder. Otherwise copy it from one of the other folders.


#### Kernel headers 
Find yours here https://www.niksula.hut.fi/~mhiienka/Rpi/linux-headers-rpi/
