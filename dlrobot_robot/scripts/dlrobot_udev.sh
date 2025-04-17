echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="0002", MODE:="0777", GROUP:="dialout", SYMLINK+="dlrobot_controller"' >/etc/udev/rules.d/dlrobot_controller.rules
service udev reload
sleep 2
service udev restart


