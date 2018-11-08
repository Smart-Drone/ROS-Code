# ROS-Code

## Dependencies

### Operating system:
Ubuntu 16.04.5

### Ubuntu packages:
dnsmasq  
hostapd

### ROS binary packages:
ros-kinetic-desktop-full  
ros-kinetic-ardrone-autonomy   
ros-kinetic-hector-*

### ROS source packages:
[angelsantamaria/tum_simulator](https://github.com/angelsantamaria/tum_simulator)

### C++ Libraries:
Boost 1.67.0

## Setup and configuration
* Install Ubuntu 16.04.5
* Install Ubuntu packages from official repositories
* Install ROS binary packages ([ROS Wiki](http://wiki.ros.org/kinetic/Installation/Ubuntu))
* Compile ROS source packages ([ROS Wiki](http://wiki.ros.org/ROS/Tutorials/BuildingPackages))
* Download Boost 1.67.0 and extract it into your home directory (```~/boost_1_67_0```)
* Edit ```/etc/network/interfaces```
```
auto lo
iface lo inet loopback

# WiFi interface for communication with uControllers
auto wlp2s0
iface wlp2s0 inet static
  address 10.0.0.1
  netmask 255.255.255.0
```
* Edit ```/etc/dnsmasq.conf```
```
no-resolv

# WiFi interface for communication with uControllers
interface=wlp2s0

dhcp-range=10.0.0.100,10.0.0.254,255.255.255.0,12h
no-hosts
addn-hosts=/etc/hosts.dnsmasq

# IP address assignment for uControllers
dhcp-host=5C:CF:7F:6C:D9:70,MOTION-CONTROLLER-RIGHT,10.0.0.2
dhcp-host=60:01:94:43:3D:A0,MOTION-CONTROLLER-LEFT,10.0.0.3

server=8.8.4.4
server=8.8.8.8
```
* Edit ```/etc/hostapd/hostapd.conf```
```
# WiFi interface for communication with uControllers
interface=wlp2s0

driver=nl80211
ssid=PARROT_NEST
hw_mode=g
channel=6
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
wpa=3
wpa_passphrase=pollycracker
wpa_key_mgmt=WPA-PSK
wpa_pairwise=TKIP
rsn_pairwise=CCMP
```
* Edit ```/etc/default/hostapd```
```
DAEMON_CONF=/etc/hostapd/hostapd.conf
```
