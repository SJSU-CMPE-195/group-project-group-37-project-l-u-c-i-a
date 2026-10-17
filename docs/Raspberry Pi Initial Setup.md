# Raspberry Pi Initial Setup: Wi-Fi AP + DHCP Server

## Overview

This setup configures a Raspberry Pi to:

- Broadcast a WiFi network (`pi-ap`) at every boot, even when a saved WiFi is in range
- Assign IP addresses via DHCP (`dnsmasq`)
- Allow SSH access

Checked against the robot's Pi on 2026-10-17: every value below matches what it
runs. For day-to-day use (switching to another WiFi for internet and back), see
"Network modes" in [`setup/ros2-pi.md`](setup/ros2-pi.md).

## Network Configuration

| Component    | Value                      |
|--------------|----------------------------|
| SSID         | lucia-control              |
| Password     | lucia-143-tomato           |
| Interface    | wlan0                      |
| Pi IP        | 10.42.0.1                  |
| DHCP Range   | 10.42.0.10 – 10.42.0.100   |
| Subnet       | 255.255.255.0              |
| SSH Address  | 10.42.0.1                  |

## Flash Raspberry Pi OS

Use Raspberry Pi Imager:

- Enable SSH
- Set username/password
- Set WLAN country

## Install Dependencies

```bash
sudo apt update
sudo apt install dnsmasq -y
```

## Create Access Point

```bash
sudo nmcli connection add type wifi ifname wlan0 con-name pi-ap ssid lucia-control
```

## Configure AP Mode

```bash
sudo nmcli connection modify pi-ap 802-11-wireless.mode ap
sudo nmcli connection modify pi-ap 802-11-wireless.band bg
sudo nmcli connection modify pi-ap 802-11-wireless.channel 6
```

## Configure Security

```bash
sudo nmcli connection modify pi-ap wifi-sec.key-mgmt wpa-psk
sudo nmcli connection modify pi-ap wifi-sec.psk "lucia-143-tomato"
```

## Configure Static IP

```bash
sudo nmcli connection modify pi-ap ipv4.addresses 10.42.0.1/24
sudo nmcli connection modify pi-ap ipv4.method manual
sudo nmcli connection modify pi-ap ipv4.never-default yes
```

## Bind to WiFi Interface

```bash
sudo nmcli connection modify pi-ap connection.interface-name wlan0
```

## Enable Auto-Start, Ahead of Saved WiFi

```bash
sudo nmcli connection modify pi-ap connection.autoconnect yes
sudo nmcli connection modify pi-ap connection.autoconnect-priority 100
```

The priority matters. At boot, NetworkManager brings up the highest-priority
connection it can. Saved WiFi networks default to 0, so without this the Pi joins
a saved network in range (a home WiFi) instead of starting the hotspot, and you
can't reach it on `lucia-control`. With it, the hotspot always wins at boot, and if
the Pi is switched to another WiFi that then fails, it falls back to the hotspot.

## Start Access Point

```bash
sudo nmcli connection up pi-ap
```

## Configure dnsmasq

Edit `/etc/dnsmasq.conf`:

```
interface=wlan0
bind-interfaces
port=0
dhcp-range=10.42.0.10,10.42.0.100,255.255.255.0,24h
dhcp-option=3,10.42.0.1
dhcp-option=6,10.42.0.1
```

## Start and Enable DHCP

```bash
sudo systemctl start dnsmasq
sudo systemctl enable dnsmasq
```

## Check

```bash
nmcli -f NAME,AUTOCONNECT,AUTOCONNECT-PRIORITY connection show   # pi-ap: yes, 100
systemctl is-enabled dnsmasq NetworkManager avahi-daemon         # all enabled
```

`avahi-daemon` (on by default) makes the Pi reachable as `lucia.local`
(`ssh lucia@lucia.local`) on any network, so its IP doesn't need looking up.
