# Raspberry Pi Initial Setup: Wi-Fi AP + DHCP Server 

## Overview

This setup configures a Raspberry Pi to:

- Broadcast a WiFi network
- Assign IP addresses via DHCP
- Allow SSH access

## Network Configuration

| Component        | Value                  |
|------------------|------------------------|
| SSID             | lucia-control          |
| Password         | lucia-143-tomato       |
| Interface        | wlan0                  |
| Pi IP            | 10.42.0.1              |
| DHCP Range       | 10.42.0.10 – 10.42.0.100 |
| Subnet           | 255.255.255.0          |
| SSH Address      | 10.42.0.1              |

## Flash Raspberry Pi OS

Use Raspberry Pi Imager:

- Enable SSH
- Set username/password
- Set WLAN country

## Install Dependencies

sudo apt update
sudo apt install dnsmasq -y

## Create Access Point

sudo nmcli connection add type wifi ifname wlan0 con-name pi-ap ssid lucia-control

## Configure AP Mode

sudo nmcli connection modify pi-ap 802-11-wireless.mode ap
sudo nmcli connection modify pi-ap 802-11-wireless.band bg
sudo nmcli connection modify pi-ap 802-11-wireless.channel 6

## Configure Security

sudo nmcli connection modify pi-ap wifi-sec.key-mgmt wpa-psk
sudo nmcli connection modify pi-ap wifi-sec.psk "lucia-143-tomato"

## Configure Static IP

sudo nmcli connection modify pi-ap ipv4.addresses 10.42.0.1/24
sudo nmcli connection modify pi-ap ipv4.method manual
sudo nmcli connection modify pi-ap ipv4.never-default yes

## Bind to WiFi Interface

sudo nmcli connection modify pi-ap connection.interface-name wlan0

## Enable Auto-Start

sudo nmcli connection modify pi-ap connection.autoconnect yes

## Start Access Point

sudo nmcli connectio up pi-ap

## Configure dnsmasq

/etc/dnsmasq.conf

interface=wlan0
bind-interfaces
port=0
dhcp-range=10.42.0.10,10.42.0.100,255.255.255.0,24h
dhcp-option=3,10.42.0.1
dhcp-option=6,10.42.0.1

## Start and Enable DHCP

sudo systemctl start dnsmasq
sudo systemctl enable dnsmasq

