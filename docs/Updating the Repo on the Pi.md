# Updating the Repo on the Pi

## Overview

The Pi runs as a WiFi access point (`lucia-control`) with dnsmasq as a DHCP server on `wlan0`. Because `wlan0` is used for the AP, it can't simultaneously connect to another network for internet access.

`update.sh` automates the process of temporarily tearing down the AP, connecting to the `lucia` WiFi network to pull the latest code, then restoring the AP.

## Usage

SSH into the Pi:

```bash
# Via the Pi's AP (lucia-control)
ssh lucia@10.42.0.1

# Via ethernet
ssh lucia@192.168.0.140
```

Then run:

```bash
~/repos/group-project-group-37-project-l-u-c-i-a/update.sh
```

No arguments needed. The script always restores the AP on exit, even if something goes wrong. It is safe to run repeatedly.

## What It Does

1. Stops dnsmasq and brings down the `pi-ap` connection
2. Stops the system wpa_supplicant service and releases `wlan0` from NetworkManager
3. Starts a temporary wpa_supplicant instance to connect to `lucia` WiFi
4. Obtains an IP via dhcpcd
5. Runs `git fetch --all && git pull` on the repo
6. Kills wpa_supplicant, restarts the system wpa_supplicant service, restores the AP, and restarts dnsmasq

## Configuration

These variables are at the top of `update.sh`:

| Variable   | Default                                             | Description                  |
|------------|-----------------------------------------------------|------------------------------|
| `REPO`     | `/home/lucia/repos/group-project-group-37-project-l-u-c-i-a` | Path to the repo on the Pi   |
| `SSID`     | `lucia`                                             | WiFi network to connect to   |
| `PASSWORD` | `lucia-143-tomato`                                  | WiFi password                |
| `AP_CON`   | `pi-ap`                                             | NetworkManager AP profile name |

## Troubleshooting

**Stuck at SCANNING and never connects**
A stale wpa_supplicant process may be running. Fix:
```bash
sudo killall wpa_supplicant
sudo rm -f /run/wpa_supplicant/wlan0
```
Then run the script again.

**AP doesn't come back up after the script**
Run manually:
```bash
sudo systemctl start wpa_supplicant
sudo nmcli device set wlan0 managed yes
sudo nmcli connection up pi-ap
sudo systemctl start dnsmasq
```

**Failed to associate — wrong password**
The wpa_supplicant association will cycle through `4WAY_HANDSHAKE` repeatedly without completing. Update the `PASSWORD` variable in `update.sh`.
