#!/bin/bash
# update.sh
#
# Temporarily connects to WiFi, pulls the latest repo, then restores the AP.
#
# Usage:
#   ./update.sh

REPO="/home/lucia/repos/group-project-group-37-project-l-u-c-i-a"
SSID="lucia"
PASSWORD="lucia-143-tomato"
AP_CON="pi-ap"
WPA_CONF="/tmp/wpa_update.conf"
WPA_PID="/tmp/wpa_update.pid"

restore() {
    echo "Restoring AP..."
    sudo kill "$(cat $WPA_PID 2>/dev/null)" 2>/dev/null || true
    sudo dhcpcd -k wlan0 2>/dev/null || true
    sudo rm -f "$WPA_CONF" "$WPA_PID" /run/wpa_supplicant/wlan0
    sleep 2
    sudo nmcli device set wlan0 managed yes
    sleep 1
    sudo systemctl start wpa_supplicant 2>/dev/null || true
    sudo nmcli connection up "$AP_CON"
    echo "Starting dnsmasq..."
    sudo systemctl start dnsmasq
    echo "AP restored."
}

trap restore EXIT

echo "Stopping dnsmasq..."
sudo systemctl stop dnsmasq

echo "Bringing down AP..."
sudo nmcli connection down "$AP_CON" 2>/dev/null || true
sleep 2

echo "Handing wlan0 to wpa_supplicant..."
sudo nmcli device set wlan0 managed no
sudo systemctl stop wpa_supplicant 2>/dev/null || true
sudo killall wpa_supplicant 2>/dev/null || true
sleep 1
sudo rm -f /run/wpa_supplicant/wlan0

cat > "$WPA_CONF" << EOF
ctrl_interface=/run/wpa_supplicant
ctrl_interface_group=0
network={
    ssid="$SSID"
    psk="$PASSWORD"
}
EOF

sudo wpa_supplicant -B -i wlan0 -c "$WPA_CONF" -P "$WPA_PID" 2>/dev/null

echo "Waiting for association..."
for i in $(seq 1 15); do
    sleep 1
    STATUS=$(sudo wpa_cli -i wlan0 -p /run/wpa_supplicant status 2>/dev/null | grep wpa_state | cut -d= -f2)
    echo "  [$i/15] $STATUS"
    if [ "$STATUS" = "COMPLETED" ]; then
        echo "Associated."
        break
    fi
done

if [ "$STATUS" != "COMPLETED" ]; then
    echo "Failed to associate with $SSID. Is the network in range?"
    exit 1
fi

echo "Getting IP..."
sudo dhcpcd wlan0 > /dev/null 2>&1

echo "Pulling latest repo..."
cd "$REPO" && git fetch --all && git clean -fd && git reset --hard @{u}

echo "Update complete."
