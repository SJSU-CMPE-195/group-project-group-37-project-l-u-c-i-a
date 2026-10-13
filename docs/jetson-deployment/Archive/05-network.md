# Phase 5 — Network Configuration

Configure the direct Gigabit Ethernet link between the Jetson and the Pi.
No router or switch — point-to-point cable only.

---

## Target

| Board | Interface | Static IP |
|-------|-----------|-----------|
| Raspberry Pi | eth0 | 192.168.1.1 |
| Jetson Nano | eth0 | 192.168.1.2 |

---

## Step 1 — Connect the Ethernet cable

Plug a Cat5e or Cat6 cable directly between:
- Jetson Nano `eth0`
- Raspberry Pi `eth0`

---

## Step 2 — Set static IP on Jetson

```bash
sudo nmcli con mod "Wired connection 1" \
  ipv4.addresses 192.168.1.2/24 \
  ipv4.method manual
sudo nmcli con up "Wired connection 1"
```

If the connection name is different, list available connections first:

```bash
nmcli con show
```

**Connection name used:** <!-- fill in -->

Verify:

```bash
ip addr show eth0
```

**Actual output:**
```
# paste here
```

---

## Step 3 — Set static IP on Pi

On the Pi (outside Docker):

```bash
sudo nmcli con mod "Wired connection 1" \
  ipv4.addresses 192.168.1.1/24 \
  ipv4.method manual
sudo nmcli con up "Wired connection 1"
```

Or if using `dhcpcd.conf`:

```bash
echo -e "\ninterface eth0\nstatic ip_address=192.168.1.1/24" | sudo tee -a /etc/dhcpcd.conf
sudo systemctl restart dhcpcd
```

**Method used:** <!-- nmcli / dhcpcd.conf -->

---

## Step 4 — Confirm link is up

From Jetson:

```bash
ping -c 4 192.168.1.1
```

**Actual output:**
```
# paste here
```

From Pi:

```bash
ping -c 4 192.168.1.2
```

**Actual output:**
```
# paste here
```

**Result:** <!-- Both ping / one-way / no link -->

---

## Notes

<!-- Interface names if different from eth0, any link negotiation issues -->
