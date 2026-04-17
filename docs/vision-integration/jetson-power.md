# Jetson Nano Power

## Power Modes

The Jetson Nano Developer Kit has two power input options:

| Method | Connector | Max Power | Suitable for this project? |
|--------|-----------|-----------|---------------------------|
| Micro-USB | Micro-USB | 5V/2A (10W) | No — will throttle under ZED SDK load |
| DC Barrel Jack | 5.5mm/2.1mm center-positive | 5V/4A (20W) | Yes |

**Always use the barrel jack for this project.** The ZED SDK running object detection
on the GPU will push the Jetson close to its power limits. The Micro-USB path cannot
supply enough current and will cause CPU/GPU throttling or instability.

## Required: J48 Jumper

To enable barrel jack power, the **J48 jumper** on the Jetson Nano board must be
shorted. This jumper is located near the barrel jack connector. Without it, the board
defaults to Micro-USB even if a barrel jack is plugged in.

## Power Supply Options for the Rover

The Jetson Nano requires a stable **5V DC, minimum 4A** supply. The Pi is already
handled by the X1202 UPS — the Jetson needs its own separate rail.

### Option A — Dedicated Power Bank (easiest)

Use a USB-C or barrel jack power bank rated for 5V/4A output (sometimes labeled
"20W"). Many laptop power banks support this.

**Pros:** No wiring, self-contained, easy to swap  
**Cons:** Another battery to manage and charge

### Option B — DC-DC Buck Converter from Main Battery (cleanest for rover)

If the rover has a main LiPo or other battery pack (e.g. 12V), use a buck converter
module (e.g. LM2596-based, or a pre-built USB buck module) to step it down to 5V/4A.

**Pros:** Single battery powers the whole rover; no extra pack to manage  
**Cons:** Requires wiring; buck converter adds a failure point

### Option C — Second X1202 UPS (most robust)

A second X1202 UPS unit dedicated to the Jetson, fed from the same main battery.

**Pros:** Clean regulated power with protection; matches the Pi's setup  
**Cons:** Cost and weight of a second UPS unit

## Important: Keep Jetson and Pi on Separate Power Rails

The Jetson Nano has a high inrush current at startup (can spike significantly before
settling). Sharing a power rail with the Pi risks resetting or browning out the Pi
when the Jetson boots. Always power them independently.
