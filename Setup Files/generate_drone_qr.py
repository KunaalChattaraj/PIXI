#!/usr/bin/env python3
"""
Generate a QR code that encodes the drone's connection URI as plain text.

Print this out and stick it on (or near) the drone. The main GUI's
Connect button scans for this instead of using a hardcoded IP, so
swapping drones just means printing a new QR code - no code edits.
"""

import qrcode

DRONE_URI = "udp://192.168.0.11"

qr = qrcode.QRCode(
    version=None,  # auto-pick the smallest version that fits
    error_correction=qrcode.constants.ERROR_CORRECT_M,
    box_size=10,
    border=4,
)
qr.add_data(DRONE_URI)
qr.make(fit=True)

img = qr.make_image(fill_color="black", back_color="white")
out_name = "qr_drone_uri.png"
img.save(out_name)

print(f"QR code saved as {out_name}")
print(f"Encoded: {DRONE_URI}")
