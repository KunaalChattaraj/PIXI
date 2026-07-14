import qrcode

data = "http://11.11.26.52"

qr = qrcode.QRCode(
    version=1,
    error_correction=qrcode.constants.ERROR_CORRECT_M,
    box_size=10,
    border=4,
)

qr.add_data(data)
qr.make(fit=True)

img = qr.make_image(fill_color="black", back_color="white")
img.save("qr_10.10.2.50.png")

print("QR code saved as qr_10.10.2.50.png")