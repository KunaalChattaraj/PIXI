import cflib.crtp

# Initialize drivers
cflib.crtp.init_drivers(enable_debug_driver=False)

print("Scanning for Crazyflies...")

# Scan for available interfaces
available = cflib.crtp.scan_interfaces()

if len(available) > 0:
    print("Found Crazyflies:")
    for uri, _ in available:
        print(uri)
else:
    print("No Crazyflies found")
