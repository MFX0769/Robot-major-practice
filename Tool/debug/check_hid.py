import hid
import time

# Enumerate all HID devices
print("=== HID Devices ===")
for d in hid.enumerate():
    if d['vendor_id'] == 0xEF1A:
        print(f"VID: 0x{d['vendor_id']:04x} PID: 0x{d['product_id']:04x}")
        print(f"  Product: {d['product_string']}")
        print(f"  Manufacturer: {d['manufacturer_string']}")
        print(f"  Usage: 0x{d['usage']:04x} Page: 0x{d['usage_page']:04x}")
        print(f"  Path: {d['path']}")
        print()

# Try to open and read
try:
    dev = hid.device()
    dev.open(0xEF1A, 0x74E5)
    print(f"Opened: {dev.get_product_string()}")
    
    # Try to read
    dev.set_nonblocking(True)
    data = dev.read(64)
    if data:
        print(f"Read: {list(data)}")
    else:
        print("No data available")
    
    dev.close()
except Exception as e:
    print(f"Error: {e}")
