#!/usr/bin/env python3
"""
Manual USB Reset Methods for RealSense D435
Works without additional packages
"""

import subprocess
import os
import time
import sys

def create_usbreset_tool():
    """Create and compile usbreset tool if it doesn't exist"""
    if os.path.exists('/usr/local/bin/usbreset'):
        return True
    
    print("Creating usbreset tool...")
    
    usbreset_code = '''
/* usbreset -- send a USB port reset to a USB device */
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/usbdevice_fs.h>

int main(int argc, char **argv)
{
    const char *filename;
    int fd;
    int rc;

    if (argc != 2) {
        fprintf(stderr, "Usage: usbreset device-filename\\n");
        return 1;
    }
    filename = argv[1];

    fd = open(filename, O_WRONLY);
    if (fd < 0) {
        perror("Error opening output file");
        return 1;
    }

    printf("Resetting USB device %s\\n", filename);
    rc = ioctl(fd, USBDEVFS_RESET, 0);
    if (rc < 0) {
        perror("Error in ioctl");
        return 1;
    }
    printf("Reset successful\\n");

    close(fd);
    return 0;
}
'''
    
    try:
        # Write source code
        with open('/tmp/usbreset.c', 'w') as f:
            f.write(usbreset_code)
        
        # Compile
        result = subprocess.run(['gcc', '/tmp/usbreset.c', '-o', '/tmp/usbreset'], 
                              capture_output=True, text=True)
        if result.returncode != 0:
            print(f"Compilation failed: {result.stderr}")
            return False
        
        # Install
        subprocess.run(['sudo', 'cp', '/tmp/usbreset', '/usr/local/bin/'], check=True)
        subprocess.run(['sudo', 'chmod', '+x', '/usr/local/bin/usbreset'], check=True)
        
        # Cleanup
        os.remove('/tmp/usbreset.c')
        os.remove('/tmp/usbreset')
        
        print("usbreset tool created successfully!")
        return True
        
    except Exception as e:
        print(f"Failed to create usbreset: {e}")
        return False


def find_realsense_device():
    """Find RealSense device information"""
    try:
        result = subprocess.run(['lsusb'], capture_output=True, text=True)
        for line in result.stdout.split('\n'):
            if 'Intel Corp.' in line and 'RealSense' in line:
                # Extract bus and device numbers
                parts = line.split()
                bus = parts[1]
                device = parts[3].rstrip(':')
                
                # Pad with zeros
                bus = bus.zfill(3)
                device = device.zfill(3)
                
                return bus, device, line.strip()
    except Exception as e:
        print(f"Error finding device: {e}")
    
    return None, None, None


def reset_using_usbreset(bus, device):
    """Reset USB device using usbreset tool"""
    device_path = f"/dev/bus/usb/{bus}/{device}"
    
    if not os.path.exists(device_path):
        print(f"Device path {device_path} does not exist")
        return False
    
    try:
        print(f"Resetting device at {device_path}...")
        result = subprocess.run(['sudo', 'usbreset', device_path], 
                              capture_output=True, text=True)
        print(result.stdout)
        if result.returncode == 0:
            return True
        else:
            print(f"Reset failed: {result.stderr}")
            return False
    except Exception as e:
        print(f"usbreset error: {e}")
        return False


def reset_using_sysfs():
    """Reset USB device using sysfs unbind/bind method"""
    print("\nTrying sysfs reset method...")
    
    try:
        # Find all USB devices
        result = subprocess.run(['find', '/sys/bus/usb/devices/', '-name', 'idVendor', 
                               '-type', 'f', '-readable'], 
                              capture_output=True, text=True)
        
        for vendor_path in result.stdout.strip().split('\n'):
            if not vendor_path:
                continue
                
            try:
                with open(vendor_path, 'r') as f:
                    vendor_id = f.read().strip()
                    
                if vendor_id == '8086':  # Intel vendor ID
                    device_path = os.path.dirname(vendor_path)
                    product_path = os.path.join(device_path, 'idProduct')
                    
                    if os.path.exists(product_path):
                        with open(product_path, 'r') as f:
                            product_id = f.read().strip()
                            
                        # RealSense D435 product IDs
                        if product_id.lower() in ['0b07', '0b3a']:
                            print(f"Found RealSense at: {device_path}")
                            device_name = os.path.basename(device_path)
                            
                            # Check if authorized file exists (USB authorization method)
                            auth_path = os.path.join(device_path, 'authorized')
                            if os.path.exists(auth_path):
                                print("Using USB authorization reset...")
                                # Deauthorize
                                subprocess.run(['sudo', 'sh', '-c', 
                                              f'echo 0 > {auth_path}'], check=True)
                                time.sleep(1)
                                # Reauthorize
                                subprocess.run(['sudo', 'sh', '-c', 
                                              f'echo 1 > {auth_path}'], check=True)
                                print("Authorization reset completed")
                                return True
                            
                            # Try unbind/bind method
                            unbind_path = '/sys/bus/usb/drivers/usb/unbind'
                            bind_path = '/sys/bus/usb/drivers/usb/bind'
                            
                            if os.path.exists(unbind_path) and os.path.exists(bind_path):
                                print("Using unbind/bind reset...")
                                # Unbind
                                subprocess.run(['sudo', 'sh', '-c', 
                                              f'echo {device_name} > {unbind_path}'], 
                                             check=True)
                                time.sleep(1)
                                # Bind
                                subprocess.run(['sudo', 'sh', '-c', 
                                              f'echo {device_name} > {bind_path}'], 
                                             check=True)
                                print("Unbind/bind reset completed")
                                return True
                                
            except Exception as e:
                continue
                
    except Exception as e:
        print(f"Sysfs reset error: {e}")
    
    return False


def reset_using_power_control():
    """Try to control USB port power if supported"""
    print("\nTrying USB power control method...")
    
    try:
        # Find USB hubs that support power control
        result = subprocess.run(['find', '/sys/bus/usb/devices/', '-name', 'power', 
                               '-type', 'd'], 
                              capture_output=True, text=True)
        
        for power_dir in result.stdout.strip().split('\n'):
            if not power_dir:
                continue
                
            control_path = os.path.join(power_dir, 'control')
            if os.path.exists(control_path):
                device_path = os.path.dirname(power_dir)
                
                # Check if this is related to our RealSense device
                product_path = os.path.join(device_path, 'product')
                if os.path.exists(product_path):
                    try:
                        with open(product_path, 'r') as f:
                            product = f.read().strip()
                        
                        if 'RealSense' in product:
                            print(f"Found RealSense power control at: {control_path}")
                            
                            # Cycle power
                            subprocess.run(['sudo', 'sh', '-c', 
                                          f'echo auto > {control_path}'], check=True)
                            time.sleep(0.5)
                            subprocess.run(['sudo', 'sh', '-c', 
                                          f'echo on > {control_path}'], check=True)
                            print("Power cycle completed")
                            return True
                            
                    except:
                        continue
                        
    except Exception as e:
        print(f"Power control error: {e}")
    
    return False


def perform_full_reset():
    """Perform a complete USB reset trying all available methods"""
    print("=== RealSense USB Reset Tool ===\n")
    
    # Find device
    bus, device, info = find_realsense_device()
    
    if not bus:
        print("ERROR: No RealSense device found!")
        print("\nPlease check:")
        print("1. Is the camera connected?")
        print("2. Try: lsusb | grep -i real")
        return False
    
    print(f"Found: {info}")
    print(f"Bus: {bus}, Device: {device}")
    
    # Method 1: Try usbreset tool
    if os.path.exists('/usr/local/bin/usbreset'):
        print("\nMethod 1: Using usbreset tool...")
        if reset_using_usbreset(bus, device):
            print("Success!")
            return True
    else:
        print("\nusbreset tool not found. Creating it...")
        if create_usbreset_tool():
            if reset_using_usbreset(bus, device):
                print("Success!")
                return True
    
    # Method 2: Try sysfs reset
    print("\nMethod 1 failed. Trying Method 2: sysfs reset...")
    if reset_using_sysfs():
        print("Success!")
        return True
    
    # Method 3: Try power control
    print("\nMethod 2 failed. Trying Method 3: power control...")
    if reset_using_power_control():
        print("Success!")
        return True
    
    print("\n*** All reset methods failed ***")
    print("\nManual reset options:")
    print("1. Unplug and replug the USB cable")
    print("2. Try a different USB port")
    print("3. Reboot the Raspberry Pi: sudo reboot")
    
    return False


if __name__ == "__main__":
    if os.geteuid() != 0:
        print("Note: This script may need sudo privileges for some operations.")
        print("If it fails, try running with: sudo python3", sys.argv[0])
        print()
    
    # Check for gcc (needed to compile usbreset)
    try:
        subprocess.run(['gcc', '--version'], capture_output=True, check=True)
    except:
        print("WARNING: gcc not found. Installing build-essential...")
        subprocess.run(['sudo', 'apt-get', 'install', '-y', 'build-essential'])
    
    # Perform reset
    success = perform_full_reset()
    
    if success:
        print("\nUSB reset completed successfully!")
        print("Wait 3-5 seconds before trying to use the camera again.")
        time.sleep(3)
    else:
        print("\nUSB reset failed. Please try manual methods.")
    
    sys.exit(0 if success else 1)