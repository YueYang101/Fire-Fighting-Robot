#!/usr/bin/env python3
"""
YDLidar G2 Test Script
Tests if the lidar sensor is working via USB connection
Compatible with both Raspberry Pi (Ubuntu 22.04) and macOS
"""

import sys
import time
import signal
import serial
import serial.tools.list_ports
from struct import unpack

class YDLidarG2Test:
    def __init__(self):
        self.port = None
        self.serial_connection = None
        self.running = False
        
        # G2 Lidar specifications
        self.LIDAR_BAUDRATE = 230400
        self.SCAN_COMMAND = b'\xA5\x60'
        self.STOP_COMMAND = b'\xA5\x65'
        self.INFO_COMMAND = b'\xA5\x90'
        
    def find_lidar_port(self):
        """Find the YDLidar USB port automatically"""
        print("Searching for YDLidar G2...")
        ports = serial.tools.list_ports.comports()
        
        for port in ports:
            print(f"Found port: {port.device} - {port.description}")
            # YDLidar usually shows up as CP210x or similar
            if any(keyword in port.description.lower() for keyword in ['cp210', 'ydlidar', 'silicon labs']):
                return port.device
            # On macOS, it might show up as /dev/cu.SLAB_USBtoUART or similar
            if 'SLAB' in port.device or 'usbserial' in port.device:
                return port.device
                
        # If not found automatically, try common port names
        if sys.platform == 'linux':
            # For Raspberry Pi/Ubuntu
            possible_ports = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', '/dev/ttyACM1']
        elif sys.platform == 'darwin':
            # For macOS
            possible_ports = ['/dev/cu.SLAB_USBtoUART', '/dev/cu.usbserial-0001']
        else:
            possible_ports = []
            
        for port in possible_ports:
            try:
                test_serial = serial.Serial(port, self.LIDAR_BAUDRATE, timeout=0.5)
                test_serial.close()
                print(f"Found lidar at: {port}")
                return port
            except:
                continue
                
        return None
        
    def connect(self):
        """Connect to the lidar sensor"""
        self.port = self.find_lidar_port()
        
        if not self.port:
            print("ERROR: Could not find YDLidar. Please check:")
            print("1. The lidar is connected via USB-C")
            print("2. The lidar has power (LED should be on)")
            print("3. You have proper permissions (try 'sudo' on Linux)")
            return False
            
        try:
            print(f"Connecting to {self.port} at {self.LIDAR_BAUDRATE} baud...")
            self.serial_connection = serial.Serial(
                port=self.port,
                baudrate=self.LIDAR_BAUDRATE,
                timeout=1.0,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            
            # Clear any existing data
            self.serial_connection.reset_input_buffer()
            print("Successfully connected to YDLidar G2!")
            return True
            
        except Exception as e:
            print(f"Failed to connect: {str(e)}")
            if "Permission" in str(e):
                print("\nTry running with sudo: 'sudo python3 ydlidar_test.py'")
                print("Or add your user to dialout group: 'sudo usermod -a -G dialout $USER'")
            return False
            
    def get_device_info(self):
        """Get device information"""
        if not self.serial_connection:
            return
            
        try:
            print("\nGetting device info...")
            self.serial_connection.write(self.INFO_COMMAND)
            time.sleep(0.1)
            
            response = self.serial_connection.read(20)
            if len(response) > 0:
                print(f"Device response ({len(response)} bytes): {response.hex()}")
            else:
                print("No device info received (this is normal for some G2 models)")
                
        except Exception as e:
            print(f"Error getting device info: {e}")
            
    def start_scan(self):
        """Start the lidar scanning"""
        if not self.serial_connection:
            return False
            
        try:
            print("\nStarting scan...")
            self.serial_connection.write(self.SCAN_COMMAND)
            time.sleep(0.1)
            self.running = True
            return True
        except Exception as e:
            print(f"Failed to start scan: {e}")
            return False
            
    def stop_scan(self):
        """Stop the lidar scanning"""
        if self.serial_connection:
            try:
                self.serial_connection.write(self.STOP_COMMAND)
                time.sleep(0.1)
                self.running = False
                print("\nScan stopped")
            except:
                pass
                
    def read_scan_data(self):
        """Read and display scan data"""
        if not self.serial_connection or not self.running:
            return
            
        print("\nReading scan data (press Ctrl+C to stop)...")
        print("Format: Angle (degrees) | Distance (mm) | Quality")
        print("-" * 50)
        
        sample_count = 0
        start_time = time.time()
        
        try:
            while self.running:
                # Read data byte by byte until we find start sequence
                byte = self.serial_connection.read(1)
                if not byte:
                    continue
                    
                if byte == b'\xA5':
                    # Possible start of packet
                    next_byte = self.serial_connection.read(1)
                    if next_byte == b'\x5A':
                        # Valid start sequence found
                        # Read packet header (5 bytes already read 2)
                        header = self.serial_connection.read(3)
                        if len(header) == 3:
                            sample_count += 1
                            
                            # Parse basic data (simplified for G2)
                            # Actual parsing may vary based on G2 protocol
                            if sample_count % 40 == 0:  # Print every 40th sample
                                angle = (sample_count * 0.9) % 360  # Approximate
                                distance = int.from_bytes(header[:2], 'little')
                                quality = header[2]
                                
                                if distance > 0:  # Valid measurement
                                    print(f"{angle:6.1f}° | {distance:6d} mm | {quality:3d}")
                                    
                            # Show activity indicator
                            if sample_count % 360 == 0:
                                elapsed = time.time() - start_time
                                print(f"\n[{elapsed:.1f}s] Processed {sample_count} samples...")
                                
        except KeyboardInterrupt:
            print("\n\nScan interrupted by user")
        except Exception as e:
            print(f"\nError reading scan data: {e}")
            
    def run_test(self):
        """Run the complete test sequence"""
        print("YDLidar G2 Test Script")
        print("=" * 50)
        
        # Connect to lidar
        if not self.connect():
            return
            
        # Get device info
        self.get_device_info()
        
        # Start scanning
        if self.start_scan():
            # Read scan data
            self.read_scan_data()
            
        # Stop scanning
        self.stop_scan()
        
        # Cleanup
        if self.serial_connection:
            self.serial_connection.close()
            print("\nConnection closed")
            
def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    print("\n\nShutting down...")
    sys.exit(0)

def main():
    # Set up signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    # Check if we need to install pyserial
    try:
        import serial
    except ImportError:
        print("pyserial not installed. Please install it:")
        print("  pip3 install pyserial")
        print("  or")
        print("  sudo apt-get install python3-serial  (on Ubuntu)")
        sys.exit(1)
        
    # Create and run test
    tester = YDLidarG2Test()
    tester.run_test()
    
    print("\nTest complete!")
    print("\nIf the test was successful, you should have seen:")
    print("1. Successful connection message")
    print("2. Scan data with angles and distances")
    print("\nIf not working, check:")
    print("1. USB-C cable is properly connected")
    print("2. Lidar LED is on (indicating power)")
    print("3. You have proper permissions (use sudo if needed)")
    print("4. No other program is using the serial port")

if __name__ == "__main__":
    main()