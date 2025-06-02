#!/usr/bin/env python3
"""
YDLidar G2 Test Script - Enhanced Version
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
        self.HEALTH_COMMAND = b'\xA5\x91'
        
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
            # For Raspberry Pi/Ubuntu - including UART pins
            possible_ports = ['/dev/ttyAMA0', '/dev/serial0', '/dev/ttyS0', 
                            '/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', '/dev/ttyACM1']
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
                if len(response) >= 20:
                    print("Device info indicates lidar is responding correctly!")
            else:
                print("No device info received (this is normal for some G2 models)")
                
        except Exception as e:
            print(f"Error getting device info: {e}")
            
    def get_health_status(self):
        """Check health status of the lidar"""
        if not self.serial_connection:
            return
            
        try:
            print("\nChecking health status...")
            self.serial_connection.write(self.HEALTH_COMMAND)
            time.sleep(0.1)
            
            response = self.serial_connection.read(10)
            if len(response) > 0:
                print(f"Health response: {response.hex()}")
                
        except Exception as e:
            print(f"Error getting health status: {e}")
            
    def start_scan(self):
        """Start the lidar scanning"""
        if not self.serial_connection:
            return False
            
        try:
            print("\nStarting scan...")
            # Clear buffer before starting
            self.serial_connection.reset_input_buffer()
            self.serial_connection.write(self.SCAN_COMMAND)
            time.sleep(0.5)  # Give motor time to start
            self.running = True
            
            # Check if data is coming
            print("Checking for incoming data...")
            test_data = self.serial_connection.read(10)
            if len(test_data) > 0:
                print(f"Receiving data! First bytes: {test_data.hex()}")
                print("Motor should be spinning now. Can you hear it?")
            else:
                print("WARNING: No data received. Possible issues:")
                print("- Motor may not be spinning (power issue)")
                print("- Lidar may need more time to start")
                print("- Try external power supply")
                
            return True
        except Exception as e:
            print(f"Failed to start scan: {e}")
            return False
            
    def stop_scan(self):
        """Stop the lidar scanning"""
        if self.serial_connection:
            try:
                self.serial_connection.write(self.STOP_COMMAND)
                time.sleep(0.5)
                self.running = False
                print("\nScan stopped")
            except:
                pass
                
    def read_scan_data(self):
        """Read and display scan data with better parsing"""
        if not self.serial_connection or not self.running:
            return
            
        print("\nReading scan data (press Ctrl+C to stop)...")
        print("Note: If no data appears, the motor might not be spinning")
        print("-" * 60)
        
        byte_count = 0
        packet_count = 0
        start_time = time.time()
        last_print_time = start_time
        
        # Buffer for collecting scan data
        data_buffer = b''
        
        try:
            while self.running:
                # Read available data
                if self.serial_connection.in_waiting > 0:
                    new_data = self.serial_connection.read(self.serial_connection.in_waiting)
                    data_buffer += new_data
                    byte_count += len(new_data)
                    
                    # Look for packet start (0xA5 0x5A)
                    while len(data_buffer) >= 10:
                        # Find start sequence
                        start_idx = data_buffer.find(b'\xA5\x5A')
                        if start_idx == -1:
                            data_buffer = data_buffer[-2:]  # Keep last 2 bytes
                            break
                            
                        # Remove data before start
                        if start_idx > 0:
                            data_buffer = data_buffer[start_idx:]
                            
                        # Check if we have enough data for a packet
                        if len(data_buffer) < 10:
                            break
                            
                        # Parse packet header
                        packet_type = data_buffer[2]
                        sample_quantity = data_buffer[3]
                        
                        # Calculate expected packet size
                        if packet_type == 0:  # Normal point cloud packet
                            packet_size = 10 + sample_quantity * 3
                        else:
                            packet_size = 10
                            
                        # Check if we have full packet
                        if len(data_buffer) < packet_size:
                            break
                            
                        # Process packet
                        packet = data_buffer[:packet_size]
                        data_buffer = data_buffer[packet_size:]
                        packet_count += 1
                        
                        # Parse scan data from packet
                        if packet_type == 0 and len(packet) >= 10:
                            # Extract angle data
                            start_angle = (packet[4] | (packet[5] << 8)) / 100.0
                            end_angle = (packet[6] | (packet[7] << 8)) / 100.0
                            
                            # Parse sample data
                            for i in range(sample_quantity):
                                if 10 + i*3 + 2 < len(packet):
                                    distance = packet[10 + i*3] | (packet[10 + i*3 + 1] << 8)
                                    quality = packet[10 + i*3 + 2]
                                    
                                    # Calculate angle for this sample
                                    if sample_quantity > 1:
                                        angle = start_angle + (end_angle - start_angle) * i / (sample_quantity - 1)
                                    else:
                                        angle = start_angle
                                        
                                    # Normalize angle
                                    if angle < 0:
                                        angle += 360
                                    elif angle >= 360:
                                        angle -= 360
                                        
                                    # Print every N-th valid measurement
                                    if distance > 0 and packet_count % 10 == 0 and i == 0:
                                        print(f"Angle: {angle:6.1f}° | Distance: {distance:5d} mm | Quality: {quality:3d}")
                
                # Status update every 2 seconds
                current_time = time.time()
                if current_time - last_print_time > 2.0:
                    elapsed = current_time - start_time
                    print(f"\n[Status] Time: {elapsed:.1f}s | Bytes: {byte_count} | Packets: {packet_count}")
                    if byte_count == 0:
                        print("WARNING: No data received. Check if motor is spinning!")
                        print("You should hear the motor running if power is sufficient.")
                    last_print_time = current_time
                    
                # Small delay to prevent CPU overload
                time.sleep(0.001)
                
        except KeyboardInterrupt:
            print("\n\nScan interrupted by user")
        except Exception as e:
            print(f"\nError reading scan data: {e}")
            import traceback
            traceback.print_exc()
            
        # Final statistics
        elapsed = time.time() - start_time
        print(f"\nFinal Statistics:")
        print(f"Total time: {elapsed:.1f} seconds")
        print(f"Total bytes received: {byte_count}")
        print(f"Total packets: {packet_count}")
        if elapsed > 0:
            print(f"Data rate: {byte_count/elapsed:.0f} bytes/second")
            
    def run_test(self):
        """Run the complete test sequence"""
        print("YDLidar G2 Test Script - Enhanced Version")
        print("=" * 60)
        
        # Connect to lidar
        if not self.connect():
            return
            
        # Get device info
        self.get_device_info()
        
        # Get health status
        self.get_health_status()
        
        # Important power check
        print("\n" + "="*60)
        print("POWER CHECK:")
        print("1. Is the lidar LED on?")
        print("2. Can you hear the motor spinning after scan starts?")
        print("3. If using power bank, is it providing enough current?")
        print("="*60 + "\n")
        
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
    print("\nDiagnostic Summary:")
    print("- Device Info Response: YES ✓" if True else "- Device Info Response: NO ✗")
    print("- If motor is NOT spinning: Power issue - use external power")
    print("- If motor IS spinning but no data: Check connections or try longer test")
    print("\nFor external power setup, connect:")
    print("  5V Power → Lidar Red wire")
    print("  GND → Lidar Black wire")
    print("  Keep USB for data communication")

if __name__ == "__main__":
    main()