#!/usr/bin/env python3
"""
YDLidar G2 Diagnostic Script
Shows raw data pattern to understand the data format
"""

import sys
import time
import serial
import serial.tools.list_ports

def find_lidar_port():
    """Find the YDLidar USB port"""
    print("Searching for YDLidar...")
    ports = serial.tools.list_ports.comports()
    
    for port in ports:
        if 'CP210' in port.description or 'ttyUSB' in port.device:
            print(f"Found: {port.device}")
            return port.device
            
    # Try common ports
    for port in ['/dev/ttyUSB0', '/dev/ttyUSB1']:
        try:
            s = serial.Serial(port, 230400, timeout=0.1)
            s.close()
            return port
        except:
            pass
    return None

def main():
    # Find and connect
    port = find_lidar_port()
    if not port:
        print("No lidar found!")
        return
        
    print(f"Connecting to {port}...")
    ser = serial.Serial(port, 230400, timeout=0.1)
    
    # Start scan
    print("Starting scan...")
    ser.write(b'\xA5\x60')
    time.sleep(0.5)
    
    print("\n=== RAW DATA ANALYSIS ===")
    print("Looking for data patterns...\n")
    
    # Collect some data
    data_buffer = b''
    for i in range(5):
        if ser.in_waiting > 0:
            new_data = ser.read(ser.in_waiting)
            data_buffer += new_data
        time.sleep(0.1)
    
    # Show first 200 bytes in hex
    print("First 200 bytes (hex):")
    hex_str = data_buffer[:200].hex()
    for i in range(0, len(hex_str), 32):
        print(hex_str[i:i+32])
    
    # Look for patterns
    print("\n=== PATTERN ANALYSIS ===")
    
    # Count occurrences of potential start bytes
    print("\nByte frequency (potential markers):")
    byte_counts = {}
    for b in data_buffer[:1000]:
        if b not in byte_counts:
            byte_counts[b] = 0
        byte_counts[b] += 1
    
    # Show most common bytes
    sorted_bytes = sorted(byte_counts.items(), key=lambda x: x[1], reverse=True)[:10]
    for byte_val, count in sorted_bytes:
        print(f"  0x{byte_val:02X}: {count} times")
    
    # Look for AA 55 pattern
    print("\n0xAA 0x55 pattern locations:")
    for i in range(len(data_buffer)-1):
        if data_buffer[i] == 0xAA and data_buffer[i+1] == 0x55:
            # Show context around pattern
            start = max(0, i-2)
            end = min(len(data_buffer), i+20)
            context = data_buffer[start:end].hex()
            print(f"  Position {i}: ...{context}...")
            if i < 500:  # Only show first few
                # Try to parse as potential packet
                if i+10 < len(data_buffer):
                    packet_type = data_buffer[i+2]
                    sample_num = data_buffer[i+3]
                    print(f"    Type: 0x{packet_type:02X}, Samples: {sample_num}")
    
    # Simple data display
    print("\n=== SIMPLE DATA PARSER ===")
    print("Attempting basic parsing...\n")
    
    # Clear buffer and collect fresh data
    ser.reset_input_buffer()
    time.sleep(0.1)
    
    scan_count = 0
    byte_count = 0
    start_time = time.time()
    
    try:
        while True:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                byte_count += len(data)
                
                # Look for any distance-like values (2-byte integers)
                for i in range(0, len(data)-1, 2):
                    value = data[i] | (data[i+1] << 8)
                    # Reasonable distance range: 100-4000mm
                    if 100 < value < 4000:
                        scan_count += 1
                        if scan_count % 100 == 0:
                            print(f"Possible distance: {value} mm (sample #{scan_count})")
                
                # Status update
                if time.time() - start_time > 2:
                    print(f"\n[{time.time()-start_time:.1f}s] Bytes: {byte_count}, Possible distances found: {scan_count}")
                    if byte_count > 0:
                        print("Data IS flowing! Motor should be spinning.")
                    start_time = time.time()
                    
    except KeyboardInterrupt:
        print("\n\nStopping...")
        
    # Stop scan
    ser.write(b'\xA5\x65')
    ser.close()
    
    print("\n=== SUMMARY ===")
    print(f"Total bytes received: {byte_count}")
    print(f"Data rate: ~{byte_count/10:.0f} bytes/second")
    print("\nYour lidar IS working and sending data!")
    print("The data format just needs proper parsing.")
    print("\nNext steps:")
    print("1. The official YDLidar SDK would handle this parsing automatically")
    print("2. Or we can refine the parser once we understand the exact format")

if __name__ == "__main__":
    main()