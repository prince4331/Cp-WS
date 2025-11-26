#!/usr/bin/env python3
"""
Quick Arduino Serial Test
Tests if Arduino is responding on serial port
"""
import serial
import time
import sys

PORT = '/dev/ttyACM0'
BAUD = 115200

print("╔════════════════════════════════════════════════════════════╗")
print("║         📡 ARDUINO SERIAL TEST 📡                          ║")
print("╚════════════════════════════════════════════════════════════╝")
print()

try:
    print(f"Opening {PORT} at {BAUD} baud...")
    ser = serial.Serial(PORT, BAUD, timeout=2)
    print("✅ Port opened")
    
    print("Waiting for Arduino boot...")
    time.sleep(2)
    
    print("Reading serial data for 5 seconds...")
    print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
    
    start_time = time.time()
    line_count = 0
    
    while time.time() - start_time < 5:
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"📥 {line}")
                    line_count += 1
            except Exception as e:
                print(f"⚠️  Decode error: {e}")
        time.sleep(0.01)
    
    print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
    print()
    
    if line_count > 0:
        print(f"✅ SUCCESS! Received {line_count} lines from Arduino")
        print("   Arduino is responding and sending data!")
    else:
        print("❌ NO DATA received from Arduino")
        print()
        print("Possible issues:")
        print("1. Arduino firmware not uploaded")
        print("2. Wrong baud rate (should be 115200)")
        print("3. Arduino is stuck or needs reset")
        print("4. USB cable issue")
        print()
        print("Try:")
        print("1. Press Arduino RESET button")
        print("2. Upload firmware: ./upload_arduino.sh")
    
    ser.close()
    sys.exit(0 if line_count > 0 else 1)
    
except serial.SerialException as e:
    print(f"❌ ERROR: {e}")
    print()
    print("Make sure:")
    print(f"1. Arduino is connected to {PORT}")
    print("2. No other program is using the port")
    print("3. You have permissions: sudo chmod 666 /dev/ttyACM0")
    sys.exit(1)
except KeyboardInterrupt:
    print("\n⚠️  Interrupted by user")
    sys.exit(1)
