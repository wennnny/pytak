import socket
import struct

UDP_IP = "0.0.0.0"
UDP_PORT = 49152

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening for UDP packets on {UDP_IP}:{UDP_PORT}...")

def parse_gps_packet(data):
    try:
        header, length, seq, stamp, lat, lon, endcode = struct.unpack('<BBIdddB', data)
        print(f"[GPS] Seq: {seq}, Time: {stamp:.3f}, Lat: {lat:.6f}, Lon: {lon:.6f}")
    except struct.error as e:
        print(f"[GPS] Unpack error: {e}")

def parse_vel_packet(data):
    try:
        header, length, seq, linear_x, endcode = struct.unpack('<BBIdB', data)
        print(f"[VEL] Seq: {seq}, Linear X: {linear_x:.3f}")
    except struct.error as e:
        print(f"[VEL] Unpack error: {e}")

def parse_packet(data):
    if len(data) == 31 and data[0] == 0xAA:
        parse_gps_packet(data)
    elif len(data) == 15 and data[0] == 0xAB:
        parse_vel_packet(data)
    else:
        print(f"❌ Unknown or invalid packet: {data.hex()}")

try:
    while True:
        data, addr = sock.recvfrom(1024)
        parse_packet(data)
except KeyboardInterrupt:
    print("\n Interrupted by user. Exiting gracefully...")
finally:
    sock.close()
    print("Socket closed.")
