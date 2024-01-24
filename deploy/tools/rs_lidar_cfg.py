import socket

LIDAR_IP = "192.168.123.200"
PC_IP =    "192.168.123.221"
RPM = 600


def send_udp_payload(payload, ip, port):
    # Create a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        # Send the payload to the UDP node
        sock.sendto(payload, (ip, port))
        print("Payload sent successfully.")
    except socket.error as e:
        print(f"Error occurred while sending payload: {e}")
    finally:
        # Close the socket
        sock.close()


lidar_ip = socket.inet_aton(LIDAR_IP)
pc_ip =    socket.inet_aton(PC_IP)

payload_600RPM = bytes([0xAA,0x00,0xFF,0x11,0x22,0x22,0xAA,0xAA,              #Header
                        0x02, 0x58,                                            #Rotation Speed (MSB, LSB)
                        lidar_ip[0],lidar_ip[1],lidar_ip[2],lidar_ip[3],       #Lidar IP Address
                        pc_ip[0],pc_ip[1],pc_ip[2],pc_ip[3],                   #PC IP Address
                        0x00,0x1C,0x23, 0x17,0x4A,0xCC,                        #Device MAC Address
                        0x1A, 0x2B,                                            #MSOP Port1 (Data Stream)
                        0x1A, 0x2B,                                            #MSOP Port1 (Data Stream)
                        0x1E,0x6C,                                             #DIFOP Port1
                        0x1E,0x6C,                                             #DIFOP Port1
                        0x00, 0x00,                                            #FOV Starting Angle (0-36000)
                        0x8C, 0xA0,                                            #FOV End Angle (0-36000)
                        0x11,0x03,0x0A,0x09,0x2D,0x1E,0x00,0x64,0x00,0xC8,     #UTC Time (Year, Month, Day, Hour, Minute, Second, Millisecond, Microsecond)
                        0x00, 0x5A])                                           #Motor Phase Lock

payload_1200RPM = bytes([0xAA,0x00,0xFF,0x11,0x22,0x22,0xAA,0xAA,              #Header
                        0x04, 0xB0,                                            #Rotation Speed (MSB, LSB)
                        lidar_ip[0],lidar_ip[1],lidar_ip[2],lidar_ip[3],       #Lidar IP Address
                        pc_ip[0],pc_ip[1],pc_ip[2],pc_ip[3],                   #PC IP Address
                        0x00,0x1C,0x23, 0x17,0x4A,0xCC,                        #Device MAC Address
                        0x1A, 0x2B,                                            #MSOP Port1 (Data Stream)
                        0x1A, 0x2B,                                            #MSOP Port1 (Data Stream)
                        0x1E,0x6C,                                             #DIFOP Port1
                        0x1E,0x6C,                                             #DIFOP Port1
                        0x00, 0x00,                                            #FOV Starting Angle (0-36000)
                        0x8C, 0xA0,                                            #FOV End Angle (0-36000)
                        0x11,0x03,0x0A,0x09,0x2D,0x1E,0x00,0x64,0x00,0xC8,     #UTC Time (Year, Month, Day, Hour, Minute, Second, Millisecond, Microsecond)
                        0x00, 0x5A])                                           #Motor Phase Lock

port = 6699
if RPM == 600:
    print(f'RPM set to: 600, Lidar IP is set to: {LIDAR_IP}, PC IP is set to: {PC_IP}')
    send_udp_payload(payload_600RPM, LIDAR_IP, port)
else:
    print(f'RPM set to: 1200, Lidar IP is set to: {LIDAR_IP}, PC IP is set to: {PC_IP}')
    send_udp_payload(payload_1200RPM, LIDAR_IP, port)
