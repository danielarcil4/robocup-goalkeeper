import socket
import struct

MCAST_GRP = '224.5.23.2'
MCAST_PORT = 10006

# Crear socket UDP
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

# Permitir que varias aplicaciones escuchen en el mismo puerto
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

# En sistemas Linux suele funcionar enlazar a '' (todas las interfaces)
sock.bind(('', MCAST_PORT))

# Indicarle que se una al grupo multicast
mreq = struct.pack("4sl", socket.inet_aton(MCAST_GRP), socket.INADDR_ANY)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

print("Esperando datos...")

while True:
    data, addr = sock.recvfrom(1024)
    print(f"Recibido {data} desde {addr}")
