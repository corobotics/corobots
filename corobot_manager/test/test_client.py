import socket

def main():
    host = socket.gethostbyname(socket.gethostname())
    print(host)
    s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    s.connect((host,15001))
    print("CONNECTED")
    
    s.sendall('GETPOS\n')
    data = s.recv(1024)
    print(data)
    
    s.sendall('GOTOLOC VENDDOOR\n')
    data = s.recv(1024)
    print(data)

    s.sendall('NAVTOLOC VENDDOOR\n')
    data = s.recv(1024)
    print(data)

    s.sendall('NAVTOXY 7.4128 13.0544\n')
    data = s.recv(1024)
    print(data)

    #Should return an ERROR condition
    s.sendall('NAVTOLOC NOWHERE\n')
    data = s.recv(1024)
    print(data)

    s.close()

if __name__ == '__main__':
    main()
