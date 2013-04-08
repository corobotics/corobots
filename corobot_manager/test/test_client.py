import socket

def main():
    host = socket.gethostbyname(socket.gethostname())
    print(host)
    s = socket.socket()
    s.connect((host, 15001))
    out = s.makefile("w")
    out.write('1 GETPOS\n')
    out.write('2 GOTOLOC VENDDOOR\n')
    out.write('3 NAVTOLOC VENDDOOR\n')
    out.write('4 NAVTOXY 74.3904 36.6048\n')
    #Should return an ERROR condition
    out.write('5 NAVTOLOC NOWHERE\n')
    s.close()

if __name__ == '__main__':
    main()
