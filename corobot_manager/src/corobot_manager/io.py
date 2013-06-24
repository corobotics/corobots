import socket
import traceback
from asynchat import async_chat, simple_producer
from asyncore import dispatcher

class CorobotServer(dispatcher):

    def __init__(self, port, manager):
        dispatcher.__init__(self)
        self.manager = manager
        self.handler = None
        self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
        # Allows a socket still in the TIME_WAIT state to be used.
        self.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.bind(("0.0.0.0", port))
        self.listen(1)

    def handle_error(self):
        traceback.print_exc()
        self.manager.shutdown()

    def handle_accept(self):
        sock, addr = self.accept()
        if self.handler is None:
            self.handler = CorobotHandler(self, sock)
        else:
            # Can only have one client connected at a time.
            sock.sendall("INUSE\n")
            sock.close()

class LineHandler(async_chat):

    """Reads lines and calls a callback with them."""

    def __init__(self, sock, line_read):
        async_chat.__init__(self, sock=sock)
        self.line_read = line_read
        self.received_data = []
        self.set_terminator("\n")

    def write_line(self, msg):
        self.producer_fifo.append(simple_producer(msg + "\n"))

    def collect_incoming_data(self, data):
        self.received_data.append(data)

    def found_terminator(self):
        line = "".join(self.received_data)
        self.line_read(line)
        self.received_data = []

    def handle_error(self):
        traceback.print_exc()
        self.handle_close()

class CorobotHandler(LineHandler):

    """Customize LineHandler for Corobot specific things."""

    def __init__(self, server, sock):
        LineHandler.__init__(self, sock, server.manager.handle_command)
        self.server = server

    def handle_close(self):
        self.server.handler = None
        self.close()
