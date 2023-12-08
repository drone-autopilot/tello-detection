import socket

class Command:

    def connect(self, ip: str, port: int, buf_size: int, debug: bool):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((ip, port))
        self.buf_size = buf_size
        self.debug = debug

    def send(self, cmd: str, skip: bool = False):
        if self.debug:
            #print(f"<---{cmd}--->")
            self.socket.send("command".encode('utf-8'))
            if(self.buf_size is not None):
                response = self.socket.recv(self.buf_size)
                #print("[*]Received a response : {}".format(response))
        else:
            self.socket.send(cmd.encode('utf-8'))
            print("[*]Sended a command : {}".format(cmd))
            if(not skip):
                if(self.buf_size is not None):
                    response = self.socket.recv(self.buf_size)
                    print("[*]Received a response : {}".format(response))