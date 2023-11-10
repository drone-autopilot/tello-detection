import socket

class Command:

    def connect(self, ip: str, port: int, buf_size: int, debug: bool):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((ip, port))
        self.buf_size = buf_size
        self.debug = debug

    def send(self, cmd: str):
        if self.debug:
            print(f"<---{cmd}--->")
        else:
            self.socket.send(cmd.encode('utf-8'))
            print("[*]Sended a command : {}".format(cmd))
            if(self.buf_size is not None):
                response = self.socket.recv(self.buf_size)
                print("[*]Received a response : {}".format(response))