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
            self.socket.send("commandA".encode('utf-8'))
            if(self.buf_size is not None):
                response = self.socket.recv(self.buf_size)
                #print("[*]Received a response : {}".format(response))
        else:
            self.socket.send(f"{cmd}A".encode('utf-8'))
            print(f"[*]Sended a command : {cmd}")
            if(not skip):
                if(self.buf_size is not None):
                    response = self.socket.recv(self.buf_size)
                    print("[*]Received a response : {}".format(response))
                    if(response == b"ok"):
                        return True
                    else:
                        return False