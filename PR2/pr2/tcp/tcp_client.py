import json
import socket
import time

from pr2.tcp.utils import bin2int, int2bin


class TcpClient:
    def __init__(self, ip, port):
        self.ip_ = ip
        self.port_ = port
        self.sock_ = None

    def send(self, data_bin):
        sock = self.create_connection()

        pack_size = int2bin(len(data_bin))
        sock.sendall(pack_size + data_bin)
        data_bin = self.recv_response_(sock)

        sock.close()

        return data_bin

    def create_connection(self):
        sec = 0

        while True:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.connect((self.ip_, self.port_))
                return sock
            except Exception:  # pylint: disable=broad-exception-caught
                time.sleep(1.0)
                print(f"Waiting for server {sec}s ...", end="\r")
                sec += 1

    def recv_response_(self, sock):
        pack_size = sock.recv(4)

        if len(pack_size) == 0:
            return b""

        pack_size = bin2int(pack_size)
        # fetch data package
        data_bin = self.recv_all_(sock, pack_size)

        return data_bin

    def recv_all_(self, sock, msg_length):
        data = b""
        size_left = msg_length

        while len(data) < msg_length:
            recv_data = sock.recv(size_left)
            size_left -= len(recv_data)
            data += recv_data

        return data


def main():
    client = TcpClient(ip="0.0.0.0", port=8800)
    data = {"joint_name": "elbow", "position": 1.23}
    client.send(json.dumps(data).encode("ascii"))


if __name__ == "__main__":
    main()
