from pr2.tcp.tcp_client import TcpClient
from pr2.tcp.utils import int2bin


class PersistentTcpClient(TcpClient):
    def __init__(self, ip, port):
        super().__init__(ip, port)
        self.connect()

    def __del__(self):
        self.close()

    def connect(self):
        self.sock_ = self.create_connection()

    def close(self):
        if self.sock_ is None:
            return

        # send Exit signal to server
        try:
            self.send(b"")
        except Exception:  # pylint: disable=broad-exception-caught
            pass
        self.sock_.close()
        self.sock_ = None

    def send(self, data_bin):
        pack_size = int2bin(len(data_bin))

        try:
            self.sock_.sendall(pack_size + data_bin)
            data_bin = self.recv_response_(self.sock_)
        except Exception:  # pylint: disable=broad-exception-caught
            # trying to reconnect to server
            self.connect()

        return data_bin
