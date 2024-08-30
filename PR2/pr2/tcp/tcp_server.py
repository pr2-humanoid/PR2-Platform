import socket
import threading

from pr2.tcp.utils import bin2int, int2bin


class TcpServer:
    def __init__(self, host, port):
        self.host_ = host
        self.port_ = port

        self.sock_ = None

        self.quit_event_ = threading.Event()

    def launch(self):
        print(f"Server launched at {self.host_}:{self.port_}")

        self.sock_ = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock_.bind((self.host_, self.port_))
        self.sock_.listen(1)

        self.quit_event_.clear()

        self.start_server_()

    def stop(self):
        self.quit_event_.set()

        if self.sock_ is not None:
            socket.socket(socket.AF_INET, socket.SOCK_STREAM).connect(
                (self.host_, self.port_)
            )
            self.sock_.close()
            self.sock_ = None

    def start_server_(self):
        # listen to connection request
        while not self.quit_event_.is_set():
            # blocked for next connection
            conn, addr = self.sock_.accept()
            thread = threading.Thread(target=self.handle_connection_, args=(conn, addr))
            thread.start()

    # This function need to be override by its child class
    def handle_connection_(self, conn, addr):
        conn_id = f"{addr[0]}:{addr[1]}"
        print(f"New connection from {conn_id}")

        while not self.quit_event_.is_set():
            pack_size = conn.recv(4)

            # end of Connection
            if not pack_size:
                break

            pack_size = bin2int(pack_size)
            # fetch data package
            data_bin = self.recv_all_(conn, pack_size)

            print(f"Message from client: {data_bin}")

            response = b"hello from server"
            pack_size = int2bin(len(response))
            # send back response
            conn.sendall(pack_size + response)

        conn.close()
        print(f"Connection {conn_id}: closed")

    def recv_all_(self, sock, msg_length):
        data = b""
        size_left = msg_length

        while len(data) < msg_length and not self.quit_event_.is_set():
            recv_data = sock.recv(size_left)
            size_left -= len(recv_data)
            data += recv_data

        return data


if __name__ == "__main__":
    server = TcpServer(host="0.0.0.0", port=8800)
    server.launch()
