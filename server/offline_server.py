import socket


class Server:
    def __init__(self):
        self.host = '192.168.1.107'
        self.port = 3000
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.bind((self.host, self.port))

    def print_starting_msg(self):
        print("\n -------------------------------------")
        print("| AI Touch Interface Offline Server Started")
        print("|")
        print("| IP Address: " + str(self.host))
        print("| Port: " + str(self.port))
        print(" -------------------------------------\n")

    def run(self):
        self.print_starting_msg()

        while True:
            data, address = self.s.recvfrom(1024)
            data = data.decode()
            if data == 'c':
                self.s.sendto("5".encode('utf-8'), address)
            if data != "none":
                print(data)
            self.s.sendto("none".encode('utf-8'), address)


if __name__ == '__main__':
    Server().run()
