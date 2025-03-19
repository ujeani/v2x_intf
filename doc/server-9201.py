# 이 프로그램을 AWS에서 실행하고 ROS2 노드와 통신하도록 하여 패킷 송수신을 테스트할 수 있다.
# AWS 정보
#   obu-ip : 3.34.3.66
#   obu-port : 9201 and 9202
# ROS2 노드 실행
#  각각의 디바이스에서 실행 (9201, 9202 포트로 접속)
#  $ ros2 run v2x_intf_pkg v2x_intf_node --obu-ip 3.34.3.66 --obu-port 9201
#  $ ros2 run v2x_intf_pkg v2x_intf_node --obu-ip 3.34.3.66 --obu-port 9202

import socket
import threading
import time

# Server configuration
HOST = '0.0.0.0'
PORT1 = 9201
PORT2 = 9202
IDLE_TIMEOUT = 300  # Optional timeout (in seconds)

lock = threading.Lock()
client1 = None
client2 = None
forward_threads = []
last_activity = {
    "client1" : time.time(),
    "client2" : time.time()
}

def forward(src, dst, label):
    try:
        while True:
            data = src.recv(8192)
            if not data:
                print(f"[!] {label} - No data, terminating forwarding.")
                break
            dst.sendall(data)
            print(f"{label} received {len(data)} bytes")
            if not data:
                break
            dst.sendall(data)

            # Update activity timestamp
            with lock:
                if label.startwith("Client1"):
                    last_activity["client1"] = time.time()
                else :
                    last_activity["client2"] = time.time()

    except Exception as e:
        print(f"[!] forwarding error: {e}")
    finally:
        disconnect_pair(label)


def disconnect_pair(caller=None):
    global client1, client2, forward_threads

    with lock:
        print(f"[✂] Disconnect triggered by: {caller}")
        try:
            if client1:
                client1.shutdown(socket.SHUT_RDWR)
                client1.close()
        except:
            pass
        try:
            if client2:
                client2.shutdown(socket.SHUT_RDWR)
                client2.close()
        except:
            pass

        client1 = None
        client2 = None
        forward_threads = []


def handle_connection(client_socket, client_id):
    global client1, client2

    with lock:
        if client_id == 1:
            if client1:
                print("[*] Client1 reconnecting. Cleaning old state.")
                disconnect_pair("Client1 reconnect")
            client1 = client_socket
            last_activity["client1"] = time.time()
            print("[+] Client1 ready.")
        elif client_id == 2:
            if client2:
                print("[*] Client2 reconnecting. Cleaning old state.")
                disconnect_pair("Client2 reconnect")
            client2 = client_socket
            last_activity["client2"] = time.time()
            print("[+] Client2 ready.")

        # Start forwarding only when both clients are connected and clean
        if client1 and client2 and len(forward_threads) == 0:
            print("[✓] Both clients connected. Starting forwarding...")
            t1 = threading.Thread(target=forward, args=(client1, client2, "Client1→Client2"), daemon=True)
            t2 = threading.Thread(target=forward, args=(client2, client1, "Client2→Client1"), daemon=True)
            t1.start()
            t2.start()
            forward_threads = [t1, t2]


def listen_on_port(port, client_id):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, port))
    server_socket.listen()
    print(f"[🔌] Listening on port {port} for Client{client_id}...")

    while True:
        try:
            client_socket, addr = server_socket.accept()
            print(f"[📥] Client{client_id} connected from {addr}")

            # Optional: Flush socket buffer on new connection
            try:
                client_socket.settimeout(0.1)
                while True:
                    if not client_socket.recv(1024):
                        break
            except:
                pass
            finally:
                client_socket.settimeout(None)

            threading.Thread(target=handle_connection, args=(client_socket, client_id), daemon=True).start()
        except Exception as e:
            print(f"[!] Error accepting on port {port}: {e}")

def log_status():
    while True:
        time.sleep(10)
        with lock:
            c1 = "Connected" if client1 else "Waiting"
            c2 = "Connected" if client2 else "Waiting"
            print(f"[⏱ STATUS] Client1: {c1} | Client2: {c2}")

            now = time.time()
            if client1 and now - last_activity["client1"] > IDLE_TIMEOUT:
                print("[!] Client1 idle timeout. Disconnecting.")
                disconnect_pair("Client1 timeout")
            if client2 and now - last_activity["client2"] > IDLE_TIMEOUT:
                print("[!] Client2 idle timeout. Disconnecting.")
                disconnect_pair("Client2 timeout")

def main():
    threading.Thread(target=listen_on_port, args=(PORT1, 1), daemon=True).start()
    threading.Thread(target=listen_on_port, args=(PORT2, 2), daemon=True).start()
    threading.Thread(target=log_status, daemon=True).start()

    print("[🚀] Port Forwarder is running...")
    while True:
        time.sleep(10)

if __name__ == "__main__":
    main()
