import socket

def send_navigation_request(ip, port, x, y, z):
    # Create a socket object
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    try:
        # Connect to the ROS2 action client TCP server
        client_socket.connect((ip, port))

        # Prepare the message in the format "x,y,z"
        message = f"{x},{y},{z}"

        print("Send start")
        # Send the message
        client_socket.sendall(message.encode('utf-8'))
        print("Send end")

        # Receive acknowledgment (optional)
        data = client_socket.recv(1024)
        print(f"Received from server: {data.decode('utf-8')}")

    except Exception as e:
        print(f"Error: {e}")

    finally:
        # Close the socket connection
        client_socket.close()

if __name__ == "__main__":
    # Send navigation request to the ROS 2 action client
    send_navigation_request("127.0.0.1", 12345, 3.5, 2.0, 0.0)
