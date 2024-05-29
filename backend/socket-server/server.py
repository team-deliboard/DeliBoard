import os
import json
import socket
import threading
import requests
from time import sleep
from dotenv import load_dotenv
from enum import Enum

class OrderType(Enum):
    GAME_ARRIVAL = 0
    GAME_RETURN = 1
    BEVERAGE_ARRIVAL = 2
    THEME_CHANGE = 3

# Get environs
load_dotenv()
SOCKET_PORT = int(os.getenv('SOCKET_PORT'))
SOCKET_HOST = os.getenv('SOCKET_HOST')

# key: S/N, value: connection
client_connections = {}
 
def addSizePrefix(message):
    size = len(message.encode('utf-8'))
    return f"{size:04d}{message}"

def getJsonGameOrder(message):
    try:
        json_order = json.loads(message)
        command = json_order.get('command', None)
        store_id = json_order.get('storeId', None)
        room_number = json_order.get('roomNumber', None)
        location_x = json_order.get('locationX', None)
        location_y = json_order.get('locationY', None)
        if (command and store_id and room_number and location_x and location_y):
            return json_order
    except:
        return None

def sendMsgToTurtle(message):
    try:
        turtle = client_connections.get('turtle')
        if not turtle:
            result = {'success': False, 'message': 'Fail: Turtle is not connected.'}
            print(result.get('message'))
            return result
        turtle.sendall(message.encode('utf-8'))
        result = {'success': True, 'message': 'Success: Message sent to turtle.'}
        print(result.get('message'))
        return result
    except:
        result = {'success': False, 'message': 'Fail: Error occurred while sending message to turtle.'}
        print(result.get('message'))
        return result

def sendMsgToPi(message):
    try:
        pi = client_connections.get('pi')
        if not pi:
            result = {'success': False, 'message': 'Fail: Pi is not connected.'}
            print(result.get('message'))
            return result
        pi.sendall(message.encode('utf-8'))
        result = {'success': True, 'message': 'Success: Message sent to pi.'}
        print(result.get('message'))
        return result
    except:
        result = {'success': False, 'message': 'Fail: Error occurred while sending message to pi.'}
        print(result.get('message'))
        return result


def sendMsgToClientAndClose(message, client_connection, close=True):
    try:
        client_connection.sendall(message.encode('utf-8'))
        if close:
            print('Server disconnected.')
            client_connection.close()
        result = {'success': True, 'message': 'Success: Message sent to client.'}
        print(result.get('message'))
        return result
    except:
        result = {'success': False, 'message': 'Fail: Error occurred while sending message to client.'}
        print(result.get('message'))
        return result

def getConnectionByType(type, server_socket):
    try:
        connection = client_connections.get(type)
        if not connection:
            result = {'success': False, 'message': f'Fail: {type} is not connected.'}
            print(result.get('message'))
            sendMsgToClientAndClose(result.get('message'), server_socket)
            return False
        return connection
    except:
        result = {'success': False, 'message': f'Fail: Error occurred while getting {type} connection.'}
        print(result.get('message'))
        sendMsgToClientAndClose(result.get('message'), server_socket)
        return False

def serverHandler(server_socket):
    print('Server connected.')
    order = server_socket.recv(1024).decode('utf-8')
    order_json = json.loads(order)
    order_type = order_json.get('command')
    print('Order type:', order_type)
    if order_type == OrderType.THEME_CHANGE.value:
        theme = order_json.get('theme')
        print('Theme:', theme)
        result = sendMsgToPi(str(theme))
        sendMsgToClientAndClose(result.get('message'), server_socket)
    elif order_type == OrderType.GAME_ARRIVAL.value or order_type == OrderType.GAME_RETURN.value or order_type == OrderType.BEVERAGE_ARRIVAL.value:
        result = sendMsgToTurtle(order)
        # Set default theme if command is 'return'
        if order_json.get("command") == OrderType.GAME_RETURN.value:
            theme = 'default'
            result = sendMsgToPi(theme)
        # Send response to client
        sendMsgToClientAndClose("Success", server_socket)

def sendNotification(message):
    message = json.loads(message)
    command = int(message.get('command', None))
    store_id = message.get('storeId', None)
    room_number = message.get('roomNumber', None)
    json_data = {
        'message': command,
        'storeId': store_id,
        'roomNumber': room_number
    }
    url = "https://j10a210.p.ssafy.io/api/game-order/notification"
    requests.post(url, json=json_data)
    

def clientHandler(client_socket, type):
    # keep listening for incoming data
    while True:
        try:
            message = client_socket.recv(1024).decode('utf-8')
        except:
            # Unexpected error
            client_socket.close()
            del(client_connections[type])
            break

        if message:
            print(message)
            sendNotification(message)
        else:
            client_socket.close()
            del(client_connections[type])
            break

def acceptConnections(server_socket):
    while True:
        client_socket, client_address = server_socket.accept()
        try:
            client_type = client_socket.recv(1024).decode('utf-8')
        except:
            print('Error occurred while receiving client type. Connection closed.')
            client_socket.close()
            continue
        print(f"Connection from {client_address} with type {client_type}.")

        if client_type == 'server':
            serverHandler(client_socket)
            continue

        # Invalid client type
        if client_type not in ['pi', 'turtle']:
            client_socket.close()
            continue

        client_connections[client_type] = client_socket

        # start clientHandler and sender threads
        if client_type == 'turtle':
            clientHandler_thr = threading.Thread(target=clientHandler, args=(client_socket, client_type))
            clientHandler_thr.daemon = True
            clientHandler_thr.start()


def startServer(host, port):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((host, port))
    server_socket.listen(5)
    acceptConnections(server_socket)

startServer(SOCKET_HOST, SOCKET_PORT)