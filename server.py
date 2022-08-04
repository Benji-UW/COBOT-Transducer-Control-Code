from http import client
from pydoc import cli
import socket
import numpy as np
import time
from threading import Thread
import os
import logging

logger = logging.getLogger(__name__)
HOST = ''
PORT = 508

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen(6)

# HOST = "192.168.0.5"
# PORT = 50000

# s2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# s2.bind((HOST, PORT))
# print("listening...")
# s2.listen(2)

flag = 0
motion = b'gdfdhfjhgvj'
condition = 0
robot_state_message = b"no state"
shitty_sql = {b"loopback": 420}
todo_list = []

def client_thread(conn): 
    global flag
    global motion
    global condition
    global robot_state_message
    global shitty_sql
    global todo_list

    number = flag
    flag = flag + 1
    t = time.time()
    t_nm1 = t
    is_alive = True
    refresh_rate = np.zeros((250,))
    i_rr = 0
    while is_alive:
        client_input = conn.recv(4096)
        try:
            logger.info(client_input)
        except e:
            logger.error("Client input was not loggable :/")

        # print(str(client_input))
        # print(shitty_sql)
        t = time.time()
        if client_input == b'motion':
            # print('received motion:')
            # print(motion)
            conn.send(motion)
            logger.info(b"Reply: " + motion)
        # elif client_input[:3] == b"SET":
        # #     pass
        elif b'SET' in client_input:
            '''Should be a message in the format "SET <name> <value>", can only be an integer'''
            '''Might instead be a series of strings like that'''
            cli_input = client_input.split()
            for i in range(int(len(cli_input) / 3)):
                if cli_input[i*3] == b'SET':
                    var_name = cli_input[3*i + 1]
                    var_val = cli_input[3*i + 2]
            # The shitty SQL data gets stored as ints in a dictionary
                shitty_sql[var_name] = int(var_val)
        elif client_input[:3] == b"GET":
            '''Should be a message in the format "GET <name>", only sending integers tho'''
            cli_input = client_input.split()
            try:
                dat = cli_input[1] + b' %i' % (shitty_sql[cli_input[1]])
            except KeyError as e:
                dat = b'Key not found' 
            conn.send(dat)
            logger.info(b"Reply: " + dat)
        elif client_input[:4] == b"TODO":
            '''Makes a shity todo-list for passing tasks back and forth'''
            cli_input = client_input.split()
            if len(todo_list) < 5:
                # print(b"adding item to the todo list: " + cli_input[1])
                todo_list.append(cli_input[1])
                # print(todo_list)
        elif client_input[:4] == b'RSM:':
            robot_state_message = client_input[4:] + b" your server touched this :)))"
            # print(robot_state_message)
        elif client_input == b'RSR':
            conn.send(robot_state_message)
        elif b'I am alive' in client_input:
            if len(todo_list) != 0:
                a = todo_list.pop()
                print(b"sending todo item " + a)
                # time.sleep(0.01)
                conn.send(a)
                time.sleep(0.01)
            else:
                conn.send(b"Transmit position")
        elif client_input == b'end':
            print('Connection to process %d ended.' % number)
            conn.close()
            is_alive = False
        elif client_input[0] == 84:
            motion = client_input
            # print(client_input)
        else:
            # motion = client_input
            print(shitty_sql)
            # debugging type step:
            cl = str(client_input)
            # print('client ' + str(conn) + ' just sent ' + cl)
        if b'I am alive' in client_input:
            if len(todo_list) != 0:
                a = todo_list.pop()
                print(b"sending todo item " + a)
                # time.sleep(0.01)
                conn.send(a)
                time.sleep(0.01)
        if i_rr >= 250:
            i_rr = 0
            print('Refresh rate of process %d: %3.1f' % (number, (1. / np.mean(refresh_rate))))
            if (1. / np.mean(refresh_rate)) > 2000:
                conn.close()
                is_alive = False
        refresh_rate[i_rr] = t - t_nm1


        i_rr += 1
        t_nm1 = t


def start_server():
    while True:
        print("entered while loop")
        con, addr = s.accept()
        print('Connected: ' + str(addr))
        Thread(target=client_thread, args=(con,)).start()

        # print("waiting for s2 connection")
        # con, addr = s2.accept()
        # print('Connected on server 2 ' + str(addr))
        # Thread(target=client_thread, args=(con,)).start()


def test_import():
    print("You've successfully accessed a method within this file.")


if __name__ == "__main__":
    start_server()
else:
    print(__name__)
    print("server.py is being imported into another module")