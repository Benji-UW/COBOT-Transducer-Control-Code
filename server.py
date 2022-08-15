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

flag = 0
motion = b'T00Z000000000'
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
            logger.debug(client_input)
        except e:
            logger.error("Client input was not loggable :/")

        t = time.time()
        if client_input == b'motion':
            conn.send(motion)
            logger.debug(b"Reply: " + motion)
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
            logger.debug(f"Current data content: {shitty_sql}")
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
                todo_list.append(cli_input[1])
        elif client_input[:4] == b'RSM:':
            robot_state_message = client_input[4:] + b" your server touched this :)))"
        elif client_input == b'RSR':
            conn.send(robot_state_message)
        elif b'I am alive' in client_input:
            if len(todo_list) != 0:
                a = todo_list.pop()
                logger.info(b"sending todo item " + a)
                # time.sleep(0.01)
                conn.send(a)
                time.sleep(0.01)
            else:
                conn.send(b"Transmit position")
        elif client_input == b'end':
            logger.info('Connection to process %d ended.' % number)
            conn.close()
            is_alive = False
        elif client_input[0] == 84:
            motion = client_input
        else:
            # motion = client_input
            logger.debug(shitty_sql)
            # debugging type step:
            cl = str(client_input)
            logger.debug('client ' + str(conn) + ' just sent ' + cl)
        if b'I am alive' in client_input:
            if len(todo_list) != 0:
                a = todo_list.pop()
                logger.debug(b"sending todo item " + a)
                # time.sleep(0.01)
                conn.send(a)
                time.sleep(0.01)
        if i_rr >= 250:
            i_rr = 0
            logger.info('Refresh rate of process %d: %3.1f' % (number, (1. / np.mean(refresh_rate))))
            if (1. / np.mean(refresh_rate)) > 2000:
                conn.close()
                is_alive = False
        refresh_rate[i_rr] = t - t_nm1


        i_rr += 1
        t_nm1 = t


def start_server():
    while True:
        logger.info("entered while loop")
        con, addr = s.accept()
        logger.info('Connected: ' + str(addr))
        Thread(target=client_thread, args=(con,)).start()

def test_import():
    logger.info("You've successfully accessed a method within this file.")


if __name__ == "__main__":
    start_server()
else:
    logger.info(__name__)
    logger.info("server.py is being imported into another module")