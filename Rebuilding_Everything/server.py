import socket
import numpy as np
import time
from threading import Thread

HOST = ''
PORT = 508

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen(6)

flag = 0
motion = 'gdfdhfjhgvj'
condition = 0

def client_thread(conn): 
    global flag
    global motion
    global condition
    number = flag
    flag = flag + 1
    t = time.time()
    t_nm1 = t
    is_alive = True
    refresh_rate = np.zeros((120,))
    i_rr = 0
    while is_alive:
        client_input = conn.recv(1024)
        t = time.time()
        if client_input == b'motion':
            print('received motion:')
            print(motion)
            conn.send(motion)
        elif client_input == b'cond':
            conn.send(str(condition))
        elif client_input == b'c0':
            condition = 0
        elif client_input == b'c1':
            condition = 1
        elif client_input == b'end':
            print('Connection to process %d ended.' % number)
            conn.close()
            is_alive = False
        else:
            motion = client_input
            #print(motion)
            # debugging type step:
            # cl = str(client_input)
            # print('client just sent ' + cl)
        if i_rr >= 120:
            i_rr = 0
            print('Refresh rate of process %d: %3.1f' % (number, (1. / np.mean(refresh_rate))))
            if (1. / np.mean(refresh_rate)) > 2000:
                conn.close()
                is_alive = False
        refresh_rate[i_rr] = t - t_nm1
        i_rr += 1
        t_nm1 = t


while True:
    con, addr = s.accept()
    print('Connected: ' + str(addr))
    Thread(target=client_thread, args=(con,)).start()
