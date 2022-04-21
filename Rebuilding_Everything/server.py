import socket
import numpy as np
import time
from threading import Thread

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
motion = 'gdfdhfjhgvj'
condition = 0
robot_state_message = b"no state"


def client_thread(conn): 
    global flag
    global motion
    global condition
    global robot_state_message
    number = flag
    flag = flag + 1
    t = time.time()
    t_nm1 = t
    is_alive = True
    refresh_rate = np.zeros((250,))
    i_rr = 0
    while is_alive:
        # print(f'RSM: {robot_state_message}')
        client_input = conn.recv(1024)
        t = time.time()
        if client_input == b'motion':
            print('received motion:')
            print(motion)
            conn.send(motion)
        elif client_input[:4] == b'RSM:':
            robot_state_message = client_input[4:]
            # print(robot_state_message)
        elif client_input == b'RSR':
            # print("robot state requested")
            conn.send(robot_state_message)
        elif client_input == b'I am alive':
            conn.send(b"testing IO")
        elif client_input == b'end':
            print('Connection to process %d ended.' % number)
            conn.close()
            is_alive = False
        else:
            motion = client_input
            #print(motion)
            # debugging type step:
            cl = str(client_input)
            print('client ' + str(conn) + ' just sent ' + cl)
        if i_rr >= 250:
            i_rr = 0
            print('Refresh rate of process %d: %3.1f' % (number, (1. / np.mean(refresh_rate))))
            if (1. / np.mean(refresh_rate)) > 2000:
                conn.close()
                is_alive = False
        refresh_rate[i_rr] = t - t_nm1

        # if (0.005 - refresh_rate[i_rr]) > 0:
        #     time.sleep(0.005 - refresh_rate[i_rr])

        i_rr += 1
        t_nm1 = t


while True:
    print("entered while loop")
    con, addr = s.accept()
    print('Connected: ' + str(addr))
    Thread(target=client_thread, args=(con,)).start()

    # print("waiting for s2 connection")
    # con, addr = s2.accept()
    # print('Connected on server 2 ' + str(addr))
    # Thread(target=client_thread, args=(con,)).start()


    
