import os
import sys
import time
from threading import Thread

path = os.path.abspath('')
sys.path.append(path)

import server

server.test_import()


print("Starting the server in a thread...")

Thread(target=server.start_server).start()

print("Now some other code is running...")

for i in range(40):
    print(i)
    time.sleep(0.1)