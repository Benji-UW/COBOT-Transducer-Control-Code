import time
import os
import logging
from logging.handlers import TimedRotatingFileHandler
from logging import Formatter
# from test_file import *
import test_file

date_time_str = time.strftime(r"%Y-%m-%d_%H-%M-%S")
file_itr = 0
path = os.path.dirname(__file__)

while os.path.exists(path + "\\Scans\\test_%s.json" % file_itr):
    file_itr += 1
file_itr = 4

root_logger = logging.getLogger()
handler = TimedRotatingFileHandler(f"logging\debug_log_test{file_itr}.log",\
    when='D',backupCount=8,encoding="utf-8")
handler.setLevel(logging.DEBUG)

ch = logging.StreamHandler()
ch.setLevel(logging.INFO)

formatter = Formatter(fmt='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
handler.setFormatter(formatter)
ch.setFormatter(formatter)

root_logger.addHandler(handler)
root_logger.addHandler(ch)
root_logger.setLevel(logging.DEBUG)

logger = logging.getLogger(__name__)

def main():
    print(__name__)
    # logging.basicConfig(filename = path + "\logging\debug_log " + date_time_str + ".log", encoding='utf-8',\
    #     level=logging.DEBUG, format='%(levelname)s:%(message)s')
    
    logger.debug("Debug log for the robot starting on " + date_time_str)
    # logging.debug("Debug log for the robot starting on " + date_time_str)

    thing = test_file.Foo()
    thing.log_something("Hey here's a string I passed in from main!!!")
    thing.log_something("This is the updated kind where I did more things!!!!")

main()