#!/usr/bin/python3

import argparse
import logging
import time
from interpreter.interpreter import InterpreterHelper


def parseArgs():
    parser = argparse.ArgumentParser(description = 'Example for using skipbuffer, and abort commands')
    parser.add_argument('ip', help='Specify the IP of the robot (required)')
    parser.add_argument('-v', '--verbose', help='increase output verbosity', action='store_true')
    parser.add_argument('-d', '--debug', help='print debug level messages', action='store_true')
    args = parser.parse_args()

    if args.ip is None:
        sys.exit('Robot ip has to be specified with -i ')

    if args.debug:
        logging.basicConfig(level=logging.DEBUG)
    elif args.verbose:
        logging.basicConfig(level=logging.INFO)

    return args


if __name__ == "__main__":
    args = parseArgs()
    intrp = InterpreterHelper(args.ip)
    intrp.connect()
    # home position
    command_id = intrp.execute_command("movej([0, -1.57, 0,  -1.57, 0,  0],a=1,v=1.05,t=0,r=0)")
    logging.info(f"First interpreted command id: {command_id}")
    time.sleep(1)
    # send set of commands
    intrp.execute_command("movej([-1.11,-2.12,0,-1.57,0,0],a=1,v=1.05,t=0,r=0.1)")
    intrp.execute_command("movej([-1.11,-1.16,0,-1.57,0,0],a=1,v=1.05,t=0,r=0.1)")
    intrp.execute_command("movej([-1.11,-2.12,0,-1.57,0,0],a=1,v=1.05,t=0,r=0.1)")
    intrp.execute_command("movej([-1.11,-1.16,0,-1.57,0,0],a=1,v=1.05,t=0,r=0.1)")
    intrp.execute_command("movej([-1.11,-2.12,0,-1.57,0,0],a=1,v=1.05,t=0,r=0.1)")
    intrp.execute_command("movej([-1.11,-1.16,0,-1.57,0,0],a=1,v=1.05,t=0,r=0.1)")
    intrp.execute_command("movej([-1.11,-2.12,0,-1.57,0,0],a=1,v=1.05,t=0,r=0.1)")
    intrp.execute_command("movej([-1.11,-1.16,0,-1.57,0,0],a=1,v=1.05,t=0,r=0.1)")
    intrp.execute_command("movej([-1.11,-2.12,0,-1.57,0,0],a=1,v=1.05,t=0,r=0.1)")
    intrp.execute_command("movej([-1.11,-1.16,0,-1.57,0,0],a=1,v=1.05,t=0,r=0.1)")
    command_id = intrp.execute_command("movej([-1.11,-1.16,0,-1.57,0,0],a=1,v=1.05,t=0,r=0.1)")
    logging.info(f"Last interpreted command id before skipbuffer: {command_id}")
    # Allow few moves to execute before skipbuffer
    time.sleep(3)
    # deleting the buffer in interpreter mode while some of the previous commands
    # are still waiting to be executed
    intrp.execute_command("skipbuffer")
    logging.info(f"Last command executing before skipbuffer: {intrp.get_last_executed_id()}")

    logging.info("Aborting running move command")
    intrp.execute_command("abort")
    logging.info(f"Command executing or completed before abort: {intrp.get_last_executed_id()}")

    logging.info("Execute another set of motions")
    intrp.execute_command("movej([1.39,-3.08,1.16,-2.07,1.26,0.68],a=1,v=1.05,t=0,r=0.1)")
    intrp.execute_command("movej([1.309,-2.07,-1.25,-2.82,1.90,0.68],a=1,v=1.05,t=0,r=0.1)")
    intrp.execute_command("movej([1.39,-3.08,1.16,-2.07,1.26,0.68],a=1,v=1.05,t=0,r=0.1)")
    intrp.execute_command("movej([1.309,-2.07,-1.25,-2.82,1.90,0.68],a=1,v=1.05,t=0,r=0.1)")
    intrp.execute_command("movej([1.39,-3.08,1.16,-2.07,1.26,0.68],a=1,v=1.05,t=0,r=0.1)")
    intrp.execute_command("movej([1.309,-2.07,-1.25,-2.82,1.90,0.68],a=1,v=1.05,t=0,r=0.1)")
    intrp.execute_command("movej([1.39,-3.08,1.16,-2.07,1.26,0.68],a=1,v=1.05,t=0,r=0.1)")
    intrp.execute_command("movej([1.309,-2.07,-1.25,-2.82,1.90,0.68],a=1,v=1.05,t=0,r=0.1)")
    # go back to home position
    command_id = intrp.execute_command("movej([0, -1.57, 0,  -1.57, 0,  0],a=1,v=1.05,t=0,r=0)")
    logging.info(f"Last move command id: {command_id}. Waiting to execute all commands.")
    while intrp.get_last_executed_id() != command_id:
        time.sleep(1)
    logging.info(f"All commands executed. Leaving interpreter mode after last move command executed, "
                 f"and still moving to target pose.")
    intrp.end_interpreter()
 

