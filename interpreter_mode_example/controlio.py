#!/usr/bin/python3

import logging
import argparse
import sys
import time
from interpreter.interpreter import InterpreterHelper


def parseArgs():
    parser = argparse.ArgumentParser(description = 'Send Interpreter commands from file')
    parser.add_argument('ip', help='Specify the IP of the robot (required)')
    parser.add_argument('-v', '--verbose', help='increase output verbosity', action='store_true')
    parser.add_argument('-d', '--debug', help='print debug level messages', action='store_true')
    args = parser.parse_args()

    if args.ip is None:
        sys.exit('Robot ip has to be specified')

    if args.debug:
        logging.basicConfig(level=logging.DEBUG)
    elif args.verbose:
        logging.basicConfig(level=logging.INFO)

    return args


if __name__ == "__main__":
    args = parseArgs()

    interpreter = InterpreterHelper(args.ip)
    interpreter.connect()
    interpreter.execute_command("set_analog_outputdomain(1, 0)")
    time.sleep(0.1)

    interpreter.execute_command("set_standard_analog_input_domain(0, 1)")
    time.sleep(0.1)

    interpreter.execute_command("set_tool_analog_input_domain(1, 0)")
    time.sleep(0.1)

    interpreter.execute_command("set_tool_digital_input_action(0, \"freedrive\")")
    time.sleep(0.1)

    interpreter.execute_command("set_tool_digital_out(1, True)")
    time.sleep(0.1)

    interpreter.execute_command("modbus_add_signal(\"172.140.17.11\", 255, 5, 1, \"output1_interpreter\")")
    time.sleep(0.1)

    interpreter.end_interpreter()

