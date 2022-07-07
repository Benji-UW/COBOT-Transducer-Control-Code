#!/usr/bin/python3

import logging
import argparse
import sys
import time
from interpreter.interpreter import InterpreterHelper


def parseArgs():
    parser = argparse.ArgumentParser(description = 'Example for using thead, and function definitions. '
                                                   'Polyscope should be executing interpreterModeSequential program')
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


# sample UR script thread definition
SAMPLE_THREAD= f"                              \
    thread myThread():                         \
        i = 0                                  \
        while (exit_thread == False):          \
            i=i+1                              \
            if i%50 == 0:                      \
                textmsg(\"myThread runnning\") \
            end                                \
            sleep(0.01)                        \
        end                                    \
        textmsg(\"myThread Ended\")            \
    end\n"

# sample UR script function definition to end thread
SAMPLE_FUNCTION = "def exitThread(): exit_thread = True end\n"


if __name__ == "__main__":
    args = parseArgs()
    intrp = InterpreterHelper(args.ip)
    intrp.connect()

    logging.info("Create the thread , function  and run in the first interpreter_mode(clearOnEnd = True)")
    intrp.execute_command(SAMPLE_THREAD)
    intrp.execute_command(SAMPLE_FUNCTION)
    intrp.execute_command("thrd = run myThread()")
    logging.info("Exiting first interpreter")
    intrp.end_interpreter()

    # we suppose to be in the second interpreter line , wait 2 seconds here to make sure
    # previous interpreter_mode was ended. For more advance control please refer to state commands
    # in script manual
    logging.info("Waiting until program enters second interpreter mode")
    time.sleep(2)
    logging.info("End thread on the next interpreter session by setting exit_thread in exitThread function")
    intrp.execute_command("exitThread()")
    intrp.execute_command("join thrd")

    # end the last interpreter_mode session
    intrp.end_interpreter()
