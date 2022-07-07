import logging

logger = logging.getLogger(__name__)

class Foo:
    def __init__(self):
        # logger = logging.getLogger("Foo logger")
        print("Initialized foo")
        logger.info("Initialized the foo module")
        print("This should've created a new logging module :/ fuck")

    def log_something(self, something):
        print("Got told to log a thing")
        logger.info("Recieved string from main:")
        logger.info(something)
        logger.warning("Recieved string from main:")
        logger.warning(something)
        