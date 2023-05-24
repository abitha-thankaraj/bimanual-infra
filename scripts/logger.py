import logging
import argparse
from termcolor import colored

# Create a logger instance
logger = logging.getLogger(__name__)

# Set the log level of the logger
logger.setLevel(logging.DEBUG)  # Set the initial log level to DEBUG

# Set up logging handlers
console_handler = logging.StreamHandler()
console_handler.setLevel(logging.INFO)  # Set the default console log level to INFO

# Create a formatter with colored log messages
log_format = '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
colored_formatter = logging.Formatter(log_format)

# Create a custom log formatter to add color to log messages
class ColoredFormatter(logging.Formatter):
    def format(self, record):
        levelname = record.levelname
        if levelname == 'DEBUG':
            colored_levelname = colored(levelname, 'cyan')
        elif levelname == 'INFO':
            colored_levelname = colored(levelname, 'green')
        elif levelname == 'WARNING':
            colored_levelname = colored(levelname, 'yellow')
        elif levelname == 'ERROR':
            colored_levelname = colored(levelname, 'red')
        elif levelname == 'CRITICAL':
            colored_levelname = colored(levelname, 'magenta')
        else:
            colored_levelname = colored(levelname, 'white')

        record.levelname = colored_levelname
        return super().format(record)

# Set the formatter for the console handler
console_handler.setFormatter(ColoredFormatter(log_format))

# Add the console handler to the logger
logger.addHandler(console_handler)

# Parse command-line arguments
parser = argparse.ArgumentParser()
parser.add_argument('--log-level', type=str, default='info', help='Set the log level')
args = parser.parse_args()

# Convert the log level argument to upper case
log_level = args.log_level.upper()

# Set the log level of the logger based on the provided argument
if log_level == 'DEBUG':
    logger.setLevel(logging.DEBUG)
elif log_level == 'INFO':
    logger.setLevel(logging.INFO)
elif log_level == 'WARNING':
    logger.setLevel(logging.WARNING)
elif log_level == 'ERROR':
    logger.setLevel(logging.ERROR)
elif log_level == 'CRITICAL':
    logger.setLevel(logging.CRITICAL)
else:
    raise ValueError('Invalid log level specified')

# Log messages at different levels
logger.debug('Debug message')
logger.info('Info message')
logger.warning('Warning message')
logger.error('Error message')
logger.critical('Critical message')
