import os
import sys

def setup():
    sys.path.append(str(os.path.dirname(__file__)))

# Setup the directory when imported
setup()