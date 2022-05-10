"""
FiPy NB-IoT NODE - MAIN CODE

---
IoT PLATFORM - PyCom FiPy NB-IoT Node
---

(c) Matteo M. 2022

"""
import config
from node import Node

if __name__ == '__main__':
    # Initialize Node object
    n = Node()
    # Start node operation
    n.start()

    if config.DEBUG_MODE:
        # Enable REPL terminal - for debugging
        n._log('You may now press ENTER to enter the REPL')
        input()

