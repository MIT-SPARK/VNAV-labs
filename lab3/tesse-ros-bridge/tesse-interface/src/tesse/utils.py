###################################################################################################
# DISTRIBUTION STATEMENT A. Approved for public release. Distribution is unlimited.
#
# This material is based upon work supported by the Under Secretary of Defense for Research and
# Engineering under Air Force Contract No. FA8702-15-D-0001. Any opinions, findings, conclusions
# or recommendations expressed in this material are those of the author(s) and do not necessarily
# reflect the views of the Under Secretary of Defense for Research and Engineering.
#
# (c) 2020 Massachusetts Institute of Technology.
#
# MIT Proprietary, Subject to FAR52.227-11 Patent Rights - Ownership by the contractor (May 2014)
#
# The software/firmware is provided to you on an As-Is basis
#
# Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS Part 252.227-7013
# or 7014 (Feb 2014). Notwithstanding any copyright notice, U.S. Government rights in this work
# are defined by DFARS 252.227-7013 or DFARS 252.227-7014 as detailed above. Use of this work other
# than as specifically authorized by the U.S. Government may violate any copyrights that exist in
# this work.
###################################################################################################

import socket
import threading
import defusedxml.ElementTree as ET

class UdpListener(threading.Thread):
    __epsilon_timing__ = 0.0025 # 1/(2*200), the game runs at ~200 Hz but sometimes the timing is a little off.
                                # We use this epsilon to approximate that 1/rate time has elapsed in the game.

    def __init__(self, host='<broadcast>', port=None, timeout=0.1, rate=None):
        """ Initializer for UdpListener object.

            Derived from a threading.Thread object, UdpListener will spin off
            a new thread and listen for data coming in on a specified port.
            It supports handing that data with an arbitrary number and type
            of custom callback functions.

            Args:
                host: A string representing the hostname which will be used
                    to listen for data. Default is '<broadcast>', which will
                    iterate over all network interfaces for send.
                port: An integer representing the port number to listen to.
                    This must be set by the user and has no valid default.
                timeout: A float representing the timeout period for the socket.
                    Default is 0.1 seconds.
                rate: A float representing the desired rate at which to call
                    all custom callbacks on the incoming data. Default is None,
                    which will send data to the callbacks as fast as possible.
        """
        super(UdpListener, self).__init__()
        self.rate = rate

        # get ip address
        if host is None:
            host = socket.gethostbyname(socket.gethostname())
        else:
            host = socket.gethostbyname(host)

        # create socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.settimeout(timeout)
        self.sock.bind((host, port))

        # thread management
        self.alive = threading.Event()
        self.alive.set()

        # Desired rate that messages are processed
        self.rate = rate

        self.handlers = {}

    def subscribe(self, name, handler):
        """ Adds custom callback function to dictionary of subscribers.

            Args:
                name: A string representing the unique name of the callback.
                handler: A function or class (with __call__ implemented)
                    to use as a callback when data is received.
        """
        self.handlers[name] = handler

    def join(self, timeout=1):
        """ Safely close thread and unbind socket. """
        self.alive.clear()
        threading.Thread.join(self, timeout)
        self.sock.close()

    def run(self):
        last_game_time = -1e6  # initialize
        min_dt = (1.0/self.rate) - self.__epsilon_timing__
        while self.alive.isSet():
            try:
                data = self.sock.recv(1024)

                # Process the message if the game has elapsed *at least* 1/self.rate
                game_time = float(ET.fromstring(data).find('time').text)
                dt = game_time - last_game_time

                if self.rate is None or dt >= min_dt:
                    for name in self.handlers:
                        self.handlers[name](data)
                    last_game_time = game_time

            except socket.timeout as error:
                continue
