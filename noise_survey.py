#!/usr/bin/env python3

from datetime import datetime
import os
import socket
import time
import logging


#
# Configuration variables
#

VHF = ('135M:165M:10k', 'VHF')
UHF = ('420M:450M:10k', 'UHF')

HOST = 'localhost'
DATADIR = '/space/noise-survey'

PARK_POSITION = (0, 0)



# from jupyter notebook calculations.  88 points
points = [
        [  0.        ,   0.        ],
        [ 31.71747441,   0.        ],
        [ 58.28252559,   0.        ],
        [ 72.82796215,   0.        ],
        [ 90.        ,   0.        ],
        [107.17203785,   0.        ],
        [121.71747441,   0.        ],
        [148.28252559,   0.        ],
        [180.        ,   0.        ],
        [211.71747441,   0.        ],
        [238.28252559,   0.        ],
        [252.82796215,   0.        ],
        [270.        ,   0.        ],
        [287.17203785,   0.        ],
        [301.71747441,   0.        ],
        [328.28252559,   0.        ],
        [313.57234952,   8.48907956],
        [297.13337804,  13.81900921],
        [279.6937239 ,  15.24016228],
        [260.3062761 ,  15.24016228],
        [242.86662196,  13.81900921],
        [226.42765048,   8.48907956],
        [195.45043709,   9.34970354],
        [164.54956291,   9.34970354],
        [133.57234952,   8.48907956],
        [117.13337804,  13.81900921],
        [ 99.6937239 ,  15.24016228],
        [ 80.3062761 ,  15.24016228],
        [ 62.86662196,  13.81900921],
        [ 46.42765048,   8.48907956],
        [ 15.45043709,   9.34970354],
        [  0.        ,  17.17203785],
        [  0.        ,  31.71747441],
        [ 15.45043709,  26.28667666],
        [ 31.71747441,  18.        ],
        [ 49.49929499,  25.17126217],
        [ 69.09484255,  30.        ],
        [ 90.        ,  31.71747441],
        [110.90515745,  30.        ],
        [130.50070501,  25.17126217],
        [148.28252559,  18.        ],
        [180.        ,  17.17203785],
        [180.        ,  31.71747441],
        [195.45043709,  26.28667666],
        [164.54956291,  26.28667666],
        [148.28252559,  36.        ],
        [168.35927686,  42.97806766],
        [191.64072314,  42.97806766],
        [211.71747441,  36.        ],
        [234.11024534,  43.48707775],
        [257.78168013,  45.77176175],
        [282.21831987,  45.77176175],
        [305.88975466,  43.48707775],
        [328.28252559,  36.        ],
        [348.35927686,  42.97806766],
        [344.54956291,  26.28667666],
        [328.28252559,  18.        ],
        [310.50070501,  25.17126217],
        [290.90515745,  30.        ],
        [270.        ,  31.71747441],
        [249.09484255,  30.        ],
        [229.49929499,  25.17126217],
        [211.71747441,  18.        ],
        [211.71747441,  54.        ],
        [211.71747441,  72.        ],
        [241.66003263,  59.79009469],
        [270.        ,  58.28252559],
        [298.33996737,  59.79009469],
        [328.28252559,  54.        ],
        [328.28252559,  72.        ],
        [270.        ,  72.82796215],
        [180.        ,  58.28252559],
        [148.28252559,  54.        ],
        [148.28252559,  72.        ],
        [118.33996737,  59.79009469],
        [ 90.        ,  58.28252559],
        [ 61.66003263,  59.79009469],
        [ 31.71747441,  54.        ],
        [ 31.71747441,  36.        ],
        [ 11.64072314,  42.97806766],
        [ 54.11024534,  43.48707775],
        [ 77.78168013,  45.77176175],
        [102.21831987,  45.77176175],
        [125.88975466,  43.48707775],
        [ 90.        ,  72.82796215],
        [ 31.71747441,  72.        ],
        [  0.        ,  58.28252559],
        [  0.        ,  90.        ],
    ]



class Rotctl(object):
    """ rotctld (hamlib) communication class """
    # Note: This is a massive hack.

    def __init__(self, hostname, port=4533, poll_rate=5, timeout=5, az_180=False):
        """ Open a connection to rotctld, and test it for validity """
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(timeout)

        self.hostname = hostname
        self.port = port


    def connect(self):
        """ Connect to rotctld instance """
        self.sock.connect((self.hostname, self.port))
        model = self.get_model()
        if model == None:
            # Timeout!
            self.close()
            raise Exception("Timeout!")
        else:
            return model


    def close(self):
        self.sock.close()


    def send_command(self, command):
        """ Send a command to the connected rotctld instance,
            and return the return value.
        """
        self.sock.sendall((command+'\n').encode())
        try:
            recv_msg = self.sock.recv(1024).decode()
        except Exception as e:
            print(e)
            recv_msg = None

        #print(f'response: {recv_msg}')
        return recv_msg


    def get_model(self):
        """ Get the rotator model from rotctld """
        model = self.send_command('_')
        return model


    def set_azel(self, az, el=None):
        """ Command rotator to a particular azimuth/elevation """
        if el is None:
            az, el = azimuth

        # Sanity check inputs.
        if el > 90.0:
            el = 90.0
        elif el < 0.0:
            el = 0.0

        if az > 360.0:
            az = az % 360.0


        command = "P %3.1f %2.1f" % (az, el)
        response = self.send_command(command)
        if "RPRT 0" in response:
            return True
        else:
            return False


    def get_azel(self, max_retries=5):
        """ Poll rotctld for azimuth and elevation """
        # Send poll command and read in response.
        response = self.send_command('p')

        # Attempt to split response by \n (az and el are on separate lines)
        try:
            response_split = response.split('\n')
            az = float(response_split[0])
            el = float(response_split[1])
            return (az, el)
        except Exception as e:
            print(e)
            raise SyntaxError(f'Could not parse position from: {response}')


    def halt(self):
    	""" Immediately halt rotator movement, if it support it """
    	self.send_command('S')

    def moveto(self, azel, tol=2.0):
        """Command rotator to (az, el) tuple position.
        The function will block until the rotator reports that it is within tol
        degrees of the target position.  Too large a tol means the rotator will
        still be moving by the time the receiver starts.
        """
        WAIT_TIME = 0.5

        az, el = azel
        print(f'Moving to ({az:3.0f}, {el:3.0f})', end='', flush=True)
        if not self.set_azel(az, el):
            raise ConnectionError('Bad response from set_azel()')

        # this is a slow process...
        time.sleep(WAIT_TIME)
        pos = self.get_azel()
        while diff(pos, azel) > tol:
            print('.', end='', flush=True)
            time.sleep(WAIT_TIME)
            pos = self.get_azel()
        
        print(pos, flush=True)  # prevent out-of-order output
        return pos


def diff(current, target):
    """Return the maximum difference between the current and target position."""
    d = ((a - b) for a,b in zip(current, target))
    da = map(abs, d)
    return max(da)





def scan(points):
    rotator = Rotctl(HOST)
    rotator.connect()

    nPoints = len(points)
    for i, azel in enumerate(points):
        print()
        print(f'Position {i}/{nPoints} {azel}')

        # round to 0.1 degrees, one digit greater than the precision of the rotator
        # controller
        azel = tuple(map(lambda x: round(x, 1), azel))

        try:
            rotator.moveto(azel)
        except ConnectionError:
            logging.error('Connection error, skipping this point.')
            continue

        # sense spectrum
        az, el = azel
        freqs = ''
        for freq, band in (VHF, UHF):
            now = datetime.utcnow().strftime('%Y-%m-%dT%H:%M:%SZ')
            cmd = f'rtl_power -f{freq} -i 5 -1 -c 0.3 {DATADIR}/{now}_{band}_a{az:03.0f}_e{el:03.0f}.csv'
        print(cmd)
        ret = os.system(cmd)
        print(ret)

    # done.  Return to park position
    rotator.moveto(PARK_POSITION)
    rotator.close()
    print('Finished!')


if __name__ == '__main__':
    scan(points)
