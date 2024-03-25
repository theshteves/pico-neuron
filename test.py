import math
import random
import time
import threading

# Global lock for thread-safe operations on the shared data
LOCK = threading.Lock()
NODES = 5
POSITIONS = {}


def generate_address():
    '''Generate a random MAC address'''

    return ':'.join(['%02x' % random.randint(0, 255) for _ in range(6)])


def generate_position():
    '''Generate a random position'''

    return (random.randint(11, 99), random.randint(11, 99))


def calculate_distance(pos1, pos2):
    '''Calculate the Euclidean distance between two points'''

    return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)


def broadcast_perspective(host):
    '''The'''

    position = POSITIONS[host]
    hosts = list(POSITIONS.keys())
    hosts.remove(host)  # Exclude self
    
    #targets = random.sample(hosts, k=random.randint(1, num_hosts))
    targets = [(_host, POSITIONS[_host], calculate_distance(position, POSITIONS[_host])) for _host in hosts]
    targets = sorted(targets, key=lambda x: x[-1])

    scan = f'==> {host} @ {position}\n'
    for addr, pos, distance in targets:
        scan += f'[{addr}] @ {pos}:  {int(distance)}\n'

    print(scan)
    

def simulate_host(address, position):
    '''Simulates a device that broadcasts its rssi value'''
    global POSITIONS, LOCK

    while True:
        # Simulate time delay between broadcasts
        time.sleep(random.uniform(0.5, 2.0))

        num_hosts = len(POSITIONS)
        if num_hosts < 3:
          continue

        with LOCK:
            broadcast_perspective(address)

        '''
        while True:
            with LOCK:
                # Simulate signal strength value
                POSITIONS[address] = random.randint(-100, -1)

            # Print current state to see the simulation in action (optional)
            print(f'{address} broadcasting rssi: {POSITIONS[address]}')
        '''


def simulate_network():
    '''Simulate multiple homogenous network devies'''
    global NODES, POSITIONS

    threads = []
    for _ in range(NODES):
        address = generate_address()
        position = generate_position()
        POSITIONS[address] = position

        thread = threading.Thread(target=simulate_host, args=(address, position))
        threads.append(thread)
        thread.start()

    '''
    for thread in threads:
        thread.join()
    '''


if __name__ == '__main__':
    random.seed(0)
    simulate_network()
