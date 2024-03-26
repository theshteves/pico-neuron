#!/usr/bin/env python3
import argparse
from contextlib import contextmanager
import queue
import math
import pprint
import random
import time
import threading

DEFAULT_BROADCAST_COUNT = 100
DEFAULT_HOST_COUNT = 5
DEFAULT_DURATION = 8
DEFAULT_SEED = 0
#TODO: ratio for signal half-life: should each broadcast upadte signal by 1/3?? 1/10? etc...

def _random_address():
    '''Generate a random MAC address'''

    return ':'.join(['%02x' % random.randint(0, 255) for _ in range(6)])


def _random_position():
    '''Generate a random position'''

    return (random.randint(11, 99), random.randint(11, 99))


def _distance(_from, _to):
    '''Calculate the Euclidean distance between two points'''

    return int(math.sqrt(sum((_to[x] - _from[x])**2 for x in range(2))))


@contextmanager
def fixed_time(seconds):
    start_time = time.time()
    try:
        yield lambda: time.time() - start_time < seconds

    finally:
        pass  # No cleanup needed here


#def broadcast_environment(host_address):
#    '''The'''
#
#    #with LOCK:
#    position = POSITIONS[host_address]
#    hosts = list(POSITIONS.keys())
#    hosts.remove(host_address)  # exclude self
#
#    #targets = random.sample(hosts, k=random.randint(1, num_hosts))
#    targets = [(_host, POSITIONS[_host], _distance(position, POSITIONS[_host])) for _host in hosts]
#    targets = sorted(targets, key=lambda x: x[-1])
#
#    scan = f'==> {host_address} @ {position}\n'
#    for addr, pos, distance in targets:
#        scan += f'[{addr}] @ {pos}:  {int(distance)}\n'
#
#    print(scan)
    

class Host(object):
    '''Host'''

    def __init__(self):
        self.address = _random_address()
        self.position = _random_position()
        self.neighbors = {}
        self.messages = queue.Queue()

    def __repr__(self):
        return self.address[:8]

    def __str__(self):
        return f'Host({self.position[0]}:{self.position[1]})'

    def simulate(self, duration):
        '''Simulate host for [duration] seconds'''

        with fixed_time(duration) as countdown:
            while countdown():

                if self.messages.empty():
                    time.sleep(1)
                    continue

                # Wait for new messages
                message = self.messages.get(block=True, timeout=1)
                if not message:
                  continue

                signal, neighbor, their_neighbors = message
                new_signal = their_neighbors[self.address] if self.address in their_neighbors else signal
                old_signal = self.neighbors.get(neighbor, new_signal)
                self.neighbors[neighbor] = ((old_signal * 3) + new_signal) // 4


def simulate_network(broadcast_count, duration, host_count, *args, **kwargs):
    '''Simulate homogenous network of distributed hosts in parallel'''

    hosts = [Host() for _ in range(host_count)]
    distances = {host.position: 
        {_host.position: _distance(host.position, _host.position)
          for _host in hosts if _host.address != host.address
        }
        for host in hosts
    }
    pprint.pprint(distances)
    print()

    threads = [threading.Thread(target=host.simulate, args=(duration,)) for host in hosts]
    for thread in threads:
        thread.start()

    for _ in range(broadcast_count):
        broadcaster = random.choice(hosts)

        for host in hosts:
            if host.address == broadcaster.address:
                continue  # exclude self

            signal = distances[broadcaster.position][host.position]  # Populate `signal` field
            signal += int(random.uniform(-1, 1) * 20)

            # Message format: (int, (int, int), {(int, int) -> int})
            message = (signal, broadcaster.position, broadcaster.neighbors)
            host.messages.put(message, timeout=1)

    for thread in threads:
        thread.join(timeout=duration)

    for host in hosts:
        print(f'\n=> {host}')
        pprint.pprint(host.neighbors)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Simulate homogenous network of distributed hosts in parallel')
    parser.add_argument('-b', '--broadcasts', type=int, default=DEFAULT_BROADCAST_COUNT, dest='broadcast_count', help='Number of broadcasts in the simulation')
    parser.add_argument('-d', '--duration', type=int, default=DEFAULT_DURATION, help='Seed for the random number generator')
    parser.add_argument('-H', '--hosts', type=int, default=DEFAULT_HOST_COUNT, dest='host_count', help='Number of hosts in the simulation')
    parser.add_argument('-s', '--seed', type=int, default=DEFAULT_SEED, help='Seed for the random number generator')
    args = parser.parse_args()

    random.seed(args.seed)
    simulate_network(**vars(args))
