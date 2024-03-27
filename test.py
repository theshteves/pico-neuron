#!/usr/bin/env python3
import argparse
from contextlib import contextmanager
import math
import pprint
import queue
import random
import threading
import time

DEFAULT_BROADCAST_COUNT = 4
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


    def _process_message(self, message):
        '''Process message'''

        received_signal, neighbor, their_neighbors = message
        premessage_signal = self.neighbors.get(neighbor, received_signal)

        if self.address in their_neighbors:
            # Balance their observation with ours
            received_signal = (received_signal + their_neighbors[self.address]) // 2

        # Balance new observation with historical trend
        postmessage_signal = ((premessage_signal * 7) + received_signal) // 8
        self.neighbors[neighbor] = postmessage_signal
        print(f'{self}: {premessage_signal} -> {postmessage_signal}')


    def broadcast(self):
        '''Broadcast'''

        return


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

                self._process_message(message)


def simulate_network(broadcast_count, duration, host_count, *args, **kwargs):
    '''Simulate homogenous network of distributed hosts in parallel'''

    # Generate real-world distances to fuzz around
    hosts = [Host() for _ in range(host_count)]
    distances = {host.position: 
        {_host.position: _distance(host.position, _host.position)
          for _host in hosts if _host.address != host.address
        }
        for host in hosts
    }
    #pprint.pprint(distances)
    print('\n\n'.join(str(y) for y in (f"=> {_k}\n{sorted(((v, k) for k, v in _v.items()))}" for _k, _v in distances.items())))
    print()

    # Simulate hosts _in parallel!_
    threads = [threading.Thread(target=host.simulate, args=(duration,)) for host in hosts]
    for thread in threads:
        thread.start()

    #TODO: Now parallelize the broadcasts!
    # Send out random broadcasts
    for _ in range(broadcast_count):
        broadcaster = random.choice(hosts)

        for host in hosts:
            if host.address == broadcaster.address:
                continue  # exclude self
  
            # Populate `signal` field
            signal = distances[broadcaster.position][host.position]
            signal += int(random.uniform(-1, 1) * 20)

            # Message format: (int, (int, int), {(int, int) -> int})
            message = (signal, broadcaster.position, broadcaster.neighbors)
            host.messages.put(message, timeout=1)
            #print(f'=> {signal}: {broadcaster.position} + {host.position}')

    # Wait for all threads to timeout
    for thread in threads:
        thread.join(timeout=duration)

    # Dump final state of all hosts
    for host in hosts:
        print(f'\n=> {host}')
        print('\n'.join(str(x) for x in sorted(((v, k) for k, v in host.neighbors.items()))))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Simulate homogenous network of distributed hosts in parallel')
    parser.add_argument('-b', '--broadcasts', type=int, default=DEFAULT_BROADCAST_COUNT, dest='broadcast_count', help='Number of broadcasts in the simulation')
    parser.add_argument('-d', '--duration', type=int, default=DEFAULT_DURATION, help='Seed for the random number generator')
    parser.add_argument('-H', '--hosts', type=int, default=DEFAULT_HOST_COUNT, dest='host_count', help='Number of hosts in the simulation')
    parser.add_argument('-s', '--seed', type=int, default=DEFAULT_SEED, help='Seed for the random number generator')
    args = parser.parse_args()

    random.seed(args.seed)
    simulate_network(**vars(args))
