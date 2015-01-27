#!/usr/bin/env python

import os
import fcntl
import select
import struct

def main():
    f = open('/dev/gyro0', 'rb')
    fd = f.fileno()

    ## put the file descriptor into non-blocking mode
    flags = fcntl.fcntl(fd, fcntl.F_GETFL, 0)
    flags |= os.O_NONBLOCK
    fcntl.fcntl(fd, fcntl.F_SETFL, flags)
    
    ## setup epoll on file descriptor
    ep = select.epoll()
    ep.register(fd, select.EPOLLIN)

    print 'yaw   pitch roll' # print heading
    try:
        while True:
            ret = ep.poll() # wait on epoll
            data = f.read(12)
            yaw, pitch, roll = struct.unpack('3h', data[:6])
            print yaw, pitch, roll
    finally:
        f.close()

if __name__ == '__main__':
    main()
