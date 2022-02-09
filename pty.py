#!/usr/bin/env python
#coding=utf-8

import pty
import os
import select

def mkpty():
#Open a new tty
        master1, slave = os.openpty()
        slaveName1 = os.ttyname(slave)
        master2, slave = os.openpty()
        slaveName2 = os.ttyname(slave)

        print('\nslave device names:', slaveName1, slaveName2)
        return master1, master2

if __name__ == "__main__":

        master1, master2 = mkpty()
        while True:
        #       rl=read list, wait until ready to reading 
        #       wl=write list, wait until ready to writing
        #       el=exception list, wait for an "exceptional condition"
        #       timeout = 1s
                rl, wl, el = select.select([master1, master2], [], [], 1)
                for device in rl:
                        data = os.read(device, 128)
                        print("read %d data."%len(data))
                        if device == master1:
                                os.write(master2, data)
                        else:
                                os.write(master1, data)
