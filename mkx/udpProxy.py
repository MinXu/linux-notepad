#!/usr/bin/python

import select, socket, sys, os, fcntl, struct
ip_prefix1 = ""
ip_prefix2 = ""
port=""
interface_eth0 = ""
interface_wave = ""


def getip(ethname):
    s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(s.fileno(), 0x8915, struct.pack('256s', ethname[:15]))[20:24])

from threading import Thread

class Proxy(Thread):
    """ used to proxy single udp connection 
    """

    BUFFER_SIZE = 4096 
    def __init__(self, listening_address, phone_address, box_address):
    	print " Server started on", listening_address
    	Thread.__init__(self)
    	self.bind = listening_address
    	self.phone = phone_address
	self.box   = box_address
    
    def run(self):
    	# listen for incoming connections:
    	phone = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
        	phone.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        except socket.error, err:
        	print "phone socket Couldn't set BROADCAST"
        	pass
    	phone.connect(self.phone)


    	box = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
        	box.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        except socket.error, err:
        	print "box socket Couldn't set BROADCAST"
        	pass
    	box.connect(self.box)


    	s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
	        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
	except socket.error, err:
		print "listener socket Couldn't set BROADCAST"
		pass
        
	s.setblocking(0)

    	try:
    		s.bind(self.bind)
    	except socket.error, err:
    		print "Couldn't bind server on %r" % (self.bind)
    		raise SystemExit

   	while 1:
		try:
			readable, writeable, exceptional = select.select([s], [], [])
		except:
			s.close()
			print 'Closing , exception error during select'
			raise SystemExit

		for fd in readable:
			if fd is s:
		    		datagram, (ip, port) = fd.recvfrom(self.BUFFER_SIZE)
		    		if not datagram:
		    			break
				print "receive from %s %s" % (ip , port)
				
				if ip == interface_eth0:
					break
				elif ip == interface_wave:
					break
				elif ip_prefix1 in ip:
					length = len(datagram)
			 	   	sent = box.send(datagram)
		    			if length != sent:
		    				print 'cannot send to %r, %r !+ %r' % (self.box, length, sent)
				elif ip_prefix2 in ip:
					length = len(datagram)
			 	   	sent = phone.send(datagram)
		    			if length != sent:
		    				print 'cannot send to %r, %r !+ %r' % (self.phone, length, sent)
		    		else:
		    			print "error address from %s %s" % (ip , port)
		
 	
   	s.close()
	box.close()
	phone.close();


if __name__ == "__main__":
    interface_eth0 = getip('eth0')
    ip_parts = interface_eth0.split(".")
    ip_prefix1 = ip_parts[0] + "." + ip_parts[1] + "." + ip_parts[2] + "."

    interface_wave = getip('wave-data')
    ip_parts = interface_wave.split(".")
    ip_prefix2 = ip_parts[0] + "." + ip_parts[1] + "." + ip_parts[2] + "."

    port = int(os.environ['PORT'])

    LISTEN = ("0.0.0.0", port)
    TARGET_PHONE = (ip_prefix1 + "255", port)
    TARGET_BOX   = (ip_prefix2 + "255", port)

    while 1:
    	proxy = Proxy(LISTEN, TARGET_PHONE, TARGET_BOX)
	print "lan ip(%s/%s)   wave ip(%s/%s) " % (interface_eth0, ip_prefix1, interface_wave, ip_prefix2)
    	proxy.start()
    	proxy.join()
    	print ' [restarting] '
