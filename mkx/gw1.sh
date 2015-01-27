#tatic IP address
  #unload_dot3
  #load_dot4
  #teardown
  #ifconfig eth0 down
  #ifconfig eth0:1 192.168.104.100 netmask 255.255.254.0 broadcast 192.168.104.255 up
  #ifconfig eth0 192.168.104.200 netmask 255.255.255.128 broadcast 192.168.104.255 up
    
  # Set the H1 wireless static IP address
  #ifconfig wave-data down
  ifconfig wave-data 192.168.234.1/24 up
          
  # Setup routes to H2
  ip -4 route del 192.168.104.0/25
  ip -4 route add 192.168.104.0/25 via 192.168.234.2 dev wave-data
                
  #iptables -t nat -A POSTROUTING -O eth0 -jMASQUERADE
                  
  # Enable IPv4 routing
   echo "1" > /proc/sys/net/ipv4/ip_forward
   iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE
   iptables -t nat -A POSTROUTING -o wave-data -j MASQUERADE
 
