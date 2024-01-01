# Overview
First try to implement a interface to the [MostSocket](https://github.com/rhysmorgan134/SocketMost) in Python.

## Features (routines)
- <b>setAmplifierSink</b> Switch the amplifier sink

## Execution
1. First add the py-socketmost-client.service to your system to startup the py-socketmost-client base server, this will accelerate further interaction with the py-client as the connection is already established and further communication happens via ZeroMQ.
   1. Copy py-socketmost-client.service to /etc/systemd/system/
   2. Startup the py-socketmost-client service via systemctl start py-socketmost-client
   3. Additionaly add to startup: systemctl enable py-socketmost-client
2. Execute py-socketmost-client with arguments:
   - python py-socketmost-client.py <routine> <args>
   - e.g. python py-socketmost-client.py setAmplifierSink 2
