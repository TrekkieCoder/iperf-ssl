This is Iperf v2.0.0, a tool for measuring Internet bandwidth performance. This iperf fork is customized to work with SSL/TLS. This is not super stable but is good for quick testing.

See the doc directory for more documentation. 

## How to build 
```
  ./build.sh                         # -- Build Iperf
  make install                       # -- install Iperf, if desired
```

  iperf -s               (on machine "foo.bar.edu")
  iperf -c foo.bar.edu   (on some other machine)
  iperf -h               (for help screen)
  iperf -v               (for version information)
  iperf -tls=v1.2        (for running in TLS mode)

Examples for testing ssl/tls :

### In server mode:
```
./src/iperf --tls=v1.2 -s
```

### In client mode :
```
./src/iperf --tls=v1.2 --ktls -c <ip> -t 60 -i 1
```

### Notes about certificates

The pre-generated cert file is ```newreq.pem``` and key is ```key.pem```. This is just for testing purposes. Kindly generate fresh certificates if need be.

### Misc Info 
```
Copyright 1999, 2000, 2001, 2002, 2003, 2004
The Board of Trustees of the University of Illinois
All rights reserved
See UI License (doc/ui_license.html) for complete details.

$Id: README,v 1.1.1.1 2004/05/18 01:50:44 kgibbs Exp $
```
