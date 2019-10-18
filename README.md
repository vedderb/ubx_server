# Ubx Server

This is a CLI client to communicate with Ublox GNSS receivers such as the M8T, M8T and F9P. It can act as a base station that outputs RTCM data over TCP, or as a rover that outputs NMEA data over TCP. In base station mode it can configure the ublox receiver to generate the RTCM messages, alternatively configure the ublox to output raw data which is encoded to RTCM3.

**Supported arguments**

```
./ubx_server -h
Arguments
--help : Show help text
--ublox [tty] : Ublox port, e.g. /dev/ttyACM0
--tcpport [port] : RTCM server port, e.g. 65100
--baseaccthres [th] : Accuracy threshold for base position, e.g. 4.0 (m)
--basemovethres [th] : Movement threshold for base position, e.g. 4.0 (m)
--basepos [lat]:[lon]:[height] : Fixed base pos, e.g. 57.714:12.890:200 
--baseencoderaw : Encode raw data to RTCM. Works with e.g. ublox M8T as base.
--isrover : Act as a rover
--roverrtcmserver [ip] : Rover RTCM server, e.g. 192.168.1.3
--roverrtcmport [port] : Rover RTCM port, e.g. 65100
--roverdatatcpport [tcpPort] : Start data tcp server on tcpPort
--rovernmeaserver [tcpPort] : Start NMEA TCP server on tcpPort
--roverdynmodel [model] : Dynamic model, e.g. 2 (stat), 4 (car), 5 (sea), 6 (air 1G), 7 (air 2G), 8 (air 4G)
--ratems [ms] : Navigation rate in milliseconds.
--verbose : Print more debug information
```

**Example: Base Station**

```bash
./ubx_server --ublox /dev/ttyACM0 --ratems 1000 --tcpport 65100
```
In this example, the ublox receiver is configured to output RTCM samples at 1 Hz (1000 ms) on TCP port 65100.

**Example: Rover**

```bash
./ubx_server --ublox /dev/ttyACM0 --isrover --ratems 200 --roverrtcmserver 127.0.0.1 --roverrtcmport 65100 --rovernmeaserver 65101
```

This rover will connect to the base station above, assuming it runs in the same computer (127.0.0.1). It will start a TCP server on port 65101 that outputs NMEA data at 5 Hz (200 ms).

