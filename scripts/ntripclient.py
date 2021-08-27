#!/usr/bin/python

import rospy

#from nmea_msgs.msg import Sentence
from ntrip_ros.msg import RTCM

import datetime
import socket
import httplib
from base64 import b64encode
from threading import Thread
from std_msgs.msg import String

class ntripconnect(Thread):

    def __init__(self, ntc):
        super(ntripconnect, self).__init__()
        self.ntc = ntc
        self.stop = False


    def run(self):
        self.rmsg = RTCM()
        connection, connected = self.connect()
        while not self.stop:
            try: 
                if not connected:
                    connection, connected = self.connect()
                else:
                    self.parse_and_pub()
            except (socket.error, httplib.IncompleteRead) as ex:
                print 'Exception: ', ex
                connected = False
                rospy.sleep(2.0)
                continue
        
        print "Closing connection"
        connection.close()

    def connect(self):
        headers = {
            'Ntrip-Version': 'Ntrip/2.0',
            'User-Agent': 'NTRIP ntrip_ros',
            'Connection': 'close',
            'Authorization': 'Basic ' + b64encode(str(self.ntc.ntrip_user) + ':' + self.ntc.ntrip_pass)
        }
        connection = httplib.HTTPConnection(self.ntc.ntrip_server, timeout=5.0)
        now = datetime.datetime.utcnow()
        nmeadata = self.ntc.nmea_gga % (now.hour, now.minute, now.second)

        csum = 0
        for c in nmeadata:
               # XOR'ing value of csum against the next char in line
               # and storing the new XOR value in csum
               if ord(c)!=',':
                  csum ^= ord(c)

        #convert hex characters to upper case
        csum = hex(csum).upper()

        #add 0x0 if checksum value is less than 0x10
        if len(csum)==3:
            csum='0'+csum[2]
        else:
            csum=csum[2:4]

        nmeastring = '$'+nmeadata+'*'+csum

        connection.request('GET', '/'+self.ntc.ntrip_stream,nmeastring,headers)

        self.response = connection.getresponse()
        if self.response.status != 200: 
            raise Exception("blah")
            return connection, False
        else:
            return connection, True



    def parse_and_pub(self):
        print "reading data", self.rmsg.header.seq
        data = self.response.read(1)
        if data!=chr(211):
            print "DATA ERROR"
            return
        l1 = ord(self.response.read(1))
        l2 = ord(self.response.read(1))
        pkt_len = ((l1&0x3)<<8)+l2
        print 'data: ', data, 'packet len: ', pkt_len
        pkt = self.response.read(pkt_len)
        parity = self.response.read(3)
        if len(pkt) != pkt_len:
            rospy.logerr("Length error: {} {}".format(len(pkt), pkt_len))
            return
        self.rmsg.header.seq += 1
        self.rmsg.header.stamp = rospy.get_rostime()
        self.rmsg.data = data + chr(l1) + chr(l2) + pkt + parity
        self.ntc.rtcm_pub.publish(self.rmsg)
        print "done"

class ntripclient:

    def __init__(self):
        rospy.init_node('ntripclient', anonymous=True)

        self.ntrip_server = rospy.get_param('~ntrip_server')
        self.ntrip_user = rospy.get_param('~ntrip_user')
        self.ntrip_pass = rospy.get_param('~ntrip_pass')
        self.ntrip_stream = rospy.get_param('~ntrip_stream')

        # NMEA GGA data is needed to send to NTRIP server to keep connection
        # Should replace with real GGA msg from our fix or nmea msg
        self.nmea_gga = rospy.get_param('~nmea_gga')

        self.rtcm_pub = rospy.Publisher('rtcm', RTCM, queue_size=10)

        self.connection = None
        self.connection = ntripconnect(self)
        self.connection.start()


    def run(self):
        rospy.spin()
        if self.connection is not None:
          self.connection.stop = True
          rospy.loginfo("Connection stopped")



if __name__ == '__main__':
    c = ntripclient()
    c.run()

