    #!/usr/bin/env python
                
#from time import gmtime, strftime
import time
import serial
import couchdb           
import datetime
      
ser = serial.Serial(    
	port='/dev/ttyUSB0',
        baudrate = 9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1
)
counter=0

user = "wtk9d1av6Z"
password = "COOGykR3wU"
couchserver = couchdb.Server("http://%s:%s@52.65.170.67:5984/"%(user, password))

dbname = "telemetry_db"
if dbname in couchserver:
	db = couchserver[dbname]
else:
	print "db not found"

for dbname in couchserver:
	print dbname
          
sessionID = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

while 1:
	x=ser.readline()
        print x
	if x.find("Got: ") != -1:
		val = x[5:]
		print val
		currentDateTime = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
		currentKey = "IFB102[" + currentDateTime +  "]"			
		db.save({'key': currentKey, 'session': sessionID, 'time':currentDateTime, 'value': val})
