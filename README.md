Spectrum Shield (MSGEQ7) Driver v0.1  
------------------------------------
  
Sparkfun Spectrum Shield Driver v0.1 (contributors: Tom Flock, John Boxall, Elijah Gregory)  
Written using Arduino IDE v1.0.5 and Processing v2.0.3 on Nov 3rd, 2013. by Tom Flock  
Based on Example 48.1 - tronixstuff.com/tutorials > chapter 48 - 30 Jan 2013 by John Boxall  
Modified June 25th, 2014 by Elijah J. Gregory  
  
Currently the driver computes the running average upon initialization and uses this value  
to baseline correct later values. A 'smoothP' point running average is performed so that  
printed values are smoothed out in time.  
  
This code receives multiplexed data from the Spectrum Shield's two MSGEQ7 ICs  
and transmits the values via serial at a 115200 baud rate.  
The fourteen values are seperated by commas and are terminated by a newline,  
which is the format your visualization code should expect. 