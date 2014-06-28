Spectrum Shield (MSGEQ7) Driver v0.2  
------------------------------------
  
// Sparkfun Spectrum Shield Driver v0.2 (contributors: Tom Flock, John Boxall, Elijah Gregory)
// Written using Arduino IDE v1.0.5 and Processing v2.0.3 on Nov 3rd, 2013. by Tom Flock
// Based on Example 48.1 - tronixstuff.com/tutorials > chapter 48 - 30 Jan 2013 by John Boxall
// Running stats computations added June 25th, 2014 by Elijah J. Gregory

// This code receives multiplexed data from the Spectrum Shield's two MSGEQ7 ICs
// and transmits the values via serial at a 115200 baud rate.
// The fourteen values are seperated by commas and are terminated by a newline,
// which is the format your visualization code should expect.

// Currently the driver computes the running average, variance, and standard deviation for each
// input band. Upon initialization the mean is computed assuming zero input volume. This mean is
// used to baseline correct later values. The driver outputs the average values to create smooth
// visualization input. The standard deviation is used to detect changes in the process generating
// the input data and will be used to detect visualization start/stop times.
