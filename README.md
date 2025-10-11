# secret_heights
Question 1: Determine the altitude of the airplane
Problem: Our aircraft have two downward-facing laser altimeters (which report the airplane’s
height above ground in meters) and a GPS (which reports the plane’s altitude relative to mean
sea level). When we are flying down low (below ~15 meters), we care very much exactly how far
above the ground the aircraft is, in order to maintain a precise height for spraying and landing.
As the plane gets higher, it tends to fly over rougher terrain, and it becomes more important to
fly a smooth trajectory through space than to precisely follow the contour of the ground below.
Also, due to dust, sensor noise, and other phenomena, one or both of the altimeters will
sometimes report height readings that are not reflective of the plane’s actual height above the
ground. These incorrect readings will be physically impossible (e.g., they will jump around a lot
between successive readings, 10ms apart). The naive approach of just using the data from the
altimeters is clearly not sufficient to produce an accurate, smooth estimate of the plane’s current
height above the ground.
Your task is to write a C++ program that reads in the real airplane log data we’ve provided (as
CSV files) of both correct and incorrect altimeter behavior and outputs (to standard out) a
running estimate of the plane’s true height above ground level. This estimate sho