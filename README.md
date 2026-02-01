# HondaCAN

This library was made and tested on a 2016 Honda Accord LX. It is designed to allow multiple vehicle profiles to be added and switched out from HondaCAN.h

The goal is to eventually have a small collection of parsed and labeled CAN data profiles. Instead of manually changing which vehicle profile to use in the code, the library should automatically find the make, model, year, and trim.
This is accomplished using a method from OpenDBC. It uses the VIN, as well as calibration IDs from a list of known ECU addresses. Even the existence of certain ECUs (fwd camera, LKAS/ADAS) can help determine optional trim features.

Unfortunately after changing a lot of features, my car was totaled and I am unable to test the new code. It will be uploaded as a second project until I can verify the changes didn't break anything. I plan on getting the same car but a higher trim.
