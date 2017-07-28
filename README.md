# piksi_surveyor
ROS package to survey incoming Piksi readings and output running average while saving readings to files.

Run the piksi_surveyor node to accumulate and obtain the running average of incoming Piksi readings. The terminal should display the running average of readings since the node was started and the sample count from each topic gps/fix and gps/rtkfix. The individual samples from both topics will be written to two seperate text files (formatted as CSV, gps_log.txt and gpsrtk_log.txt) in the home directory. Each time the node is run, those files will be overwritten with new samples, so move them as neccessary.
