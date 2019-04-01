echo echo accelerator on F130 E30
rostopic pub /r1/servo/F std_msgs/UInt16 --once -- 130
rostopic pub /r1/servo/E std_msgs/UInt16 --once -- 30
