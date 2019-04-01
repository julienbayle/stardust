echo portergolden
rostopic pub /r1/servo/E std_msgs/UInt16 --once -- 100
rostopic pub /r1/servo/F std_msgs/UInt16 --once -- 128
