echo tirerMarcheArriere 30 30
rostopic pub /r1/servo/E std_msgs/UInt16 --once -- 30
rostopic pub /r1/servo/F std_msgs/UInt16 --once -- 30
