import roslibpy, time
ros = roslibpy.Ros(host='10.0.0.234', port=10090)
ros.run(); print('connected?', ros.is_connected)
t = roslibpy.Topic(ros, '/hide_and_seek/target_spot', 'std_msgs/msg/Int32')
t.advertise()
t.publish(roslibpy.Message({'data': 9}))
time.sleep(0.5)
t.unadvertise()
ros.terminate()