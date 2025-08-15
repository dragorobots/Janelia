import roslibpy

class RosLink:
    def __init__(self, host, port=10090):
        self.ros = roslibpy.Ros(host=host, port=port)
        self.target = roslibpy.Topic(self.ros,'/hide_and_seek/target_spot','std_msgs/msg/Int32')
        self.toggle = roslibpy.Topic(self.ros,'/hide_and_seek/toggles','std_msgs/msg/String')
        self.cmdvel = roslibpy.Topic(self.ros,'/hide_and_seek/cmd_vel','geometry_msgs/msg/Twist')
        self.line_color = roslibpy.Topic(self.ros,'/hide_and_seek/line_color','std_msgs/msg/String')
        self.lf     = roslibpy.Topic(self.ros,'/line_follow/status','std_msgs/msg/String')
        self.rat    = roslibpy.Topic(self.ros,'/rat_detection/found','std_msgs/msg/Bool')
        self.prog   = roslibpy.Topic(self.ros,'/hide_and_seek/progress','std_msgs/msg/String')

    def connect(self, on_lf=None, on_rat=None, on_prog=None):
        self.ros.run()
        self.target.advertise(); self.toggle.advertise(); self.cmdvel.advertise(); self.line_color.advertise()
        if on_lf:  self.lf.subscribe(lambda m: on_lf(m['data']))
        if on_rat: self.rat.subscribe(lambda m: on_rat(bool(m['data'])))
        if on_prog:self.prog.subscribe(lambda m: on_prog(m['data']))

    def send_target(self, i):
        self.target.publish(roslibpy.Message({'data': int(i)}))

    def send_toggle(self, s):
        self.toggle.publish(roslibpy.Message({'data': str(s)}))

    def send_line_color(self, hue):
        self.line_color.publish(roslibpy.Message({'data': f'hue={hue}'}))

    def send_cmdvel(self, v, w):
        self.cmdvel.publish(roslibpy.Message({
            'linear':  {'x': float(v), 'y': 0.0, 'z': 0.0},
            'angular': {'x': 0.0, 'y': 0.0, 'z': float(w)}
        }))

    def close(self):
        for t in [self.target,self.toggle,self.cmdvel,self.line_color,self.lf,self.rat,self.prog]:
            try: t.unadvertise()
            except: pass
        self.ros.terminate()

    @property
    def connected(self):
        return self.ros.is_connected
