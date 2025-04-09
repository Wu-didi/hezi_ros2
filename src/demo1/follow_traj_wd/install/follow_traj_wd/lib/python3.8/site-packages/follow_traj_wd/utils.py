
class Bbox:
    """用于存放每个检测框信息的简单类"""
    def __init__(self, x, y, z, w, l, h, theta, score, label):
        self.x = x
        self.y = y
        self.z = z
        self.w = w
        self.l = l
        self.h = h
        self.theta = theta
        self.score = score
        self.label = label

    def __str__(self):
        return (f"Bbox(x={self.x}, y={self.y}, z={self.z}, w={self.w}, "
                f"l={self.l}, h={self.h}, theta={self.theta}, "
                f"score={self.score}, label={self.label})")
        
        
class State:
    """
    vehicle state class
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.predelta = None
