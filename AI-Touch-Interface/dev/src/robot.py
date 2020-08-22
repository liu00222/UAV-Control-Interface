class Robot:
    def __init__(self, id, x, y, z, length, width, height, type):
        self.id = str(id)
        self.x = x
        self.y = y
        self.z = z
        self.length = length
        self.width = width
        self.height = height
        self.type = type
    
    def get_id(self):
        """
        Return the ID of the robot

        :return: <string> the ID of the robot
        """
        return self.id
        
    def set_position(self, x, y, z):
        """
        Set the position of the robot

        :param x: <float> x coordinate
        :param y: <float> y coordinate
        :param z: <float> z coordinate
        """
        self.x = x
        self.y = y
        self.z = z
