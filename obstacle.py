from typing import List

class Obstacle():

    def __init__(self,coords: List[float],radius: float):
        """
        ENTER ARGUMENTS IN METERS,   
        represents a spherical obstacle  
            coords: (x,y,z) coordinates representign location of the obstacle  
            radius: radius of the sphere  
        """

        self.location = coords
        self.radius =   radius

    