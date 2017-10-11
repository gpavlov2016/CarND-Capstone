import math

def distance(a, b):
    """ Distance between two positions
    """
    return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
# Mile per hour to meter per second.
def mph2mps(x): return (x * 0.447)

# Meter per second to mile per hour.
def mps2mph(x): return (x / 0.447)