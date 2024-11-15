import numpy as np

def printArr(array, precision: int = 3):
    """
    Prints a numpy array nicely.
    Pass in number of decimals, or default is 3.
    """
    print(np.array2string(array, precision=precision, suppress_small=True))
