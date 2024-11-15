import sympy as sp
from sympy import sin,cos,symbols,latex,Matrix
from IPython.display import display,Math
from typing import List

def generateSymbolicTransformMatrixFromDHparams(numJoints=0) -> List[Matrix]:
    if numJoints==0:
        raise TypeError("please enter the number of joints")


    result: List[Matrix] = []
    for i in range(numJoints):
        jointNum = i+1
        theta,d,a,alpha = symbols(f'\\theta_{jointNum} d_{jointNum} a_{jointNum} \\alpha_{jointNum}')
        A = sp.Matrix([  [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
                        [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
                        [0, sin(alpha), cos(alpha), d],
                        [0, 0, 0, 1]
                        ])
        result.append(A)
    return result

def sppl(string: str):
    """
    pass in a string in latex format:
        f'//theta_i' 
    this will print it out all pretty in jupyter notebooks
    """
    display(Math(latex(symbols(string))))
    