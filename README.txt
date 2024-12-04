Dr. Killpack,

to make your grading of our project easier I'm hoping I can point you in the right direction 
so that it's easier to understand what our code is doing

------------Important Files------------
navigationScene.py      - the PRM algorithm (the meat of our project) is implemented inside the NavigationScene class here.

test_navScene.py        - includes unit tests and visualization tests for various configurations
                            - test_* - unit tests to test basic functionality of important methods
                            - pest_* - visualization tests that don't have assert statements but are helpful for visually debugging
                            - mess_* - messing around with the networkx library we use for solvign the shortest path
                                       includes a helpful visualization

artifacts/              - includes a number of screenshots and videos illustrating various test cases from test_navScene.py
                          if you created a virtual environment with requirements.txt you can run all the tests and see that
                          they pass as well as run the "pest" visualizations to see the program in action.
                          BUT, if you're in a hurry and don't want to bother with that you can see all the basic functionality
                          of our program by looking at the videos and images in the artifacts/ folder. our program is so awesome
                          I want to make sure you don't miss the coolest part of seeing the cool c-space patterns 
                          and an animation of the robot doign it's path finding!


------------minor files------------
obstacle.py             - is a simple class to help define attributes of an obstacle

main.py                 - initial testing before we made the dedicated test_navScene.py

myLib/                  - libraries we developed earlier in the class like kinematics, visualization, etc.


------------rubric------------
