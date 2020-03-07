# ENPM 661 - Dijkstra Implementation on Point and Rigid Robot - Project 2 - Group 36

### Description

This program uses Dijkstra's algorithm to find the optimal path
in a given map for Point and Rigid robots.
The user can provide an initial state and a goal state,
and the algorithm will output vizualization of the 
cells explored and the optimal path in reaching the goal.

--------------------------------------

### Libraries

The solver uses the following Python libraries:

| Name      | Usage                                                             |
| --------- | ----------------------------------------------------------------- | 
| cv2       | OpenCV for vizualization                                          |
| numpy     | For scientific computing                                          |
| sys       | Outputting to stdout                                              |
| math      | To use the mathematical functions                                 |
| argparse  | Parsing input arguments to the program                            |

--------------------------------------

### Execution and Explanation

For Point Robot - 
  The program, Dijkstra_point.py, is run by executing the command `python Dijkstra_point.py`

For Rigid Robot - 
  Similarly, Dijkstra_rigid.py is run by executing `python Dijkstra_rigid.py`

Additional files, baseRobot.py and obstacleMap.py are run when the above execution happens.
baseRobot.py - It consists of a class from which an object created would have an instance of the main algorithms
such Dijkstra algorithm, checks for the cells in obstacles and outer map, 
backtracking required for the initial node to reach the goal node.
obstacleMap.py - The file creates a map of obstacles which are generated using half-planes.

If your python command is different, for example python3, adjust accordingly.
This script requires python 3.x to run.

--------------------------------------

### Arguments

The point robot program takes the x,y - coordinates of the initial and goal cells seperately as the arguments.
Inaddition to the coordinates the rigid robot file takes radius and clearance.

Help with using the program can be found by running the command `python solver.py --help`.

NOTE:  This program outputs an mp4 file.
If generating this mp4 file does not work, alternatively you can append the flag `--play` at the end
of the command to use openCV's built-in imshow command.
The mp4 file may still work even if there is an error message in the console related to FFmpeg.
Preferably omit the `--play` flag if the mp4 file is playable.

Some examples of valid commands:

        python Dijkstra_rigid.py 10 110 50 150 2 2 (arg : Ix Iy Gx Gy R C)

        python Dijkstra_point.py 160 10 10 290

        python Dijkstra_point.py 40 20 175 275 --play
--------------------------------------


### Output

While the program is running, it will output the vizualization of the cells being explored.
Once a solution has been found, the program outputs the cells explored.
The program also outputs an optimal sequence of actions to get
from the initial to the goal point.
