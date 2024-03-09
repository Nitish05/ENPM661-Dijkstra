## Dijkstra's Algorithm (ENPM 661 - Planning for Autonomous Robots)
This repository contains the implementation of Dijkstra's Algorithm for path planning of a point robot in a 2D grid space. The algorithm is implemented in Python and the visualization is done using the opencv library.

## Dependencies
- Python 3.7
- OpenCV
- Numpy
- Queue
- Time

## Usage
- Clone the repository
- Execute the dijkstra script using the following command: 
```bash
python dijkstra_nitish_ravisankar_raveendran.py
```
- The script will prompt the user to enter the start and goal coordinates. The origin (0,0) is at the bottom left corner of the grid space.
- If the start and goal coordinates are valid, the script will display the path from the start to the goal using the Dijkstra's Algorithm.
- If the start and goal coordinates are invalid, the script will prompt the user to enter the coordinates again.
- The script will also display the time taken to find the path and cost of the path.
- The script will also display the path in the map.
- The script will also save the path and the node creation as a video as dijkstra_nitish_ravisankar_raveendran.mp4 in the current directory.
- In the video, the start node is displayed in red, the goal node is displayed in green, the nodes visited are displayed in blue and the path is displayed in red.
- The script should not take more than 2 minutes to find the path.

GitHub repository link: https://github.com/Nitish05/ENPM661-Dijkstra.git