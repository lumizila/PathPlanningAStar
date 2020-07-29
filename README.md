# PathPlanningAStar

### This is the repo of my first assignment of the Perspectives in Informatics 3 class.

In this assigment I implemented the A* path search algorithm. This assigment follows task "TA8" from Lecture 10. 

## Contents

1. The A* algorithm
2. Multiple runs of the program
3. Conclusion
4. Extra content
5. References

## 1. The A* algorithm

&emsp;A* was first published in 1968 by Peter Hart, Nils Nilsson and Bertram Raphael. 
It can be seen as an extention of the Dijkstra algorithm and it presents better performance than the Dijkstra algorithm. 
A* is a graph traversal algorithm to find a path that uses heuristics to guide the search (hence why it has better performance).
One of the drawbacks of A* is a high space complexity, since it needs to store all the generated nodes in memory. You can learn more about A* algorithm at https://en.wikipedia.org/wiki/A*_search_algorithm 

In pseudocode without extra details, the algorithm goes as follows:

```
create an openlist and add the starting node
create an empty closed list
   while (the open list is not empty):
       choose the node with the lowest f score in the open list
       if (this node is the destination node) :
           shows path from this node until the starting node by looking at each node's "parents"
       if not:
           put the current node in the closed list and look at all of its neighbors
           for (each neighbor of the current node):
               calculate new g,h,f values
               if (neighbor is not in the open list and not in the closed list) :
                  add it to the open list and set its g,f,h and set the parent node to be the chosen node      
               else if (neighbor is not in closed list and the new f value is < than the oldest one) :
                  update g,h,f values with new values for them and update parent node to be the chosen node
                   
```

&emsp;In A*, additional variables g,h,f are used as seen above. h is the estimated distance to the goal using a certain heuristic. In my program, h can be calculated by using diagonal distance or manhatan distance to goal. g is the known distance from the starting node to this current one (it is calculated by using the parent node's g plus 1). f is just a sum of g and h. 

## 2. Multiple runs of the program

To run the code, simply type:

```
python3 aStarPython.py 
```

&emsp; In order to increase flexibility when testing the program with different settings, the user is prompted with several questions as below: 

```
---> What is the height/width of the video in pixels? (must me multiple of 50) 
---> What is the seed you want to use for the random map? 
---> Do you want more weight near barriers (answer yes or no)? 
---> Press >m< for manhatan distance, >d< for diagonal distance when measuring distance to goal: 
```

  &emsp; Therefore the user can choose the size of the output video that will be produced; which seed they want for the random map; if going near barriers has added weights and if the calculation of the distance to the goal should be done using manhatan distance or diagonal distance. 

  &emsp; In order to test the code I decided to run it 3 times with different settings: with/without weight near barriers and with manhatan/diagonal distance. For all of the program runs I used the same random map generation seed "3". The output shows a path being built from start to finish on a map in form of video. The "black" squares represent barriers, "white" squares represent free spaces, "blue" squares represent free spaces however they are "harder to walk". The path going through dark blue squares costs more than that going through light blue squares, which in turn costs more than the path going through white squares. The "pink" square on the top left of the map represents the start, the "pink" square on the bottom right of the map represents the end. 

&emsp; The results of each of the different runs of the program are presented below. 

### 2.1 No weights near barriers, using manhatan distance

![GitHub Logo](/result1.png)

As seen in the image, this path is mostly diagonal. That is because the number barriers (in black) are not so big so the path is mostly open. 

### 2.2 With weights near barriers, using manhatan distance

![GitHub Logo](/result2.png)

This path shows large deviations(curves) compared to the previous result (without the weights near barriers).

### 2.3 With weights near barriers, using diagonal distance

![GitHub Logo](/result3.png)

This path is the same as the previous one in the beggining, however later it becomes mostly diagonal. 

## 3. Conclusion

&emsp; I believe map with weights near barriers is more close to a real world scenario where walking near obstacles is more risky. In that sense, the best path was done by using manhatan distance to calculate h value (test 2) because that is the one the path was furthest from the barriers most of the time. In reality when I started my algorithm I was only using diagonal distance and I realized diagonal distance gives too much preference for a diagonal path, which is why I decided to use the option of manhatan distance. I notice similar results with different seeds to generate the random maps as well. 
In the future I would like to add other distance to goal calculations for testing such as Euclidean Distance.

## 4. Extra content

The code for this assigment, as well as videos of the paths mentioned above being traced can be found at: https://github.com/lumizila/PathPlanningAStar

## 5. References

As a reference for my code I used explanations from this website: https://www.geeksforgeeks.org/a-search-algorithm/ as well as the slides of lecture 10. 

