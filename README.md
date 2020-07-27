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

#TODO explain A*

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

### 2.2 With weights near barriers, using manhatan distance

![GitHub Logo](/result2.png)

### 2.3 With weights near barriers, using diagonal distance

![GitHub Logo](/result3.png)


## 3. Conclusion

&emsp; #TODO compare and conclude

## 4. Extra content

The code for this assigment can be found at: https://github.com/lumizila/PathPlanningAStar

## 5. References

As a reference for my code I used explanations from this website: https://www.geeksforgeeks.org/a-search-algorithm/ as well as the slides of lecture 10. 

