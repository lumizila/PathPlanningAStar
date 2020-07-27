# PathPlanningAStar

### This is the repo of my first assignment of the Perspectives in Informatics 3 class.

In this assigment I implemented the A* path search algorithm. This assigment follows task "TA8" from Lecture 10. 

## Contents

1. The A* algorithm
2. Multiple runs of the algorithm
3. Conclusion
4. Extra content
5. References

## 1. The A* algorithm

## 2. Multiple runs of the algorithm

To run the code, simply type:

```
python3 aStarPython.py 
```

The output of the program is a video that shows how a robot would follow the path found by the algorithm. 
In order to increase flexibility when testing the program with different settings, the user is prompted with several questions as below: 

```
---> What is the height/width of the video in pixels? (must me multiple of 50) 
---> What is the seed you want to use for the random map? 
---> Do you want more weight near barriers (answer yes or no)? 
---> Press >m< for manhatan distance, >d< for diagonal distance when measuring distance to goal: 
```

Therefore the user can choose the size of the video that will be produced; which seed they want for the random map; if going near barriers has added weights and if the calculation of the distance to the goal should be done using manhatan distance or diagonal distance. 

In order to test the code I decided to run it 3 times with different settings: with/without weight near barriers and with manhatan/diagonal distance.
The results of each of them are presented in below. 

### 2.1 No weights near barriers, using manhatan distance
### 2.2 With weights near barriers, using manhatan distance
### 2.3 With weights near barriers, using diagonal distance

## 3. Conclusion

## 4. Extra content

The code for this assigment can be found at: https://github.com/lumizila/PathPlanningAStar

## 5. References

As a reference for my code I used explanations from this website: https://www.geeksforgeeks.org/a-search-algorithm/ as well as the slides of lecture 10. 

