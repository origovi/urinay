# Urinay

Urinay is a path+tracklimits algorithm developed for computing the track midline and track limits of a Formula Student track. It uses Delaunay triangulation and a limited-heuristic-ponderated tree search taken only the cone positions and the car position. Made for [BCN eMotorsport Formula Student team](https://bcnemotorsport.upc.edu) by me.

## 1. Delaunay Triangulation
The first step of this approach consists in obtaining the Delaunay triangulation (a set of triangles) using the detected cones as points in a 2D space.
This set is computed using my implementation of the [Bowyer-Watson algorithm](https://en.wikipedia.org/wiki/Bowyer%E2%80%93Watson_algorithm). This is an iterative process that is computed in O(*n*log*n*), being *n* the number of points. The execution time of the triangulation with a high number of cones is approximately 1ms. *Fig. 1* shows the triangle set (red) and the midpoints of every edge (green).

<p align="center">
  <img src="./documentation/assets/urinay_triangulation_1.png" alt="Delaunay triangulation" width="500" /><br />
  Figure 1: Delaunay triangulation
</p>

## 2. Filtering and midpoints
Next step would be to find the midline of the track using the midpoints, but as shown in *Fig. 1*, there are so many midpoints that can lead to a misscalculation of the track's midline. To avoid this issue, we filter:
- The triangles: remove the ones having large edges or tiny angles.
- The midpoints: remove the ones not having a triangle circumcenter near.

As shown in *Fig. 2*, we can already see a very clear path (midline) out of the green points.

<p align="center">
  <img src="./documentation/assets/urinay_triangulation_3.png" alt="Filtered Delaunay triangulation" width="500" /><br />
  Figure 2: Filtered Delaunay triangulation
</p>

## 3. Midline calculation
Now we need to obtain a midline from the filtered midpoints. This midline will be an array of midpoints in the order in which the car will reach them.

Urinay solves this problem by implementing an iterative heuristic-ponderated height-limited tree search. For every possible next point the angle and distance with the last point will be calculated. This heuristic will say how "good" or "bad" a next point is. Only the *n* best options will be considered (normally *n* = 2). The height of this tree will be limited so no extreme computation time will be required. Now that we have a tree, we need to choose the best option, Urinay considers it as the longest one with minimal heuristic. The first point of this option will be considered as a point of the midline. It also removes points already in the midline from the next possible points.

This procedure will be repeated until next points have a hauristic greater than a threshold. This means that a tree search has been performed for every point in the obtained midline.

## 4. Track limits computation
When guiding the car through the obtained midline, the planner (an algorithm that tells the car exactly where to go using the midline) will also need the track limits so the car might not follow the midline strictly but also optimize its path (knowing the track width at every moment).

To calculate the track limits having the midline is a relatively easy task knowing that every point in the midline has its correspondant edge in the triangle set. The two points that create this edge will be part of the track limits.

If we iterate the midline and we find which side of the track every point of the midpoint's edge belongs to, an aggregation of these points create the track limits.

## 5. Midline accumulation
We need to accumulate the midline so when the car moves we still know which was the midline the car has already driven through and to close the loop when we have already seen the whole track. There is another problem, we cannot compute again and again the midline from the beginning of the track because it would take longer to compute and it may fail if we lose the cones of the beginning (or a lot of false positives are detected).

This is why program has also the car's position in the track. Knowing where the car is at every moment gives the possibility to compute the midline from the car's position and then merge it with last iteration's midline, as seen in *Fig. 3*. This way, when a circular midline is detected, the program can communicate it to the path planning (and loop path optimizer) and stop.

## 6. Result
The result can be seen in *Fig. 3*. Although the car's position is not displayed, it remains always far behind the last midpoint.

<p align="center">
  <img src="./documentation/assets/urinay_1.gif" alt="Urinay's path+tracklimits" width="500" /><br />
  Figure 3: Urinay's path+tracklimits
</p>

## 7. Safety considerations
The following considerations have been taken into account when design and implementation of Urinay:
1. No abusive use of pointers: In the program there are some places where data is duplicated with the intention to maintain modularity, make a procedure easier to undersand or to avoid a possible invalid pointer.
2. Maximum computating time guarantee: For this application, a solution which takes too long to compute is not realistic. This is the algorithm that "tells" the car the track path, if it takes too long, the car may crash into the cones. To avoid that 3 mechanisms are used:
    - Tree height limit: The height of the search tree will always be limited to a parameter (usually 10, at maximum).
    - Tree sons numer limit: The number of next possible points will be cut down to a parameter (usually 2).
    - Computing time limit: Even if the mentioned restrictions are not sufficent to guarantee, a time limit for every tree search is applied. This way, if a particular search takes too long, e.g. when too many cone false positives are given, the best option when the time limit is hit will be considered as valid.