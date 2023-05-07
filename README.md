# Urinay

Urinay is a color-blind path+tracklimits algorithm developed for computing the centerline and track limits of the Formula Student driverless autocross track without the need of sensing the cones' color. It uses Delaunay triangulation and a limited-heuristic-ponderated tree search and only takes the cone positions and the car pose. Made for [BCN eMotorsport Formula Student team](https://bcnemotorsport.upc.edu) by me (Oriol Gorriz) entirely in C++ and to work with ROS.

***NEW!*** For a **color dependant / hybrid** version check out the branch **`color`**.

## Disclaimer
If you use this algorithm for a Formula Student competition, the **only** thing I ask for is acknowledgment for the project. **ALWAYS REFERENCE** the team ***BCN eMotorsport***.

## Dependencies
- [Ubuntu](https://ubuntu.com) (tested on 20.04)
- [ROS](http://wiki.ros.org/ROS/Installation) (tested on Noetic)
- [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)
- *as_msgs*: The team's proprietary communication messages package. Change it for yours. See [this issue](https://github.com/origovi/urinay/issues/1) for more info.

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

Urinay solves this problem by implementing an iterative heuristic-ponderated height-limited tree search.

### 3.1. Tree search
Tree search provides a certain "intelligence" to the algorithm, of course we can iteratively append the best midpoint to the midline by just looking at the distance and angle but this will not end up with the best overall midline. We need to consider options that are not "the best" at a local sight.
This tree search gives **only** the next midpoint that belongs to the midline from a starting point. To obtain a full midline, multiple calls to the tree search will be performed.
The search is defined as follows:

1. The **starting point** will be the midline's closest point to the car, or the car's position if the midline is empty.
2. This point will be the root of the tree, steps 2-7 will be performed for each point *p* of the tree until a stop condition is found for every point. These are:
    - No possible midpoints are found.
    - The point's tree path reaches a maximum height threshold.
3. All midpoints within a **radius** of the point will be considered.
4. A filtering will be carried out, the following midpoints will be removed:
    - Any midpoint creating an **angle** too big with last point (avoid points behind and closed curves).
    - Any midpoint being in the same side of last edge, make sure the path **crosses** every midpoint edge (avoid bouncing on track limits).
    - [Not applied to first midpoint] Any midpoint **already contained** in the path (avoid creating incorrect loops).
    - Any midpoint whose edge is too big or too small compared to the midline average edge length (avoid incorrect midline).
    - Any midpoint that when appended to the path creates an intersection (on the path itself).
5. A **heuristic** value for each of the remaining midpoints will be calculated.
6. **Discard** all midpoints whose heuristic exceeds a threshold.
7. Append all remaining points as sons of *p*.
8. Find the best path. This will be the **longest path** (note that a path length will be at most the tree height). If two paths have equal length the one with smallest sum of heuristics will prevail.

### 3.2. Fail-safe
Fail-safe(s) allow Urinay to continue the path when certain circumstances cannot be met. When track layout not being perfect/adequate, the angle or distance from a certain midpoint to another is too big, so the search will stop there (effect of midpoint filtering). To solve this issue, Urinay implements a "general" fail-safe mechanism, i.e. when the car gets too close to the end of the path (aka midline), until the normal sight horizon gets recovered, higher values of these parameters (max angle, distance, etc) are set. During this extraordinary functioning, the sight horizon will be artificially limited (keep in mind that the objective is to get out of this "misbuilt" zone). Results can be seen in *Fig. 4*.

### 3.3. Loop closure
Obviously we want to detect and compute the whole track midline. Urinay does so by checking if the loop is closed every time a new midpoint is added to the midline. There are two problems here:
1. How we detect a loop closure? If the following conditions are met:
    - The midline has a **length** greater than a threshold.
    - The first and last points of the midline are **closer** than a threshold.
2. The tree search will never take points already contained in the midline. It can fail before reaching the end of the midline (finding a longer path) and it will fail taking the first midline's midpoint (already contained). We solve the problem by letting the tree search take all midpoints that close the loop and any midpoint (contained or not) **after** closing the loop.

## 4. Track limits computation
When guiding the car through the obtained midline, the planner (an algorithm that tells the car exactly where to go using the midline) will also need the track limits so the car might not follow the midline strictly but also optimize its path (knowing the track width at every moment).

To calculate the track limits having the midline is a relatively easy task knowing that every point in the midline has its correspondant edge in the triangle set. The two points that create this edge will be part of the track limits.

If we iterate the midline and we find which side of the track every point of the midpoint's edge belongs to, an aggregation of these points create the track limits.

## 5. Midline accumulation
We need to accumulate the midline so when the car moves we still know which was the midline the car has already driven through and to close the loop when we have already seen the whole track. There is another problem, we cannot compute again and again the midline from the beginning of the track because it would take longer to compute and it may fail if we lose the cones of the beginning (or a lot of false positives are detected).

This is why program has also the car's position in the track. Knowing where the car is at every moment gives the possibility to compute the midline from the car's position and then merge it with last iteration's midline, as seen in *Fig. 3*. This way, when a circular midline is detected, the program can communicate it to the path planning (and loop path optimizer) and stop.

## 6. Result
The result can be seen in *Fig. 3*. Urinay always chooses the best-longest way, this is why sometimes it wants to take an incorrect path but amends it once the correct path is perceived.

<p align="center">
  <img src="./documentation/assets/urinay_1.gif" alt="Urinay's path+tracklimits" width="500" /><br />
  Figure 3: Urinay's path+tracklimits
</p>

In *Fig. 4*, you can see at the left-most point of the path, there is a very tight turn. You can see that the path does not continue until the car gets close the end of it. Then, the fail-safe gets triggered and momentarily the path continues for only a limited length. When the car gets passed this situation, normal parameters and "long" path get restored.

<p align="center">
  <img src="./documentation/assets/urinay_colorBlind_failsafe.gif" alt="Urinay's fail-safe situation" width="500" /><br />
  Figure 4: Urinay's fail-safe situation.
</p>


## 7. Considerations
The following considerations have been taken into account when design and implementation of Urinay:
1. Cone position gets more accurate when car comes close to cones: It is assumed that the closer a cone is, the more probable it is that this cone has the real position. The algorithm is constantly updating the cones (and the path). This way the path just ahead of the car always is as accurate as possible.
2. Calculated path considered behind the car is always right: Cones belonging to a sector the car already went through may also move (due to perception). In this case, since the car has already been there, it is no further re-calculated.
3. Car may go off-track (or run over cones): Due to control or mechanical issues, the car may deviate from the calculated path. It is very important that the path does not follow the car (we want the car to go back to the correct path). To do so (and also achieving consideration 2.), the path is always calculated FROM the midpoint closest to the car.
4. No abusive use of pointers: In the program there are some places where data is duplicated with the intention to maintain modularity, make a procedure easier to undersand or to avoid a possible invalid pointer.
5. Maximum computating time guarantee: For this application, a solution which takes too long to compute is not realistic. This is the algorithm that "tells" the car the track path, if it takes too long, the car may crash into the cones. To avoid it, 3 mechanisms are used:
    - Tree height limit: The height of the search tree will always be limited to a parameter (usually 10, at maximum).
    - Tree sons numer limit: The number of next possible points will be cut down to a parameter (usually 2).
    - Computing time limit: Even if the mentioned restrictions are not sufficent to guarantee, a time limit for every tree search is applied. This way, if a particular search takes too long, e.g. when too many cone false positives are given, the best option when the time limit is hit will be considered as valid.