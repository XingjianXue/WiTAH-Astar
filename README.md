## Summary
In this paper, we propose a variant of the anytime hybrid A* algorithm that generates a fast but suboptimal solution before progressively optimizing the paths to find the shortest winding-constrained paths for a pair of tethered robots under curvature constraints. Specifically, our proposed algorithm uses a tangent graph as its underlying search graph and leverages an anytime A* search framework with appropriately defined cost metrics in order to reduce the overall computation and to ensure that a winding angle constraint is satisfied. Moreover, we prove the completeness and optimality of the algorithm for finding the shortest winding-constrained paths in an anytime fashion. 

## Results
Below are the paths and tether configurations under different winding angle constraints. The blue dashed line is the tether configuration of the suboptimal solution, while the red solid line is the optimal solution.

## Results
Below are the paths and tether configurations under different winding angle constraints.  
The blue dashed line is the tether configuration of the suboptimal solution,  
while the red solid line is the optimal solution.

| 0π | 1π | 1.5π |
|----|----|------|
| <img src="WiTAH Astar Simulation/0pi.gif" width="250"/> | <img src="WiTAH Astar Simulation/1pi.gif" width="250"/> | <img src="WiTAH Astar Simulation/1.5pi.gif" width="250"/> |

| 2π | 3π | 4π |
|----|----|------|
| <img src="WiTAH Astar Simulation/2pi.gif" width="250"/> | <img src="WiTAH Astar Simulation/3pi.gif" width="250"/> | <img src="WiTAH Astar Simulation/4pi.gif" width="250"/> |

