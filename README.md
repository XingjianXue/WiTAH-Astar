## Summary
In this paper, we propose a variant of the
anytime hybrid A* algorithm that generates a fast but
suboptimal solution before progressively optimizing
the paths to find the shortest winding-constrained
paths for a pair of tethered robots under curvature
constraints. Specifically, our proposed algorithm uses
a tangent graph as its underlying search graph and
leverages an anytime A* search framework with ap
propriately defined cost metrics in order to reduce
the overall computation and to ensure that a winding
angle constraint is satisfied. Moreover, we prove the
completeness and optimality of the algorithm for
finding the shortest winding-constrained paths in an
anytime fashion. 

## Results
Below are the paths and tether configurations under different winding angle constraints.  
The blue dashed line is the tether configuration of the suboptimal solution,  
while the red solid line is the optimal solution.

<p align="center">
  <div style="display:inline-block; text-align:center; margin:10px;">
    <img src="WiTAH Astar Simulation/0pi.gif" width="30%"/>
    <br/>0π
  </div>
  <div style="display:inline-block; text-align:center; margin:10px;">
    <img src="WiTAH Astar Simulation/1pi.gif" width="30%"/>
    <br/>1π
  </div>
  <div style="display:inline-block; text-align:center; margin:10px;">
    <img src="WiTAH Astar Simulation/1.5pi.gif" width="30%"/>
    <br/>1.5π
  </div>
</p>

<p align="center">
  <div style="display:inline-block; text-align:center; margin:10px;">
    <img src="WiTAH Astar Simulation/2pi.gif" width="30%"/>
    <br/>2π
  </div>
  <div style="display:inline-block; text-align:center; margin:10px;">
    <img src="WiTAH Astar Simulation/3pi.gif" width="30%"/>
    <br/>3π
  </div>
  <div style="display:inline-block; text-align:center; margin:10px;">
    <img src="WiTAH Astar Simulation/4pi.gif" width="30%"/>
    <br/>4π
  </div>
</p>

