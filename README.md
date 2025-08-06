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
Below are the paths and tether configurations under different winding angle constraints(ω = 10, γ = 10)
The blue dashed line is the tether configuration of the suboptimal solution, while the red solid line is the optimal solution.
<p align="center">
  <img src="WiTAH Astar/0pi.png" width="30%" />
  <img src="WiTAH Astar/1.5pi.png" width="30%" />
  <img src="WiTAH Astar/3pi.png" width="30%" />
</p>
<p align="center">
  <img src="WiTAH Astar/1pi.png" width="30%" />
  <img src="WiTAH Astar/2pi.png" width="30%" />
  <img src="WiTAH Astar/4pi.png" width="30%" />
</p>
