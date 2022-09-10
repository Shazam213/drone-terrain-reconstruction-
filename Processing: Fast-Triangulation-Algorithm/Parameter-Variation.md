<h1> Fast Triangulation & Associated Parameters </h1>

<!-- Blockquote -->
> * The approach for creating the triangular mesh is based on the *incremental surface growing principle*.
> * Our algorithm selects a starting triangle’s vertices and connects new triangles to it until either all points are considered or no more valid triangles can be connected. In the second case a new seed triangle is placed in the unconnected part and the triangulation is restarted.

<h2> Defining Parameters </h2>

* **setMaximumNearestNeighbors**(unsigned int) and **setMu**(double): They control the size of the neighborhood. The former defines how many neighbors are searched for, while the latter specifies the maximum acceptable distance for a point to be considered, relative to the distance of the nearest point (in order to adjust to changing densities).
* **setSearchRadius**(double) is practically the maximum edge length for every triangle. This has to be set by the user such that to allow for the biggest triangles that should be possible.
* **setMinimumAngle**(double) and **setMaximumAngle**(double) are the minimum and maximum angles in each triangle.
* **setMaximumSurfaceAgle**(double) and **setNormalConsistency**(bool) are meant to deal with the cases where there are sharp edges or corners and where two sides of a surface run very close to each other. To achieve this, points are not connected to the current point if their normals deviate more than the specified angle (note that most surface normal estimation methods produce smooth transitions between normal angles even at sharp edges). This angle is computed as the angle between the lines defined by the normals (disregarding the normal’s direction) if the normal-consistency-flag is not set, as not all normal estimation methods can guarantee consistently oriented normals. Typically, 45 degrees (in radians) and false works on most datasets.
