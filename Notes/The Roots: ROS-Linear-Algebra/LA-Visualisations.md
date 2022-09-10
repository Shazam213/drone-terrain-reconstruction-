# Linear Algebra & Its Visualisations (3B1B)

### Chapter 02: Linear Combinations, Span & Basis Vectors

<!--UL-->
* In the XY-coordinate systems, there are two very special vectors:

<!-- Blockquote -->
<!--UL -->
> * The one pointing to the right with length 1, commonly called “i-hat” or the unit vector in the x-direction.
> * The one pointing straight up with length 1, commonly called “j-hat” or the unit vector in the y-direction.


* _i_-hat and _j_-hat are called the "**basis vectors**" of the standard coordinate system.

![BasisVectors](https://user-images.githubusercontent.com/95737452/189468015-785c515e-065d-44c6-b058-62c9439f6fc1.png)

* So, each vector in the XY-coordinate system can be obtained as a sum of two vectors obtained by the scaling (stretching/squishing/flipping) of the basis vectors of the XY system. This is called "**linear combination**" of the two vectors.

![02](https://user-images.githubusercontent.com/95737452/185761879-0124a0ab-7088-47b6-b368-5f447567c8b4.png)

* Intuition: When we fix one of the two scalars and let the other vary, the tip of the resultant vector obtained traces a straight-line path, hence “linear” combination of vectors.

* The span of two vectors v and w is the set of all their possible linear combinations.
![03](https://user-images.githubusercontent.com/95737452/185761896-0be0d191-dce3-416a-b4f8-143ee5b81b19.png)

* When the two vectors do not occur along the same line, each of them individually adds a dimension to the resultant vector, resulting in a 2D vector whose span is the entire 2D plane. When each vector adds a different dimension to our span, they are “**linearly independent**”.
![04](https://user-images.githubusercontent.com/95737452/185761937-4dc8fc32-ec09-4d6b-aa77-cb7fabb4ea5c.png) ![07](https://user-images.githubusercontent.com/95737452/185761951-6bab122f-b397-4192-96c7-ce0e3cbc3f61.png)

* However, when they line up, the span is all the vectors whose tip sits on a certain line. And when both scalars are zero in this case, we end up reaching the origin.<br> 
![05](https://user-images.githubusercontent.com/95737452/185761968-bd323d14-4378-4504-b299-58bb2c8fda50.png)
 
* When at least one of the vectors is redundant (does not add anything to our span; two vectors lining up) the vectors are said to be “**linearly dependent**”, where one of the vectors can be expressed as a linear combination of the others.
![06](https://user-images.githubusercontent.com/95737452/185761999-cc89d10c-4d18-40ba-974c-e1b07074f528.png)

* Technical Definition of Basis Vectors: The basis of a vector space is a set of linearly independent vectors that span the full space.

__________________________

### Chapter 03: Linear Transformations & Matrices

* Rules:
  * The origin must remain fixed.
  * The grid lines of the system must remain parallel and equidistant & must not get curved.

* To determine where the vector lands after transformation, we only have to find out where the two basis vectors i-hat and j-hat land after the transformation. <br>
![image](https://user-images.githubusercontent.com/95737452/185762484-8efc21a1-22a1-4cb1-afc6-48aa8f509a4b.png)

* So, for every vector which is a linear combination of the basis vectors, the transformed vector will also obey the same linear combination of the transformed basis vectors. <br>
![Chap03_03](https://user-images.githubusercontent.com/95737452/185762503-3f1823a0-a369-479e-92bb-054cda65dd79.png)

* If the transformed basis vectors are linearly dependent, then the linear transformation squishes all of 2D space onto the line on which the two vectors sit. <br>
![Chap03_01](https://user-images.githubusercontent.com/95737452/185762524-84c1adf7-f2bf-4dfb-8c2a-d0416280e9a0.png)

__________________________


### Chapter 04: Matrix Multiplication & Composition

* Composition: The combined effect of applying one transformation and then another.

* Multiplying two matrices has the geometric meaning of applying one transformation after another, which are applied from right to left (stems from function notation).

* The inner matrix gives the coordinates of the basis vectors after applying the first transformation, after which we can apply the transformation depicted by the second matrix by considering the columns of the first as two separate vectors and individually applying the transformations.

* By considering the geometric interpretation of matrix multiplication, we can understand why **order of multiplication in matrices holds significance**, but at the same time, **matrices are associative** (since the order of multiplication in this case remains same).

__________________________

### Chapter 05: Three-Dimensional Linear Transformations

* Geometrically kuch naya nahi hai, basically instead of a 2x2 matrix you have a 3x3 matrix, fairly obvious.

* Main crux vahi hai, to keep the gridlines evenly spaced and parallel in the transformation process.

__________________________

### Chapter 06: The Determinant

* The Determinant: **Exactly how much** does a transformation streches or squishes things? More specifically, the factor by which the area (2D) or volume (3D) occupied increases or decreases.

* If we know by how much the area of a single unit square formed by the basis vectors of the sytem changes, then by incorporating appropriate assumptions and size of squares as required, we can compute (or atleast, estimate) the factor by which the entire area of the shape changes. This follows from the fact that _gridlines remain equally spaced and parallel_. This _factor_ is technically the **determinant**.

* Negative values of the determinant imply a change in orientation of the shape post-transformation (flipping of space).

* Zero value of the determinant implies that the transformation squishes everything into a smaller dimension (dependent vectors).

* 3D transformations involve a change in volume. Linear transformations in 3D normally lead to the conversion of a 1x1x1 cube to a parallelopiped, with the exception of cases where dependent vectors (one vector is a scaled result of another) are involved (determinant = zero, all of space is squished onto a flat surface), where a 2D plane, line or a point (the origin) may also be obtained. 

* Orientation flip may also be incorporated in 3D, and can be understood using the Right Hand Rule.

__________________________

### Chapter 07: Inverse Matrices, Column Space & Null Space

* Linear System of Equations: Similar to matrix vector multiplication. Can be interpreted as the product of a matrix containing all the constant coefficients, a vector containing all the variables, and then their matrix vector product equals some different constant vector.

* Inverse matrices:
![image](https://user-images.githubusercontent.com/95737452/185761775-c086bcf3-d063-427c-b2ee-b2cafa1d447c.png)

 * For the matrix vector product Ax = v, the geometric interpretation of it can be thought as: the transformation (matrix A) that is applied on vector x to obtain vector v.
 * Now, when we operate in the reverse manner, i.e., when we want to find the transformation to be applied on vector v so that vector x can be obtained, the transformation so obtained is the **inverse matrix of A**.
 * When the determinant of matrix A is zero, it would imply that all of 2D space is being squished onto a single line, but the reverse transformation from a line to plane cannot be performed by any function. Hence, _inverse of matrices with zero determinant is **not defined**_.
 * In general, A inverse is the unique transformation with the property that if you first apply A then follow it with the transformation A inverse, you end up back where you started.
 * Applying one transformation after another is captured algebriacally with matrix multiplication, so the core property of this transformation A inverse is that A inverse times A equals the matrix that corresponds to doing nothing. _The transformation that does nothing is called the **identity transformation**_.

* Rank: Number of dimensions in the output transformation. _Full Matrix_: When the rank of the matrix is as high as it can be.
* Column Space: Set of all possibilities outputs of the matrix vector product: ![image](https://user-images.githubusercontent.com/95737452/185762396-6b599749-f0b6-419d-aa24-e42a0f6d7b40.png)
* Null Space: The set of vectors that land onto the origin post-transformation, aka, kernel. Space of all vectors that become null.

__________________________

### Chapter 13: Change of Basis

* Our two basis vectors i-hat and j-hat encapsulate all of the following implicit assumptions of our coordinate system:
![image](https://user-images.githubusercontent.com/95737452/185762720-3471b897-da18-4241-b5ca-369deb46ef88.png)

* i-hat and j-hat are the basis vectors of our _standard coordinate system_. A different coordinate sytem altogether can be obtained if one choses to work on a different set of basis vectors.
* Considering the following two basis vectors: <br>
![image](https://user-images.githubusercontent.com/95737452/185762807-d0866aa3-986f-4e57-9d17-9dab4a069c05.png)<br>
 Describing a vector ![image](https://user-images.githubusercontent.com/95737452/185762848-edf474bf-b27e-4b6f-91a9-2032a6553f6a.png) using the standard coordinate sytem, we get: <br>
![image](https://user-images.githubusercontent.com/95737452/185762863-a3e6e44d-698d-44fa-888f-edd87e3ed48d.png)<br>
However, with the alternative coordinate sytem, we get the coordinates of the same vector as:![image](https://user-images.githubusercontent.com/95737452/185762898-6fc09c51-c12b-4a81-9450-79e9aa4a0f3d.png) <br>![image](https://user-images.githubusercontent.com/95737452/185762907-ebaf4200-3023-4508-9c24-1cfe38a77cf3.png) ![image](https://user-images.githubusercontent.com/95737452/185762958-8ec20616-d2e1-4028-b2b9-924df1ffd9f4.png)
* To translate from the _alternative_ to the _standard_ coordinate sytem, we first find the coordinates of the basis vectors of the alternative CoSys in terms of the standard CoSys: <br>
![image](https://user-images.githubusercontent.com/95737452/185763000-40ce1d14-0586-405f-8180-5b5ac5dd2e55.png)<br>
Then, we understand the linear combination of the basis vectors of the alternative CoSys used to form the required vector in the alternative CoSys. <br>
Considering a vector with the coordinates ![image](https://user-images.githubusercontent.com/95737452/185763052-cbd9d995-8597-4ab9-86e1-6ae7bfdee587.png) in the alternative CoSys, we can understand this vector to be as follows: <br> ![image](https://user-images.githubusercontent.com/95737452/185763092-e0386a62-2bb7-4491-9c4d-c146327753d9.png)<br>
Hence effectively, this vector is: ![image](https://user-images.githubusercontent.com/95737452/185763124-b6e8ab07-44f5-4385-a1d7-67c030c22ac2.png)
Now, we substitute the basis vectors b1 and b2 in terms of their coordinates in the standard CoSys, and finally obtain the coordinates of the final vector in terms of the standard CoSys: <br> ![image](https://user-images.githubusercontent.com/95737452/185763190-a132db39-9b8c-4eca-bdf5-97676e1f8998.png) <br>
This entire process can be thought of as matrix-vector multiplication, where the matrix represents the coordinates of the basis vectors of the alternative CoSys in the standard CoSys, and the vector is the coordinates of the vector in the alternative CoSys.
* To translate from the _standard_ to the _alternative_ CoSys, the transformation to be applied is actually the inverse of the matrix you'd apply for conversion from alternative CoSys to standard CoSys. <br>
![image](https://user-images.githubusercontent.com/95737452/185763416-a62c74a3-bdb4-4a65-a50a-f338da101017.png)<br>
* ![image](https://user-images.githubusercontent.com/95737452/185763499-5fce6a8c-d140-41bc-b52e-e684ef99f5e7.png)<br>
* **Additional Reference:** https://www.youtube.com/watch?v=P2LTAUO1TdA&list=PLZHQObOWTQDPD3MizzM2xVFitgF8hE_ab&index=13 

__________________________


### Chapter 14: Eigenvectors and Eigenvalues

* The vectors that do not get knocked off their span after a particular transformation, i.e. the only effect that transformation matrix has on the vector is to either strech or squish it are called the "**eigenvectors**" of that transformation.<br>
![image](https://user-images.githubusercontent.com/95737452/185763668-dac2ef44-e0fc-4f65-a6e3-959298edeb3c.png) ![image](https://user-images.githubusercontent.com/95737452/185763682-c238d4dd-d64d-4796-bc61-1118b32d2e37.png) <br>
For example, for the specific transformation ![image](https://user-images.githubusercontent.com/95737452/185763709-70795cb8-a5bd-4480-a67d-c03c1b5b4c13.png), the vectors i-hat (and therefore, all vectors scaled from purely i-hat) and ![image](https://user-images.githubusercontent.com/95737452/185763749-8548a179-df03-48ac-b6e0-df5420b31da8.png) (and therefore, all vectors scaled from purely this vector)are eigenvectors for this transformation.<br>
![image](https://user-images.githubusercontent.com/95737452/185763802-8972caa4-193c-417c-9fdf-0293265bff5f.png)
* The factor by which eigenvectors get squished or streched is called their "**eigenvalues**".
* An important use: In a 3D rotation, when we find the eigenvector for a particular rotation, then that eigenvector is one of the axis of rotation for that object.
* Symbolically: <br> ![image](https://user-images.githubusercontent.com/95737452/185763923-d32cef78-219f-413d-a96b-3da9ae42a593.png)

__________________________










