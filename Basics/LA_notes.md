# LINEAR ALGEBRA
* In linear algebra when thinking of a vector always first think of an arrow present in the X-Y coordinate system with its tip of the tail is fixed at the origin.
* The coordinates of the vector then can be considered as the list of numbers.
  
  ![image1](../assets/Screenshot%20from%202022-08-22%2020-15-33.png)

* any vector can also be represented by sum of scaling of unit vectors of the three coordinate axes
  eg. suppose a vector has coordinates (x,y,z) then can be written as 
  xi+yj+zk
  This is also known as __basis__ of the vectors
* linear combination of two vectors is by considering those two vectors as the fixed vectors which are then scaled and added to get new vectors.
  ![image2](../assets//Screenshot%20from%202022-08-22%2020-42-34.png)
* think of individual vectors as arrows and multiple vectors as points
* Transformations is nothing but a function which takes in one vector and gives an output vector
* linear transformation for an infinite grid has two properties:

    1) all lines must remain straight
    2) the origin must not shift
* for linear transformation of any vector we must always track where does i and j goes
  ![image3](../assets/Screenshot%20from%202022-08-22%2021-14-41.png)
* ![image4](../assets/Screenshot%20from%202022-08-22%2021-19-27.png)
  here __a__ and __c__ are the coordinates where i lands and __b__ and __d__ are the coordinates where j lands in the new transformed grid.
* When there are two or more linear transformations involved then the tranformation matrix of the final transformation will be the product of matrix multiplication of all the linear transformation matrix.
  ![image5](../assets/Screenshot%20from%202022-08-23%2017-11-01.png)
* matrix multiplication is not commutative but is associative
* for 3 dimensions the basis vectors are i,j and k
* when you apply linear transformations in 3-d space you get a 3x3 matrix where each column specifies the transformation of individual basis
* The scaling factor by which linear transformation changes any area is known as its determinant.
* when we have negative determinants it means that the orientation of the transformation is opposite to the original transformation ie., they 'invert the orientation of the space'.
* For 3 dimansions determinant always tells by how much volume the transformation is scaled.
  ![image6](../assets/Screenshot%20from%202022-08-23%2017-43-55.png)
* Linear system of Equations:
  ![image7](../assets/Screenshot%20from%202022-08-23%2019-04-34.png)
  ![image8](../assets/Screenshot%20from%202022-08-23%2019-04-17.png)
* linear system of equations can be considered as finding a vector such that after applying the given transformations we get the desired vector. when determinant(A) is not equal to zero then we have a unique solution for x
* for determinant(A) is not equal to zero we can have an inverse transformation which when multiplied by A gives identity matrix.
* Rank of a matrix is the number of dimensions in the output of the transformation.
  ![image9](../assets/Screenshot%20from%202022-08-23%2019-13-59.png)
* when rank of a matrix is equal to the number of columns in the transformation it is known as full rank.
* for a full rank the only vector that lands on the origin is the zero vector.
* for a 3d transformation that squishes space onto a plane, there is an entire line that lands on the origin.
* for a 3d transformation that squishes space onto a line, there is an entire plane that lands on the origin.
* the set of vectors that land on the origin is called "null space" or "kernel" of your matrix. 
* When v in Ax=v is a null matrix, then the null space gives all the solution to the equation.
* when inverse of A does not exist, then column space helps to understand whether the solution exists and the null space helps us to understand what set of all possible solutions can look like.
* ![image10](../assets/Screenshot%20from%202022-08-23%2019-30-22.png)
* ![image11](../assets/Screenshot%20from%202022-08-23%2019-32-13.png)
* to find the changed transformation matrix in some other cordinate system, we first multiply any vector by the original transformation matrix, then we multiply the changing matrix in our coordinate system and thenwe multiply the inverse of the transformation matrix to convert the changed transformation matrix into the other cordinate system.
* ![image12](../assets/Screenshot%20from%202022-08-23%2019-37-44.png)
* ![image13](../assets/Screenshot%20from%202022-08-23%2019-40-15.png)
* When linear transformations are applied most of the vectors get knocked off from their spam, but there are some special vectors that remain on the their span and are just stretched to a particular value.
* such vectors are known as eigen vectors and the value is known as eigen values.
* ![image14](../assets/Screenshot%20from%202022-08-23%2019-44-35.png)
* ![image15](../assets/Screenshot%20from%202022-08-23%2019-47-11.png)
* for 3d transformations the eigen vectors are their axis of rotations, then the eigen values are 1 as they do not scale.
* ![image16](../assets/Screenshot%20from%202022-08-23%2019-51-13.png)
* ![image17](../assets/Screenshot%20from%202022-08-23%2019-51-35.png)
* ![image18](../assets/Screenshot%20from%202022-08-23%2019-54-04.png)
* in a transformation corresponding to diagonal matrix all the basis vectors are eigen vectors with the diagonal entries of the matrix being their eigen values.
* set of basis vectors which are eigen vectors are known as eigen basis.