<h1> Comparing Pre-Processing Algorithms </h1>

*Overview* : Principal Component Analysis and Moving Least Squares are both techniques that can be used to consolidate our data set, so that it only includes points that contribute significantly to surface normal estimation. A comparison of the results of the two algorithms for the terrain file **CSite1_orig-utm.pcd** has been presented below. The source file can be found in the repository.

<h3> CSite1_orig-utm.pcd Results </h3>

Original File: <br>

![image](https://user-images.githubusercontent.com/95737452/189100023-006a37f1-5c18-43b6-ba96-edcd0e759acb.png) <br>

At searchRadius  = 0.03, following results were obtained: <br>
*By PCA:* 
<br>
![CSite_PCA](https://user-images.githubusercontent.com/95737452/189099334-32f27dbb-5138-4555-8eef-5d258f4c1eca.png)<br>

*By MLS:* <br>
![CSite_PCA](https://user-images.githubusercontent.com/95737452/189099494-349d4600-a0fd-4f97-944f-40b73702ccb5.png)<br>

Increasing the searchRadius to 30, the results obtained were as follows: <br>

*By PCA:* <br>
![image](https://user-images.githubusercontent.com/95737452/189099818-55a0c991-742c-459e-916e-2048028683f3.png) <br>

*By MLS:* <br>
![image](https://user-images.githubusercontent.com/95737452/189099918-92fc4c8e-221b-4c23-b1ab-3df34d228ccc.png) <br>

<h3> Conclusion </h3>

* Clearly in both cases of the searchRadius, **MLS gave a better output than PCA** and therefore, *might* be better suited for industry application.
* The uncertainty of it's application lies in the fact that in some examples other than our particular use case, PCA might produce better results, which so far aren't known to us.
* A better understanding of application of both algorithms would thus be developed after considering the results of both post the use of mesh reconstruction algorithms on them.



