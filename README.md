# Point-Cloud-Analysis
This repository contains Python code for analyzing point cloud data, including implementations of plane and sphere fitting algorithms using RANSAC and ICP

The point cloud fitting functions are implemented in `fitting.py` along with helper functions in `utils.py`

The different functions implemented can be run by `point_cloud(input)` where `input = {q1_a,q1_b,q1_c,q2,q3,q4_a,q4_b,q4_c}` where the executions are as follows:
* `q1_a` :  fit a plane by calculating the sample mean and covariance matrix of 100 point in 3D space. Here, the points are generated such that there are no outliers that do not actually fit a plane, i.e, they are along a plane.

![image](https://github.com/josejosepht/Point-Cloud-Analysis/assets/97187460/6270e1b6-ac53-408d-88ca-c5470f215749)

* `q1_b` : This time, we have outliers that do not actually lie on the plane. Let's see how the sample mean and covariance matrix method performs in fitting a plane to the points.(Notice the outliers from the fitted plane)

![image](https://github.com/josejosepht/Point-Cloud-Analysis/assets/97187460/56a61862-7ed9-4b18-b9ee-81b5efd902d8)
![image](https://github.com/josejosepht/Point-Cloud-Analysis/assets/97187460/d32ecf02-d15d-4f68-9246-22ffd012a406)

* `q1_c` : RANSAC implementation to fit the plane. From the output figure, we see that the RANSAC approach is robust to outliers and gives a better fit according to the subset of point cloud that is close to a plane. SVD is easier in implementation but susceptible to outliers while RANSAC would be a bit more compex to implement while being resilient to outliers.

![image](https://github.com/josejosepht/Point-Cloud-Analysis/assets/97187460/02c27e28-1e90-4cf9-851d-a6463b67f6ec)
![image](https://github.com/josejosepht/Point-Cloud-Analysis/assets/97187460/b2c51c54-7661-4b05-a4a9-1b2229433ddc)

* `q2` : Using RANSAC to localize and fit a sphere in object3d.npy

![image](https://github.com/josejosepht/Point-Cloud-Analysis/assets/97187460/3773c072-99c9-4f7a-a0ef-41f6f3756700)

* `q3` : Using RANSAC to localize and fit a cylinder in objected.npy

![image](https://github.com/josejosepht/Point-Cloud-Analysis/assets/97187460/c1335ca3-8960-4f4d-833b-43153d8ccae7)

* `q4_a` : Finding the transformation matrix that aligns two point clouds given full correspondences between points in the two clouds. The point clouds are centered and normalized and then SVD is applied.

![image](https://github.com/josejosepht/Point-Cloud-Analysis/assets/97187460/9f1431f8-bdbc-4f2c-b04e-353d160d24ca)

* `q4_b` : Gaussian noise is induced into the transformed matrix passed to q4_a(). Note that the correspondences are still kept and hence the underlying geometry of the point clouds not lost. The figure below on the right shows the output when the point clouds are shuffled because correspondences are lost.

![image](https://github.com/josejosepht/Point-Cloud-Analysis/assets/97187460/8278bddd-6fff-457a-975b-df4b9c806efb)


* `q4_c` : ICP is implemented to to generate transformation T to best align the points clouds

![image](https://github.com/josejosepht/Point-Cloud-Analysis/assets/97187460/939fc865-2a4a-4e07-bf7a-da1c3fecaaf4)
