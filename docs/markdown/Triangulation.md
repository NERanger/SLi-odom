# Triangulation

This is a explanation for function `bool Triangulation(const std::vector<Sophus::SE3d> &poses, const std::vector<Vec3> points, Vec3 &pt_world)`

## Implementation

* Header file: `include/sli_slam/Algorithm.hpp`
* Source file: `src/sli_slam/Algorithm.cc`

```cpp
bool sli_slam::Triangulation(const vector<SE3d> &poses, 
                             const vector<Vec3> points, 
                             Vec3 &pt_world) {

    MatXX A(2 * poses.size(), 4);
    VecX b(2 * poses.size());
    b.setZero();
    for (size_t i = 0; i < poses.size(); ++i) {
        Mat34 m = poses[i].matrix3x4();
        A.block<1, 4>(2 * i, 0) = points[i][0] * m.row(2) - m.row(0);
        A.block<1, 4>(2 * i + 1, 0) = points[i][1] * m.row(2) - m.row(1);
    }

    BDCSVD<MatrixXd> svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    pt_world = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();

    if (svd.singularValues()[3] / svd.singularValues()[2] < 1e-2) {
        // Bad solution, discard
        return true;
    }
    
    return false;
}
```

## Problem Formulation

Assuming we have a stereo camera, the pose of the left and right camera in world coordinate is known as $T_{C_lW}$ and $T_{C_rW}$ respectively.

Then, consider a certain point in space $P$, which has a homogeneous coordinate $\boldsymbol {P}=(X,Y,Z,1)^T$. $P$ is projected to left camera normalized plane as $\boldsymbol x_l=(u_l,v_l,1)^T$ and projected to right camera normalized plane as $\boldsymbol x_r=(u_r,v_r,1)^T$.

In this problem, we know the left and right camera

 pose $\boldsymbol T_{C_lW}$, $\boldsymbol T_{C_rW}$, and the projected point on left and right image plane $\boldsymbol x_l$ and $\boldsymbol x_r$. We want a solution for the point $\boldsymbol P$ in space.

## Solution

First, we can have left camera as an example. We define a $3\times4$ matrix $\boldsymbol {T}_{C_lW}=[\boldsymbol{R}_{C_lW}|\boldsymbol{t}_{C_lW}]$ consisting of rotation and translation information. Then we have the following equation:
$$
R_{C_lW}
=
\left[
 \begin{matrix}
   t_1 & t_2 & t_3 \\
   t_5 & t_6 & t_7 \\
   t_9 & t_{10} & t_{11}
  \end{matrix} 
\right]
,
t_{C_lW}
=
\left[
 \begin{matrix}
   t_4 \\
   t_8 \\
   t_{12}
  \end{matrix} 
\right]
\\
$$

$$
s
\left[
 \begin{matrix}
   u_l \\
   v_l \\
   1
  \end{matrix} 
\right]
=
\left[
 \begin{matrix}
   t_1 & t_2 & t_3 & t_4 \\
   t_5 & t_6 & t_7 & t_8 \\
   t_9 & t_{10} & t_{11} & t_{12}
  \end{matrix} 
\right]
\left[
 \begin{matrix}
   X \\
   Y \\
   Z \\
   1
  \end{matrix} 
\right]
$$

We can know that
$$
s=t_9X+t_{10}Y+t_{11}Z+t_{12}
$$
Then we can eliminate $s$ from the equation, which yields
$$
u_l=\frac{t_1X+t_2Y+t_3Z+t_4}{t_9X+t_{10}Y+t_{11}Z+t_{12}}
$$

$$
v_l=\frac{t_5X+t_6Y+t_7Z+t_8}{t_9X+t_{10}Y+t_{11}Z+t_{12}}
$$

To simplify, we define row vectors of $T$:
$$
\boldsymbol t_{C_lW1}=[t_1,t_2,t_3,t_4]^T \\
\boldsymbol t_{C_lW2}=[t_5,t_6,t_7,t_8]^T \\
\boldsymbol t_{C_lW3}=[t_9,t_{10},t_{11},t_{12}]^T
$$
Then we have
$$
\boldsymbol{t_{C_lW1}}^T\boldsymbol{P}-\boldsymbol{t_{C_lW3}}^T\boldsymbol{P}u_l=0 \\
\boldsymbol{t_{C_lW2}}^T\boldsymbol{P}-\boldsymbol{t_{C_lW3}}^T\boldsymbol{P}v_l=0
$$
The same can be obtained from right camera
$$
\boldsymbol{t_{C_rW1}}^T\boldsymbol{P}-\boldsymbol{t_{C_rW3}}^T\boldsymbol{P}u_r=0 \\
\boldsymbol{t_{C_rW2}}^T\boldsymbol{P}-\boldsymbol{t_{C_rW3}}^T\boldsymbol{P}v_r=0
$$
Therefore, we can have the following linear equations
$$
\left[
 \begin{matrix}
   \boldsymbol{t_{C_lW1}}^T-\boldsymbol{t_{C_lW3}}^Tu_l \\
   \boldsymbol{t_{C_lW2}}^T-\boldsymbol{t_{C_lW3}}^Tv_l \\
   \boldsymbol{t_{C_rW1}}^T-\boldsymbol{t_{C_rW3}}^Tu_r \\
   \boldsymbol{t_{C_rW2}}^T-\boldsymbol{t_{C_rW3}}^Tv_r
  \end{matrix} 
\right]
\boldsymbol{P}
=0
$$
We can define $\boldsymbol{A}$ to be the following matrix
$$
\boldsymbol{A}=
\left[
 \begin{matrix}
   \boldsymbol{t_{C_lW1}}^T-\boldsymbol{t_{C_lW3}}^Tu_l \\
   \boldsymbol{t_{C_lW2}}^T-\boldsymbol{t_{C_lW3}}^Tv_l \\
   \boldsymbol{t_{C_rW1}}^T-\boldsymbol{t_{C_rW3}}^Tu_r \\
   \boldsymbol{t_{C_rW2}}^T-\boldsymbol{t_{C_rW3}}^Tv_r
  \end{matrix} 
\right]
$$


Now we need to solve this $\boldsymbol{Ax}=0$ problem to get a solution for $\boldsymbol{P}$, which is the eigenvector corresponding to the smallest eigenvalue of $\boldsymbol A^T \boldsymbol A$. 

With SVD decomposition, for any real $m\times n$ matrix $\boldsymbol{A}$, we have
$$
\boldsymbol{A=UDV}^T
$$
$\boldsymbol{U}$ is a $m\times n$ orthogonal matrix whose columns are eigenvectors of $\boldsymbol{AA}^T$.

$\boldsymbol{V}$ is a $n\times n$ orthogonal matrix whose columns are eigenvectors of $\boldsymbol{A}^T\boldsymbol{A}$.

$\boldsymbol{D}$ is a $n\times n$ diagonal matrix which has non-negative real values called singular values. $\boldsymbol{D}=diag(\sigma_1,\sigma_2,\cdots,\sigma_n)$ is ordered so that $\sigma_1\ge\sigma_2\ge\cdots\ge\sigma_n$.

Therefore, we can have the last column vector of $\boldsymbol V$ as the solution of $\boldsymbol P$.

## Reference

* http://www.cs.cmu.edu/~16385/s17/Slides/11.4_Triangulation.pdf
* https://www.ecse.rpi.edu/~qji/CV/svd_review.pdf
* https://www.cse.unr.edu/~bebis/MathMethods/SVD/lecture.pdf
* https://web.mit.edu/be.400/www/SVD/Singular_Value_Decomposition.htm

