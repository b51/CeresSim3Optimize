# CeresSim3Optimize

## What's this repository for
**This is a sim(3) poses optimize with ceres solver and display with qt**

## Preinstall
```bash
$ sudo apt-get install libqglviewer-dev-qt5 -y

$ git clone https://github.com/strasdat/Sophus
$ cd Sophus
$ mkdir build
$ cd build
$ cmake ..
$ sudo make install
```

## Run
```bash
$ git clone https://github.com/b51/CeresSim3Optimize.git
$ cd CeresSim3Optimize
$ mkdir build
$ cd build
$ cmake ..
$ make -j
$ ./CeresSim3Optimize
click initial and choose sim3_sphere_data.g2o in data folder, then optimize
```

## Error and Jacobian calculation of sim(3)
**!!! Chrome extension [TeX All the Things](https://chrome.google.com/webstore/detail/tex-all-the-things/cbimabofgmfdkicghcadidpemeenbffn) is neccessary for Latex below display !!!**

### Some neccessary equations before Jacobian calculation
**1. With the property of Lie Algebra Adjoint, [Reference](https://blog.csdn.net/heyijia0327/article/details/51773578)**
$$\mathbf{S * exp(\hat{\zeta}) * S^{-1} = exp[(Adj(S) * \zeta)^{\Lambda}]}$$
We can get equations below
$$\mathbf{S * exp(\hat{\zeta}) = exp[(Adj(S) * \zeta)^{\Lambda}] * S}$$
and
$$\mathbf{exp(\hat{\zeta}) * S^{-1} = S^{-1} * exp[(Adj(S) * \zeta)^{\Lambda}]}$$

**2. Baker-Campbell-Hausdorf equations, [STATE ESTIMATION FOR ROBOTICS](http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) P.234**

$\hspace{2cm}\mathbf{ln(S_1 S_2)^{v} = ln(exp[\hat{\xi_1]}exp[\hat{\xi_2]})^{v}}$

$\hspace{4cm}\mathbf{ = \xi_1 + \xi_2 + \frac{1}{2} \xi_1^{\lambda} \xi_2 + \frac{1}{12}\xi_1^{\lambda} \xi_1^{\lambda} \xi_2 + \frac{1}{12}\xi_2^{\lambda} \xi_2^{\lambda} \xi_1 + \dots}$

$\hspace{4cm}\mathbf{\approx J_l(\xi_2)^{-1}\xi_1 + \xi_2}\hspace{2cm}$ (if $\xi_1 small$)

$\hspace{4cm}\mathbf{\approx \xi_2 + J_r(\xi_1)^{-1}\xi_2}\hspace{2cm}$ (if $\xi_2 small$)

With
$\hspace{3cm}\mathbf{J_l(\xi)^{-1} = \sum_{n = 0}^{\infty} \frac{B_n}{n!} (\xi^{\lambda})^{n}}$

$\hspace{3cm}\mathbf{J_r(\xi)^{-1} = \sum_{n = 0}^{\infty} \frac{B_n}{n!} (-\xi^{\lambda})^{n}}$

With$\hspace{1cm}B_0 = 1, B_1 = -\frac{1}{2}, B_2 = \frac{1}{6}, B_3 = 0, B_4 = -\frac{1}{30}\dots$, $\hspace{5mm}\mathbf{\xi^{\lambda} = adj(\xi)}$, $\hspace{5mm}$ is adjoint matrix of $\xi$

**3. Adjoint Matrix of sim(3)**
a) Main property of adjoint matrix on Lie Algebras, [Reference: LIE GROUPS AND LIE ALGEBRAS, 1.6](http://www.math.jhu.edu/~fspinu/423/7.pdf)
$$\mathbf{[x, y] = adj(x)y}$$

b) sim3 Lie Brackets, [Reference: Local Accuracy and Global Consistency for Efficient Visual SLAM, P.184, A.3.4](https://www.doc.ic.ac.uk/~ajd/Publications/Strasdat-H-2012-PhD-Thesis.pdf):
$$\mathbf{[x, y] = [\begin{bmatrix} \nu \newline \omega \newline \sigma\end{bmatrix} \begin{bmatrix} \tau \newline \varphi \newline \varsigma \end{bmatrix}] = \begin{bmatrix}\omega \times \tau + \nu \times \varphi + \sigma\tau - \varsigma\nu \newline \omega \times \varphi \newline  0 \end{bmatrix}}$$
$$\mathbf{= \begin{bmatrix}(\hat{\omega} + \sigma I)\tau + \nu \times \varphi - \varsigma\nu \newline \omega \times \varphi \newline  0 \end{bmatrix}}$$
$$\hspace{-4cm} = \mathbf{adj(x) y}$$

We can get $\hspace{4cm}\mathbf{\xi^{\lambda} = adj(\xi) = \begin{bmatrix} (\hat{\omega} + \sigma I) & \hat{\nu} & -{\nu} \newline 0 & \hat{\varphi} & 0 \newline 0 & 0 & 0 \end{bmatrix}}$

### Left multiplication/Right multiplication for pose update
sim(3) update with **Left multiplication/Right multiplication** has affect on Jacobian calculation, Formula derivation below used Right multiplication as example

### Jacobian Calculation of sim(3)
$$\mathbf{error = S_{ji}  S_{iw}  S_{jw}^{-1}}$$

Derivation of $\mathbf{Jacobian_i}$

$\hspace{6cm}\mathbf{ln(error(\xi_i + \delta_i))^v = ln(S_{ji}S_{iw}exp(\hat{\delta_i})S_{jw}^{-1})^{v}}$

$\hspace{10cm}\mathbf{= ln(S_{ji}S_{iw}S_{jw}^{-1}exp[(Adj(S_{jw}){\delta_i})^{\Lambda}])^{v}}$

$\hspace{10cm}\mathbf{= ln(exp(\xi_{error}) \cdot exp[(Adj(S_{jw}){\delta_i})^{\Lambda}])^{v}}$

$\hspace{10cm}\mathbf{= \xi_{error} + J_r(\xi_{error})^{-1} Adj(S_{jw}){\delta_i}}$

$$\mathbf{Jacobian_i = \frac{\partial ln(error)^v}{\partial \delta_i} = \lim_{\delta_i \to 0} \frac{ln(error(\xi_i + \delta_i))^v - ln(error)^v}{\delta_i} = J_r(\xi_{error})^{-1} Adj(S_{jw}) }$$

Same to $\mathbf{Jacobian_j}$

$\hspace{6cm}\mathbf{ln(error(\xi_i + \delta_j)) = ln(S_{ji}S_{iw}(S_{jw}exp(\hat{\delta_j}))^{-1})^{v}}$
$\hspace{10cm}\mathbf{= ln(S_{ji}S_{iw}exp(-\hat{\delta_j})S_{jw}^{-1})^{v}}$

$\hspace{10cm}\mathbf{= ln(exp(\xi_{error}) \cdot exp[(Adj(S_{jw})(-\delta_j))^{\Lambda}])^{v}}$

$\hspace{10cm}\mathbf{= \xi_{error} - J_r(\xi_{error})^{-1} Adj(S_{jw}){\delta_j}}$

$$\mathbf{Jacobian_j = \frac{\partial ln(error)^v}{\partial \delta_j} = \lim_{\delta_j \to 0} \frac{ln(error(\xi_i + \delta_j))^v - ln(error)^v}{\delta_j} = -J_r(\xi_{error})^{-1} Adj(S_{jw}) }$$

### Coding with ceres solver
Code Reference: https://github.com/b51/CeresSim3Optimize

Define Plus method with **Right multiplication** in ceres Sim3LocalParameterization 

``` cpp
class CERES_EXPORT Sim3Parameterization : public ceres::LocalParameterization {
public:
    virtual ~Sim3Parameterization() {}
    virtual bool Plus(const double* x,
                      const double* delta,
                      double* x_plus_delta) const;
    virtual bool ComputeJacobian(const double* x,
                                 double* jacobian) const;
    virtual int GlobalSize() const { return 7; }
    virtual int LocalSize() const { return 7; }
};

/**
 *  该方法为定义的右乘更新，所以在计算 Jacobian 时也需要按照该方法进行更新
 *  若是左乘更新，则 x_plus_delta_lie = (delta_S * S).log();
 */
bool Sim3Parameterization::Plus(const double* x,
                                const double* delta,
                                double* x_plus_delta) const {
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> lie(x);
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> delta_lie(delta);
    Sophus::Sim3d S = Sophus::Sim3d::exp(lie);
    Sophus::Sim3d delta_S = Sophus::Sim3d::exp(delta_lie);
    Eigen::Matrix<double, 7, 1> x_plus_delta_lie(x_plus_delta);
    x_plus_delta_lie= (S * delta_S).log();
    return true;
}

/**
 *  该方法为 Jacobian 计算方法，因为在 Ceres Costfunction 中已经定义了 jacobian 的
 *  计算方法，所以这里设置为 Identity 矩阵就行
 */
bool Sim3Parameterization::ComputeJacobian(const double *x, double *jacobian) const {
    ceres::MatrixRef(jacobian, 7, 7) = ceres::Matrix::Identity(7, 7);
    return true;
}
```

### Reference:
1. https://www.encyclopediaofmath.org/index.php/Adjoint_representation_of_a_Lie_group
2. http://www.math.jhu.edu/~fspinu/423/7.pdf
3. [STATE ESTIMATION FOR ROBOTICS, P.234](http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf)
4. [Local Accuracy and Global Consistency for Efficient Visual SLAM, P.184, A.3.4](https://www.doc.ic.ac.uk/~ajd/Publications/Strasdat-H-2012-PhD-Thesis.pdf)
5. https://www.jianshu.com/p/efe0d10197ba
6. https://blog.csdn.net/heyijia0327/article/details/51773578
