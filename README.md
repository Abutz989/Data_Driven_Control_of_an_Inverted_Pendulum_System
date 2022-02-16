# Data-Driven Control of an Inverted Pendulum System
This is the repository of the paper [ Data-Driven Control of an Inverted Pendulum System](https://www.overleaf.com/project/5fdf0263d2991b9894727c59). 

Yizhak Abu, Tom Hirshberg and Alex M. Bronstein

This notebook present the proposed framework of the paper.

## Define the system  

![RealVsSchame.png](README_images/RealVsSchame.png)
![equation.png](README_images/equation.png)

## Ganerate Data for the referance model
This script Builds an iLQR controller and simulates its input and output in various initial conditions.
![DataScheme.png](README_images/DataScheme.png)
```matlab:Code
run DataGen.m
```

## Train the NN-controller and save the best validation results.

```matlab:Code
run FindingNets.m
```

## Stabilty Check
credit : https://github.com/heyinUCB/Stability-Analysis-using-Quadratic-Constraints-for-Systems-with-Neural-Network-Controllers
### Prerequisites
* [CVX](http://cvxr.com/cvx/): Matlab software for convex programming
* [SOSOPT](https://dept.aem.umn.edu/~AerospaceControl/): General SOS optimization utility
* [Multipoly](https://dept.aem.umn.edu/~AerospaceControl/): Package used to represent multivariate polynomials

```matlab:Code
addpath('.\Inverted_Pendulum_control_saturation')
My_Pendulum_sin_local
```
```text:Output
traceP = 252.3150
```
![figure_0.png](README_images/figure_0.png)

## Experimental system validation
The last state is in Lab, each controller should be tested on a real system for preformance evaluating.:

Response to Unstable Initial Condition , compare to LQR controller:
![Stab.gif](README_images/Stab.gif)
![figure_1.png](README_images/figure_1.png)


Robustness to Disturbances:
![Disturbed.gif](README_images/Disturbed.gif)
![figure_2.png](README_images/figure_2.png)


Robustness to delays:
![figure_3.png](README_images/figure_3.png)


## License

This project is released under the MIT License. Please review the [License file](LICENSE) for more details.
