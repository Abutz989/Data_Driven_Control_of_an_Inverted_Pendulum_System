# Data-Driven Control of an Inverted Pendulum System


Yizhak Abu, Tom Hirshberg and Alex M. Bronstein


  


This notebook present the proposed frame work of the paper.


  

   1.  Define the system  


<img src="https://latex.codecogs.com/gif.latex?\dot{x}&space;=\begin{array}{cccc}&space;\alpha^˙&space;\\&space;\frac{M\cdot&space;g\cdot&space;l}{I}\sin&space;(\alpha&space;)-\frac{c}{I}\alpha^˙&space;+\frac{1}{I}(\frac{k_t&space;\cdot&space;k_e&space;}{R}+f)\theta^˙&space;-\frac{k_t&space;}{R\cdot&space;I}V\\&space;-\alpha^¨&space;-\frac{k_t&space;(k_e&space;+f\cdot&space;R)}{R\cdot&space;I_{Wheel}&space;}\theta^˙&space;+\frac{k_t&space;}{R\cdot&space;I_{Wheel}&space;}V&space;\end{array}"/>



2.  Ganerate Data for the referance model



```matlab:Code
run DataGen.m
```



3. Train the NN-controller and save the best validation results.



```matlab:Code
run FindingNets.m
```



4. Stabilty Check




credit : https://github.com/heyinUCB/Stability-Analysis-using-Quadratic-Constraints-for-Systems-with-Neural-Network-Controllers



```matlab:Code
addpath('.\Inverted_Pendulum_control_saturation')
My_Pendulum_sin_local
```


```text:Output
P = 13x13    
1.0e+03 *

    0.0797    0.0116   -0.0050    0.0051    0.0017   -0.0020    0.0070    0.0016   -0.0012    0.0038    0.0009   -0.0006   -0.0003
    0.0116    0.0102   -0.0030   -0.0029   -0.0014   -0.0001   -0.0013   -0.0013   -0.0000   -0.0006   -0.0011    0.0002    0.0011
   -0.0050   -0.0030    0.0061    0.0022    0.0008   -0.0013    0.0011    0.0005   -0.0009    0.0004    0.0004   -0.0009    0.0000
    0.0051   -0.0029    0.0022    0.0487    0.0060   -0.0011    0.0264    0.0033   -0.0006    0.0262    0.0043   -0.0009    0.0000
    0.0017   -0.0014    0.0008    0.0060    0.0031   -0.0005    0.0015    0.0007   -0.0003    0.0020    0.0007   -0.0004    0.0000
   -0.0020   -0.0001   -0.0013   -0.0011   -0.0005    0.0015   -0.0001   -0.0002    0.0000   -0.0003   -0.0002    0.0001   -0.0000
    0.0070   -0.0013    0.0011    0.0264    0.0015   -0.0001    0.0490    0.0065   -0.0007    0.0248    0.0042   -0.0009    0.0000
    0.0016   -0.0013    0.0005    0.0033    0.0007   -0.0002    0.0065    0.0030   -0.0004    0.0017    0.0007   -0.0003   -0.0000
   -0.0012   -0.0000   -0.0009   -0.0006   -0.0003    0.0000   -0.0007   -0.0004    0.0013    0.0003   -0.0001   -0.0000   -0.0000
    0.0038   -0.0006    0.0004    0.0262    0.0020   -0.0003    0.0248    0.0017    0.0003    0.0457    0.0070   -0.0009    0.0000

traceP = 252.3150
```


![figure_0.png](README_images/figure_0.png)



The last state is in Lab, each controller should be tested on a real system for preformance evaluating.:


  

```matlab:Code
open Experimntal_Results_1.fig 
open DistrubExpAlph.fig
```


![figure_1.png](README_images/figure_1.png)


![figure_2.png](README_images/figure_2.png)


```matlab:Code
open DelayExpAlph.fig
```


![figure_3.png](README_images/figure_3.png)


```matlab:Code

```

