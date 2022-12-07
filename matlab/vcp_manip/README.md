# FlatVCP: Optimal Control for a Planar Manipulator with n Links

This folder contains source code written for a class project in which the FlatVCP approach is
extended to the planar manipulator with n links system. See the project's [final
report](https://github.com/vifremel/FlatVCP/blob/master/matlab/vcp_manip/final_report.pdf)
for additional details.


## Python3
Unfortunately, there is no Python3 code available at this time for the manipulator system.


## MATLAB
### Dependencies & Installation
The code has the same dependencies as specified in the [top-level
README](https://github.com/vifremel/FlatVCP). You should add the following folder to your
path:
```
addpath(userpath+"\FlatVCP\matlab\vcp_manip")
```

For this class project, we didn't make use of YALMIP's
[optimizer](https://yalmip.github.io/command/optimizer/) objects.


### Example
For a quick-start, inspect the files `matlab/vcp_manip/vcp_manip_example.m` and
`matlab/vcp_manip/vcp_manip_example2.m` and run them. You should see the following plots.


<p align="center">
  <img src="https://github.com/vifremel/FlatVCP/blob/master/matlab/vcp_manip/img/manip4_1.png" width="400" alt="Manipulator with 4 links example 1">
  <img src="https://github.com/vifremel/FlatVCP/blob/master/matlab/vcp_manip/img/manip4_2.png" width="400" alt="Manipulator with 4 links example 2">
</p>
<p align="center">
  <img src="https://github.com/vifremel/FlatVCP/blob/master/matlab/vcp_manip/img/manip8_1.png" width="400" alt="Manipulator with 8 links example 1">
  <img src="https://github.com/vifremel/FlatVCP/blob/master/matlab/vcp_manip/img/manip8_2.png" width="400" alt="Manipulator with 8 links example 2">
</p>

## Acknowledgements
This work was supported by the University of Colorado Boulder.
