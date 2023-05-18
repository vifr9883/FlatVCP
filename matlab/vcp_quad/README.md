# FlatVCP: Optimal Control for Quadcopters

This folder contains MATLAB code for the trajectory optimization approach in the paper
[Flatness-Based Quadcopter Trajectory Planning and Tracking With Continuous-Time Safety
Guarantees](https://ieeexplore.ieee.org/document/10068121).

If you use this code, we would appreciate it if you cited the paper as follows:
```
@article{freire2021flatness,
  author={Freire, Victor and Xu, Xiangru},
  journal={IEEE Trans. Control Syst. Technol.},
  title={Flatness-Based Quadcopter Trajectory Planning and Tracking With
Continuous-Time Safety Guarantees},
  year={early access. 2023},
  volume={},
  number={},
  pages={1-16},
  doi={10.1109/TCST.2023.3250954}
}
```

For more information about our work, please visit [ARC Lab@UW-Madison](https://xu.me.wisc.edu/).

## Python3
Unfortunately, there is no Python3 code available at this time for the quadcopter system.

## MATLAB
### Dependencies & Installation
The code has the same dependencies as specified in the [top-level
README](https://github.com/vifremel/FlatVCP). You should add the following folder to your
path:
```
addpath(userpath+"\FlatVCP\matlab\vcp_quad")
```

Then, you need to compile FLAT-SOCP into YALMIP
[optimizer](https://yalmip.github.io/command/optimizer/) object by running
`matlab/vcp_quad/vcp_quad_compile.m`. This will generate `vcp_quad_compiled.mat` which will later be
sourced to avoid re-compiling the SOCP at each solve time.

### Example
For a quick-start, inspect the file `matlab/vcp_quad/vcp_quad_example.m` and run it.
You should see the following plots.


<p align="center">
  <img src="https://github.com/vifremel/FlatVCP/blob/master/matlab/vcp_quad/img/quad_path.png" width="400" alt="Quadcopter example: Path">
  <img src="https://github.com/vifremel/FlatVCP/blob/master/matlab/vcp_quad/img/quad_r.png" width="400" alt="Quadcopter example: Position">
</p>
<p align="center">
  <img src="https://github.com/vifremel/FlatVCP/blob/master/matlab/vcp_quad/img/quad_xi.png" width="400" alt="Quadcopter example: Angles">
  <img src="https://github.com/vifremel/FlatVCP/blob/master/matlab/vcp_quad/img/quad_u.png" width="400" alt="Quadcopter example: Inputs">
</p>
<p align="center">
  <img src="https://github.com/vifremel/FlatVCP/blob/master/matlab/vcp_quad/img/quad_dr.png" width="400" alt="Quadcopter example: Velocity">
  <img src="https://github.com/vifremel/FlatVCP/blob/master/matlab/vcp_quad/img/quad_v.png" width="400" alt="Quadcopter example: Speed">
</p>

## Acknowledgements
This work was supported by the University of Wisconsin-Madison
