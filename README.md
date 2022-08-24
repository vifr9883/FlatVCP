# FlatVCP: Optimal Control for Kinematic Bicycle Model
This repository contains source code for the paper Optimal Control for Kinematic Bicycle Model with Continuous-time Safety Guarantees: A Sequential Second-order Cone Programming Approach.
The repository contains both Python3 and MATLAB code.

If you use this code, we would appreciate it if you cited as follows:
```
@article{freire2022optimal,
  title={Optimal Control for Kinematic Bicycle Model with Continuous-time Safety Guarantees: A
Sequential Second-order Cone Programming Approach},
  author={Freire, Victor and Xu, Xiangru},
  journal={arXiv preprint arXiv:2204.08980},
  year={2022}
}
```

## Python3
### Dependencies & Installation
The repository is structured as a Python3 package. Thus, you can run the following command to
install the dependencies.
```
pip3 install -r requirements.txt
```
For [cvxpy](https://www.cvxpy.org/), we recommend using [MOSEK](https://www.mosek.com/) noting
that you will need a license (free academic license available). We also tested
[ECOS](https://web.stanford.edu/~boyd/papers/ecos.html) with satisfactory performance.

### Example
For a quick-start, inspect the file `examples/fig_s.py` and run it with:
```
python3 examples/fig_s.py
```

The output should be a trajectory file `traj_figS.csv` with the generated state-space
trajectory and the following plot of the x-y trajectory.

<p align="center">
  <img
  src="https://github.com/ARC-Lab-Research-Group/FlatVCP/blob/master/img/bk_python_example.png"
  width="400" alt="Bicycle Python
  Example">
</p>
<!--(https://github.com/ARC-Lab-Research-Group/FlatVCP/blob/master/img/bk_python_example.png)
-->

Note that this example uses [matplotlib](https://matplotlib.org/) for visualization.


## MATLAB
### Dependencies & Installation
The MATLAB implementation requires a working installation of
[YALMIP](https://yalmip.github.io/) and we recommend using [MOSEK](https://www.mosek.com/).
noting that you will need a license (free academic license available). However, any
second-order cone programming solver should also work.

Also required is the
[B-splines](https://www.mathworks.com/matlabcentral/fileexchange/27374-b-splines) Add-On by
[Levente Hunyadi](https://www.mathworks.com/matlabcentral/profile/authors/1879353).

You might also find the following MATHWORKS toolboxes useful/necessary:
* Optimization Toolbox
* Symbolic Math Toolbox
* Control Systems Toolbox

The package is lightweight and there is no installation beyond adding the following folders to
your path:
```
addpath(userpath+"\FlatVCP\matlab\bspline")
addpath(userpath+"\FlatVCP\matlab\vcp_bk")
```
Then, you need to compile the SOCPs into YALMIP
[optimizer](https://yalmip.github.io/command/optimizer/) objects by running
`matlab/vcp_bk/vcp_bk_compile.m`. This will generate `vcp_bk_compiled.mat` which will later be
sourced to avoid re-compiling the SOCPs at each solve time.

### Example
For a quick-start, inspect the file `matlab/vcp_bk/vcp_bk_example.m` and run it.
The output should be the following plots.

<p float="left">
[<img
src="https://github.com/ARC-Lab-Research-Group/FlatVCP/blob/master/img/bk_matlab_example_path.png"
width="200" alt="Bicycle MATLAB
Example Path">](https://github.com/ARC-Lab-Research-Group/FlatVCP/blob/master/img/bk_matlab_example_path.png)
[<img
src="https://github.com/ARC-Lab-Research-Group/FlatVCP/blob/master/img/bk_matlab_example_state.png"
width="200" alt="Bicycle MATLAB
Example State">](https://github.com/ARC-Lab-Research-Group/FlatVCP/blob/master/img/bk_matlab_example_state.png)
[<img
src="https://github.com/ARC-Lab-Research-Group/FlatVCP/blob/master/img/bk_matlab_example_input.png"
width="200" alt="Bicycle MATLAB
Example Input">](https://github.com/ARC-Lab-Research-Group/FlatVCP/blob/master/img/bk_matlab_example_input.png)
</p>
