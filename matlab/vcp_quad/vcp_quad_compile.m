function vcp_quad_compile()
%VCP_QUAD_COMPILE  Compile YALMIP optimizer objects, save as mat file.
%
%   VCP_QUAD_COMPILE() creates a file called 'vcp_quad_compiled.mat' in 
%          current directory. The file contains a struct with the
%          compiled SOCP program: QUAD-SOCP.
%
%   Copyright (c) 2022, University of Wisconsin-Madison

yalmip('clear')
socp = vcp_quad_socp();
save("vcp_quad_compiled.mat","socp");