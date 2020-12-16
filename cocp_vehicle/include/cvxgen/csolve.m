% csolve  Solves a custom quadratic program very rapidly.
%
% [vars, status] = csolve(params, settings)
%
% solves the convex optimization problem
%
%   minimize(lam_3*square(u(1)) + lam_4*square(u(2)) + quad_form(P_sqrt*y, eye(4)) + q'*y)
%   subject to
%     y == fx + B*u
%     -a_max <= u(1)
%     u(1) <= a_max
%     abs(u(2) + L*k) <= tan_d_max
%     abs(u(2) + L*k - z_prev - L*k_prev) <= dtan_max
%
% with variables
%        u   2 x 1
%        y   4 x 1
%
% and parameters
%        B   4 x 2
%        L   1 x 1
%   P_sqrt   4 x 4
%    a_max   1 x 1
% dtan_max   1 x 1
%       fx   4 x 1
%        k   1 x 1
%   k_prev   1 x 1
%    lam_3   1 x 1    positive
%    lam_4   1 x 1    positive
%        q   4 x 1
% tan_d_max   1 x 1
%   z_prev   1 x 1
%
% Note:
%   - Check status.converged, which will be 1 if optimization succeeded.
%   - You don't have to specify settings if you don't want to.
%   - To hide output, use settings.verbose = 0.
%   - To change iterations, use settings.max_iters = 20.
%   - You may wish to compare with cvxsolve to check the solver is correct.
%
% Specify params.B, ..., params.z_prev, then run
%   [vars, status] = csolve(params, settings)
% Produced by CVXGEN, 2020-09-17 15:09:17 -0400.
% CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2017 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: csolve.m.
% Description: Help file for the Matlab solver interface.
