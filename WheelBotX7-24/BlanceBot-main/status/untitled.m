%非变量
syms Iw ll Rw mw ml lbl lwl Ill mb g;
syms lr lbr lwr Ilr ;
syms Ib lc Rl Iz ;
% % 参数变量
syms theta_wl_dot2 theta_ll_dot2 theta_ll Tbll Tlwl;
syms theta_wr_dot2 theta_lr_dot2 theta_lr Tblr Tlwr;
syms theta_b;
syms s s_dot;
%列方程m
eqns = [
   (Iw*ll/Rw+mw*Rw*ll+ml*Rw*lbl)*theta_wl_dot2+(ml*lwl*lbl-Ill)*theta_ll_dot2+(ml*lwl+0.5*mb*ll)*g*theta_ll+Tbll-Tlwl*(1+ll/Rw)==0,...
   (Iw*lr/Rw+mw*Rw*lr+ml*Rw*lbr)*theta_wr_dot2+(ml*lwr*lbr-Ilr)*theta_lr_dot2+(ml*lwr+0.5*mb*lr)*g*theta_lr+Tblr-Tlwr*(1+lr/Rw)==0,...
   -(mw*Rw*Rw+Iw+ml*Rw*Rw+0.5*mb*Rw*Rw)*theta_wl_dot2-(mw*Rw*Rw+Iw+ml*Rw*Rw+0.5*mb*Rw*Rw)*theta_wr_dot2-(ml*Rw*lwl+0.5*mb*Rw*ll)*theta_ll_dot2-(ml*Rw*lwr+0.5*mb*Rw*lr)*theta_lr_dot2+Tlwl+Tlwr ==0,...
   (mw*Rw*lc+Iw*lc/Rw+ml*Rw*lc)*theta_wl_dot2+(mw*Rw*lc+Iw*lc/Rw+ml*Rw*lc)*theta_wr_dot2+ml*lwl*lc*theta_ll_dot2+ml*lwr*lc*theta_lr_dot2-Ib*theta_lr_dot2+mb*g*lc*theta_b-(Tlwl+Tlwr)*lc/Rw-(Tbll+Tblr) == 0,...
   (0.5*Iz*Rw/Rl+Iw*Rl/Rw)*theta_wl_dot2-(0.5*Iz*Rw/Rl+Iw*Rl/Rw)*theta_wr_dot2+0.5*Iz*ll/Rl*theta_ll_dot2-0.5*Iz*lr/Rl*theta_lr_dot2-Tlwl*Rl/Rw+Tlwr*Rl/Rw == 0
];
% 确保 vars 是一个符号变量向量
vars = [theta_wl_dot2, theta_ll, theta_wr_dot2, theta_lr,theta_b];
disp(s)
% 求解方程
sol = solve(eqns, vars);
% 显示解
disp(sol);
% theta_wl_dot2=simplify(collect(sol.theta_wl_dot2));
% theta_ll=simplify(collect(sol.theta_ll));
% theta_wr_dot2=simplify(collect(sol.theta_wr_dot2));
% theta_lr=simplify(collect(sol.theta_lr));
% theta_b=simplify(collect(sol.theta_b));
% p_theta_wl_dot2 = subs(theta_wl_dot2,{theta_ll_dot2,theta_lr_dot2},{diff(theta_ll,2),diff(theta_lr,2)})