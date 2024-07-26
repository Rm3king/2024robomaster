% % 非变量
% syms Iw ll Rw mw ml lbl lwl Ill mb g;
% syms lr lbr lwr Ilr ;
% syms Ib lc Rl Iz ;
% % 参数变量
% % syms theta_wl_dot2 theta_ll_dot2 theta_ll Tbll Tlwl;
% % syms theta_wr_dot2 theta_lr_dot2 theta_lr Tblr Tlwr;
% % syms theta_b;
% syms theta_wl(t) theta_ll(t) theta_wr(t) theta_lr(t) theta_b(t);
% syms Tbll(t) Tlwl(t) Tblr(t) Tlwr(t);
% 
% % 列方程
% eqns = [
%    (Iw*ll/Rw + mw*Rw*ll + ml*Rw*lbl)*diff(theta_wl, t, 2) + (ml*lwl*lbl - Ill)*diff(theta_ll, t, 2) + (ml*lwl + 0.5*mb*ll)*g*theta_ll(t) + Tbll(t) - Tlwl(t)*(1 + ll/Rw) == 0,...
%    (Iw*lr/Rw + mw*Rw*lr + ml*Rw*lbr)*diff(theta_wr, t, 2) + (ml*lwr*lbr - Ilr)*diff(theta_lr, t, 2) + (ml*lwr + 0.5*mb*lr)*g*theta_lr(t) + Tblr(t) - Tlwr*(1 + lr/Rw) == 0,...
%    -(mw*Rw*Rw + Iw + ml*Rw*Rw + 0.5*mb*Rw*Rw)*diff(theta_wl, t, 2) - (mw*Rw*Rw + Iw + ml*Rw*Rw + 0.5*mb*Rw*Rw)*diff(theta_wr, t, 2) - (ml*Rw*lwl + 0.5*mb*Rw*ll)*diff(theta_ll, t, 2) - (ml*Rw*lwr + 0.5*mb*Rw*lr)*diff(theta_lr, t, 2) + Tlwl(t) + Tlwr(t) == 0,...
%    (mw*Rw*lc + Iw*lc/Rw + ml*Rw*lc)*diff(theta_wl, t, 2) + (mw*Rw*lc + Iw*lc/Rw + ml*Rw*lc)*diff(theta_wr, t, 2) + ml*lwl*lc*diff(theta_ll, t, 2) + ml*lwr*lc*diff(theta_lr, t, 2) - Ib*diff(theta_lr, t, 2) + mb*g*lc*theta_b(t) - (Tlwl(t) + Tlwr(t))*lc/Rw - (Tbll(t) + Tblr(t)) == 0,...
%    (0.5*Iz*Rw/Rl + Iw*Rl/Rw)*diff(theta_wl, t, 2) - (0.5*Iz*Rw/Rl + Iw*Rl/Rw)*diff(theta_wr, t, 2) + 0.5*Iz*ll/Rl*diff(theta_ll, t, 2) - 0.5*Iz*lr/Rl*diff(theta_lr, t, 2) - Tlwl(t)*Rl/Rw + Tlwr(t)*Rl/Rw == 0
% ];
% 
% % 确保 vars 是一个符号变量向量
% vars = [theta_wl, theta_ll, theta_wr , theta_lr, theta_b];
% % 求解方程
% sol = solve(eqns, vars);
% 
% % 显示解
% % disp(sol);
% 定义符号变量
% 定义符号变量
syms Iw ll Rw mw ml lbl lwl Ill mb g lb lc Rl Iz Rb
syms theta_wl(t) theta_ll(t) theta_wr(t) theta_lr(t) theta_b(t)
syms Tbll(t) Tlwl(t) Tblr(t) Tlwr(t)
syms theta_wl_dot theta_ll_dot theta_wr_dot theta_lr_dot theta_b_dot

% 定义状态变量和控制输入
x = [theta_wl, diff(theta_wl, t), theta_ll, diff(theta_ll, t), ...
     theta_wr, diff(theta_wr, t), theta_lr, diff(theta_lr, t), theta_b, diff(theta_b, t)];

u = [Tbll, Tlwl, Tblr, Tlwr];

% 动力学方程
eqns = [
   (Iw*ll/Rw+mw*Rw*ll+ml*Rw*lbl)*theta_wl_dot2+(ml*lwl*lbl-Ill)*theta_ll_dot2+(ml*lwl+0.5*mb*ll)*g*theta_ll+Tbll-Tlwl*(1+ll/Rw)==0,...
   (Iw*lr/Rw+mw*Rw*lr+ml*Rw*lbr)*theta_wr_dot2+(ml*lwr*lbr-Ilr)*theta_lr_dot2+(ml*lwr+0.5*mb*lr)*g*theta_lr+Tblr-Tlwr*(1+lr/Rw)==0,...
   -(mw*Rw*Rw+Iw+ml*Rw*Rw+0.5*mb*Rw*Rw)*theta_wl_dot2-(mw*Rw*Rw+Iw+ml*Rw*Rw+0.5*mb*Rw*Rw)*theta_wr_dot2-(ml*Rw*lwl+0.5*mb*Rw*ll)*theta_ll_dot2-(ml*Rw*lwr+0.5*mb*Rw*lr)*theta_lr_dot2+Tlwl+Tlwr ==0,...
   (mw*Rw*lc+Iw*lc/Rw+ml*Rw*lc)*theta_wl_dot2+(mw*Rw*lc+Iw*lc/Rw+ml*Rw*lc)*theta_wr_dot2+ml*lwl*lc*theta_ll_dot2+ml*lwr*lc*theta_lr_dot2-Ib*theta_lr_dot2+mb*g*lc*theta_b-(Tlwl+Tlwr)*lc/Rw-(Tbll+Tblr) == 0,...
   (0.5*Iz*Rw/Rl+Iw*Rl/Rw)*theta_wl_dot2-(0.5*Iz*Rw/Rl+Iw*Rl/Rw)*theta_wr_dot2+0.5*Iz*ll/Rl*theta_ll_dot2-0.5*Iz*lr/Rl*theta_lr_dot2-Tlwl*Rl/Rw+Tlwr*Rl/Rw == 0
];

% 状态变量及其导数
state_vars = [theta_wl, theta_ll, theta_wr, theta_lr, theta_b, ...
              theta_wl_dot, theta_ll_dot, theta_wr_dot, theta_lr_dot, theta_b_dot];

% 初始化状态导数向量
state_dot = zeros(10, 1);

% 替换动力学方程中的二阶导数为0，以便计算雅可比矩阵
for i = 1:length(eqns)
    eqn_i = eqns(i);
    for j = 1:length(state_vars)
        deriv = diff(state_vars(j), t, 2);
        if deriv ~= 0
            eqn_i = subs(eqn_i, deriv, 0);
        end
    end
    state_dot(i) = eqn_i;
end

% 计算雅可比矩阵A（状态变量的导数相对于状态变量本身）
A = jacobian(state_dot, state_vars);

% 定义控制输入
control_inputs = [Tbll, Tlwl, Tblr, Tlwr];

% 计算输入矩阵B（状态变量的导数相对于控制输入）
for i = 1:length(control_inputs)
    B(:, i) = jacobian(state_dot, control_inputs(i));
end

% 显示结果
disp("状态矩阵 A:");
disp(A);
disp("输入矩阵 B:");
disp(B);
