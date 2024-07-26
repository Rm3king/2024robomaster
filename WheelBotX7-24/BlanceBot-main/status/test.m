% 定义符号变量
syms Iw Rl Tlwl Tbll Iz Rw mb ml mw Ill theta_ll_dot2 theta_lr_dot2 g ll lr lbl lwl lwr theta_ll Tlwr theta_lr

% 定义函数 theta_wl_dot2
theta_wl_dot2 = -(Rl^2*Rw^2*Tlwr*mb - Iz*Rw^2*Tlwl - Iz*Rw^2*Tlwr - Rl^2*Rw^2*Tlwl*mb - 4*Iw*Rl^2*Tlwl - 2*Rl^2*Rw^2*Tlwl*ml + 2*Rl^2*Rw^2*Tlwr*ml - 2*Rl^2*Rw^2*Tlwl*mw + 2*Rl^2*Rw^2*Tlwr*mw + Iw*Iz*Rw*ll*theta_ll_dot2 - Iw*Iz*Rw*lr*theta_lr_dot2 + Iz*Rw^3*ll*mb*theta_ll_dot2 + Iz*Rw^3*ll*ml*theta_ll_dot2 - Iz*Rw^3*lr*ml*theta_lr_dot2 + Iz*Rw^3*ll*mw*theta_ll_dot2 + Iz*Rw^3*lwl*ml*theta_ll_dot2 + Iz*Rw^3*lwr*ml*theta_lr_dot2 - Iz*Rw^3*lr*mw*theta_lr_dot2 + Iw*Rl^2*Rw*ll*mb*theta_ll_dot2 + Iw*Rl^2*Rw*lr*mb*theta_lr_dot2 + 2*Iw*Rl^2*Rw*lwl*ml*theta_ll_dot2 + 2*Iw*Rl^2*Rw*lwr*ml*theta_lr_dot2)/((2*Iw*Rl^2 + Iz*Rw^2)*(2*Iw + Rw^2*mb + 2*Rw^2*ml + 2*Rw^2*mw))-(Rl^2*Rw^2*Tlwl*mb - Iz*Rw^2*Tlwl - Iz*Rw^2*Tlwr - 4*Iw*Rl^2*Tlwr - Rl^2*Rw^2*Tlwr*mb + 2*Rl^2*Rw^2*Tlwl*ml - 2*Rl^2*Rw^2*Tlwr*ml + 2*Rl^2*Rw^2*Tlwl*mw - 2*Rl^2*Rw^2*Tlwr*mw - Iw*Iz*Rw*ll*theta_ll_dot2 + Iw*Iz*Rw*lr*theta_lr_dot2 + Iz*Rw^3*lr*mb*theta_lr_dot2 - Iz*Rw^3*ll*ml*theta_ll_dot2 + Iz*Rw^3*lr*ml*theta_lr_dot2 - Iz*Rw^3*ll*mw*theta_ll_dot2 + Iz*Rw^3*lwl*ml*theta_ll_dot2 + Iz*Rw^3*lwr*ml*theta_lr_dot2 + Iz*Rw^3*lr*mw*theta_lr_dot2 + Iw*Rl^2*Rw*ll*mb*theta_ll_dot2 + Iw*Rl^2*Rw*lr*mb*theta_lr_dot2 + 2*Iw*Rl^2*Rw*lwl*ml*theta_ll_dot2 + 2*Iw*Rl^2*Rw*lwr*ml*theta_lr_dot2)/((2*Iw*Rl^2 + Iz*Rw^2)*(2*Iw + Rw^2*mb + 2*Rw^2*ml + 2*Rw^2*mw));

% 定义 theta_ll_dot2 作为 theta_ll 的二阶导数
% theta_ll_dot2 = diff(theta_ll, 2);
theta_wl_dot2=subs(theta_wl_dot2,{theta_ll_dot2,theta_lr_dot2},{diff(theta_ll, 2),diff(theta_lr, 2)});
% 计算 theta_wl_dot2 对 theta_ll 的偏导数
partial_derivative = diff(theta_wl_dot2, theta_ll);

% 显示结果
disp(partial_derivative);
