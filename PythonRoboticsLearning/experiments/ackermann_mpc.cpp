#include <ackermann_mpc/ackermann_mpc.h>

using namespace ackermann_mpc;

void AckermannMPC::setup_problem() {
        using namespace casadi;
        
        // 定义状态变量                                  运动学方程:
        MX x = MX::sym("x");                                 /* dx = v*cos(theta) */
        MX y = MX::sym("y");                                 /* dy = v*cos(theta) */
        MX theta = MX::sym("theta");                             /* dtheta = v*tan(delta) */
        MX delta = MX::sym("delta");                         /* ddelta = -delta/tao + segma/tao */ //一阶延迟增广,取消纯延迟
        MX state = MX::vertcat({x, y, theta, delta});               
        
        // 定义控制输入，速度和角速度                        控制量: [v, segma]
        MX v = MX::sym("v");                             
        MX segma = MX::sym("segma");
        MX controls = MX::vertcat({v, segma});
        
        // 系统运动学模型
        MX rhs = MX::vertcat({v*cos(theta), v*sin(theta), v*tan(delta), -delta/config_.tao + segma/config_.tao});
        Function f = Function("f", {state, controls}, {rhs});
        
        // 定义MPC优化问题
        MX obj = 0; // 目标函数
        MX g = MX::sym("g", 0, 1); // 等式约束

        MX X = MX::sym("X", dimension, config_.N+1);
        MX U = MX::sym("U", 2, config_.N);
        
        MX X0 = MX::sym("X0", dimension);
        MX Xref = MX::sym("Xref", dimension, config_.N+1);
        
        g = MX::vertcat({g, X(Slice(), 0) - X0});
        
        for (int k = 0; k < config_.N; ++k) {
            MX st = X(Slice(), k);
            MX st_next = X(Slice(), k+1);
            MX con = U(Slice(), k);
            MX ref = Xref(Slice(), k);
            
            // 使用RK4方法计算下一个状态
            MX k1 = f(std::vector<MX>{st, con})[0];
            MX k2 = f(std::vector<MX>{st + config_.dt/2 * k1, con})[0];
            MX k3 = f(std::vector<MX>{st + config_.dt/2 * k2, con})[0];
            MX k4 = f(std::vector<MX>{st + config_.dt * k3, con})[0];
            MX st_next_rk4 = st + config_.dt/6 * (k1 + 2*k2 + 2*k3 + k4);
            
             // 添加连续性约束
            g = MX::vertcat({g, st_next(Slice(0,2)) - st_next_rk4(Slice(0,2))});

            // 角度连续性约束，考虑周期性
            MX angle_diff = st_next(2) - st_next_rk4(2);
            MX angle_diff_wrapped = atan2(sin(angle_diff), cos(angle_diff));
            g = MX::vertcat({g, angle_diff_wrapped});
            // +
            g = MX::vertcat({g, st_next(3) - st_next_rk4(3)});
            
            // 目标函数
            // 用2次方误差来衡量状态和参考状态之间的差异
            obj += config_.Q_x * pow(st_next(0) - ref(0), 2);
            obj += config_.Q_y * pow(st_next(1) - ref(1), 2);
            // 使用周期性角度差异
            MX theta_diff = st_next(2) - ref(2);
            MX theta_diff_wrapped = atan2(sin(theta_diff), cos(theta_diff));
            obj += config_.Q_theta * pow(theta_diff_wrapped, 2);
            // +
            // obj += config_.Q_y * pow(st_next(3) - ref(3), 2);

            // 控制量
            obj += config_.R_v * pow(con(0), 2);
            obj += config_.R_segma * pow(con(1), 2);
            
            // 约束速度平滑
            obj += config_.W_vel_diff * pow(con(0) - U(Slice(), k-1)(0), 2);
            obj += config_.W_segma_diff * pow(con(1) - U(Slice(), k-1)(1), 2);

            // terminal cost
            if (k == config_.N-1) {
                obj += config_.W_terminal_x * pow(st_next(0) - ref(0), 2);
                obj += config_.W_terminal_y * pow(st_next(1) - ref(1), 2);
                MX terminal_theta_diff = st_next(2) - ref(2);
                MX terminal_theta_diff_wrapped = atan2(sin(terminal_theta_diff), cos(terminal_theta_diff));
                obj += config_.W_terminal_theta * pow(terminal_theta_diff_wrapped, 2);
                // +
                //obj += config_.W_terminal_y * pow(st_next(3) - ref(3), 2);
            }
        }
        
        // 创建优化问题
        MXDict nlp = {{"x", MX::vertcat({MX::reshape(X, dimension*(config_.N+1), 1), MX::reshape(U, 2*config_.N, 1)})},
                      {"f", obj},
                      {"g", g},
                      {"p", MX::vertcat({X0, MX::reshape(Xref, dimension*(config_.N+1), 1)})}};
        
        // 创建求解器
        Dict opts;
        opts["ipopt.print_level"] = 0;
        opts["print_time"] = 0;
        solver = nlpsol("solver", "ipopt", nlp, opts);
    }
    
std::vector<double> 
AckermannMPC::solve(const std::vector<double>& initial_state,
                          const std::vector<std::vector<double>>& reference_trajectory) {
    using namespace casadi;
    
    std::vector<double> X0 = initial_state;
    std::vector<double> Xref;
    for (const auto& ref : reference_trajectory) {
        Xref.insert(Xref.end(), ref.begin(), ref.end());
    }
    
    // 设置约束
    std::vector<double> lbg(dimension * (config_.N + 1), 0);
    std::vector<double> ubg(dimension * (config_.N + 1), 0);
    
    // 设置变量边界
    std::vector<double> lbx(dimension * (config_.N + 1) + 2 * config_.N, -inf);
    std::vector<double> ubx(dimension * (config_.N + 1) + 2 * config_.N, inf);

    // 前轮转角约束
    for (int i = 0; i < config_.N+1; ++i) {
        lbx[dimension * i + 3] = -config_.max_delta;
        ubx[dimension * i + 3] = config_.max_delta;
    }
    
    // 控制约束
    for (int i = 0; i < config_.N; ++i) {
        lbx[dimension * (config_.N + 1) + 2 * i] = -config_.max_v;  
        ubx[dimension * (config_.N + 1) + 2 * i] = config_.max_v;
        lbx[dimension * (config_.N + 1) + 2 * i + 1] = -config_.max_segma;
        ubx[dimension * (config_.N + 1) + 2 * i + 1] = config_.max_segma;
    }
    
    // 初始猜测
    std::vector<double> x0(dimension * (config_.N + 1) + 2 * config_.N, 0);
    
    // 求解
    DMDict arg = {{"lbx", lbx}, {"ubx", ubx}, {"lbg", lbg}, {"ubg", ubg}, {"x0", x0}, {"p", DM::vertcat({DM(X0), DM(Xref)})}};
    DMDict res = solver(arg);
    
    std::vector<double> result = static_cast<std::vector<double>>(res.at("x"));
    std::vector<double> optimal_controls(result.begin() + dimension * (config_.N + 1), result.end());
    std::vector<double> optimal_states(result.begin(), result.begin() + dimension * (config_.N + 1));
    
    return optimal_controls;
}
