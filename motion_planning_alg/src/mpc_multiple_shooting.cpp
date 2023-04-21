#include <casadi/casadi.hpp>
#include <cnpy.h>

using namespace casadi;
using namespace std;


tuple<double, SX, SX> shift(double T, double t0, SX x0, SX u, Function f){
    SX st = x0;
    SX con = u(Slice(0), Slice(0, u.size2())).T();
    SXDict f_in = {{"x", st}, {"u", con}};
    SXDict f_value = f(f_in);
    st = st + T*f_value["rhs"];
    x0 = st;
    t0 = t0 + T;
    SX u_rest = u(Slice(1, u.size1()), Slice(0, u.size2()));
    SX u_last = u(Slice(u.size1()-1, u.size1()), Slice(0, u.size2()));
    SX u0 = vertcat(u_rest, u_last);
    tuple<double, SX, SX> next_state(t0, x0, u0);
    return next_state;
}

void save_dm(DM trees, std::string file_name, int index){
       std::vector<double> edges; 
       int count = 0;
       for(int k=0; k< trees.size1(); k++){
         for(int j=0; j< trees.size2(); j++){
            edges.push_back((double)trees(k,j));
         }
       }
       cnpy::npy_save(file_name, &edges[0],{(unsigned int)trees.size1(), (unsigned int)trees.size2()/index, (unsigned int)index},"w");
}

void save_double_vector(vector<vector<double>> trees, std::string file_name, int index){
       std::vector<double> edges; 
       int count = 0;
       for(int k=0; k< trees.size(); k++){
         for(int j=0; j< trees[0].size(); j++){
            edges.push_back(trees[k][j]);
         }
       }
       cnpy::npy_save(file_name, &edges[0],{(unsigned int)trees.size(), (unsigned int)trees[0].size()/index, (unsigned int)index},"w");
}


void save_vector(vector<double> trees, std::string file_name, int index){
       std::vector<double> edges; 
       int count = 0;
       for(int k=0; k< trees.size(); k++){
            edges.push_back(trees[k]);
       }
       cnpy::npy_save(file_name, &edges[0],{(unsigned int)1, (unsigned int)trees.size(), (unsigned int)index},"w");
}
    

int main(){

    SX ff = SX::sym("U", 4, 1);
    ff(0) = 0.005;
    ff(1) = 0.005;
    ff(2) = 0.005;
    ff(3) = (pi/180)*2.0;
    ff = SX::diag(ff);

    double T = 0.2;
    int N = 20;
    double robot_diam = 0.3;
    double v_max = 0.2;
    double v_min = -v_max;
    double omega_max = pi/4;
    double omega_min = -omega_max;
    vector<double> map_dim = {-6, 6, -6, 6, 0, 6};

    DM xs = DM({5.8, 5, 5.0, 0.0});
    vector<vector<double>> obs_map = {{-2.5, 1.5, 2, 2},{3.5, 5, 3, 2}};
    int obs_length = obs_map.size();

    SX x = SX::sym("x");
    SX y = SX::sym("y");
    SX z = SX::sym("z");
    SX theta = SX::sym("theta");
    SX states = vertcat(x, y, z, theta);
    int n_status = states.size1();

    SX v_x = SX::sym("v_x");
    SX v_y = SX::sym("v_y");
    SX v_z = SX::sym("v_z");
    SX omega = SX::sym("omega");
    SX controls = vertcat(v_x, v_y, v_z, omega);
    int n_controls = controls.size1();

    SX rhs = vertcat(v_x*cos(theta)-v_y*sin(theta), v_y*cos(theta) + v_x*sin(theta), v_z, omega);
    Function f("f", {states, controls}, {rhs}, {"x", "u"}, {"rhs"});

    SX U = SX::sym("U", n_controls, N);
    SX P = SX::sym("P", n_status + n_status);
    SX X = SX::sym("X", n_status, N+1);

    SX Q = DM::zeros(4,4);
    Q(0,0) = 1;
    Q(1,1) = 5;
    Q(2,2) = 0.5;
    Q(3,3) = 0.1;
    SX R = DM::zeros(4,4);
    R(0,0) = 0.5;
    R(1,1) = 0.5;
    R(2,2) = 0.5;
    R(3,3) = 0.05;

    SX obj = 0;
    SX g = SX::sym("g", N+1, n_status);
    SX st = X(Slice(0, X.size1()), Slice(0));

    g(Slice(0), Slice(0, g.size2())) = st - P(Slice(0, n_status));
    int ibj = 1;
    SX con = 0;

    for(int k=0; k<N; k++){
        st = X(Slice(0, X.size1()), Slice(k));
        con = U(Slice(0, U.size1()), Slice(k));
        obj = obj + mtimes((st-P(Slice(n_status,n_status*2))).T(), mtimes(Q,(st-P(Slice(n_status,n_status*2))))) + mtimes(con.T(), mtimes(R, con));
        SX st_next = X(Slice(0, X.size1()), Slice(k+1));
        SXDict f_in = {{"x", st}, {"u", con}};
        SXDict f_value = f(f_in);
        SX st_next_euler = st + T*f_value["rhs"];
        g(Slice(ibj), Slice(0, g.size2())) = st_next - st_next_euler;
        ibj += 1;
    }

    int obs_count = 0;
    SX obs_g = SX::sym("obs_g",(N+1)*obs_length, 1);
    for(int k=0; k< N+1; k++){
        st = X(Slice(0, X.size1()), Slice(k));
        for(auto obs : obs_map){
            obs_g(obs_count) = -sqrt(pow((st(0)-obs[0]),2) + pow((st(1)-obs[1]),2) + pow((st(2)-obs[2]),2)) + (obs[3] + robot_diam);
            obs_count += 1;
        }
    }

    g = reshape(g, n_status*(N+1), 1);
    g = vertcat(g, obs_g);
    SX OPT_variables = vertcat(reshape(X, n_status*(N+1), 1), reshape(U, n_controls*N, 1));


    // Set options
    Dict opts;
    opts["ipopt.tol"] = 1e-4;
    opts["ipopt.max_iter"] = 100;
    opts["expand"] = true;
    opts["ipopt.print_level"] = 0;
    opts["print_time"] = 0;
    opts["ipopt.acceptable_tol"] = 1e-8;

    SXDict nlp_prob = {{"f",obj}, {"x",OPT_variables}, {"p", P}, {"g",g}};
    Function solver = nlpsol("nlpsol", "ipopt", nlp_prob, opts);

    DM lbg = DM(1, n_status*(N+1) + (N+1)*obs_length);
    DM ubg = DM(1, n_status*(N+1) + (N+1)*obs_length);

    lbg(Slice(0), Slice(0, n_status*(N+1))) = 0;
    ubg(Slice(0), Slice(0, n_status*(N+1))) = 0;

    lbg(Slice(0), Slice(n_status*(N+1), n_status*(N+1) + (N+1)*obs_length)) = -inf;
    ubg(Slice(0), Slice(n_status*(N+1), n_status*(N+1) + (N+1)*obs_length)) = 0;

    DM lbx = DM(n_status*(N+1)+n_controls*N, 1);
    DM ubx = DM(n_status*(N+1)+n_controls*N, 1);

    lbx(Slice(0, n_status*(N+1), n_status), Slice(0)) = map_dim[0];
    ubx(Slice(0, n_status*(N+1), n_status), Slice(0)) = map_dim[1];
    lbx(Slice(1, n_status*(N+1), n_status), Slice(0)) = map_dim[2];
    ubx(Slice(1, n_status*(N+1), n_status), Slice(0)) = map_dim[3];
    lbx(Slice(2, n_status*(N+1), n_status), Slice(0)) = map_dim[4];
    ubx(Slice(2, n_status*(N+1), n_status), Slice(0)) = map_dim[5];
    lbx(Slice(3, n_status*(N+1), n_status), Slice(0)) = -inf;
    ubx(Slice(3, n_status*(N+1), n_status), Slice(0)) = inf;

    lbx(Slice(n_status*(N+1), n_status*(N+1) + n_controls*N, n_controls), Slice(0)) = v_min;
    ubx(Slice(n_status*(N+1), n_status*(N+1) + n_controls*N, n_controls), Slice(0)) = v_max;
    lbx(Slice(n_status*(N+1)+1, n_status*(N+1) + n_controls*N, n_controls), Slice(0)) = v_min;
    ubx(Slice(n_status*(N+1)+1, n_status*(N+1) + n_controls*N, n_controls), Slice(0)) = v_max;
    lbx(Slice(n_status*(N+1)+2, n_status*(N+1) + n_controls*N, n_controls), Slice(0)) = v_min;
    ubx(Slice(n_status*(N+1)+2, n_status*(N+1) + n_controls*N, n_controls), Slice(0)) = v_max;
    lbx(Slice(n_status*(N+1)+3, n_status*(N+1) + n_controls*N, n_controls), Slice(0)) = omega_min;
    ubx(Slice(n_status*(N+1)+3, n_status*(N+1) + n_controls*N, n_controls), Slice(0)) = omega_max;

    double t0 = 0;
    DM x0 = DM(4,1);
    x0(0,0) = -5, x0(1,0) = -5, x0(2) = 0, x0(3,0) = 0;
    vector<double> t;
    t.push_back(t0);
    DM u0 = DM::zeros(N, n_controls);
    DM X0 = repmat(x0, 1, N+1).T();
    int sim_time = 60;
    int mpciter = 0;
    vector<vector<double>> xx1, u_cl, xx0, xx;

    DM p(n_status*2);
    std::map<std::string, DM> args, res;
    args["lbx"] = lbx;
    args["ubx"] = ubx;
    args["lbg"] = lbg;
    args["ubg"] = ubg;
    args["p"] = p;
    args["x0"] = x0;

    double error(norm_2(x0-xs));

    while(error > 1e-2 && mpciter < sim_time/T){
        args["p"] = vertcat(x0, xs);
        args["x0"] = vertcat(reshape(X0.T(), n_status*(N+1), 1), reshape(u0.T(), n_controls*N, 1));
        res = solver(args);
        DM u = reshape(res["x"](Slice(n_status*(N+1), res.at("x").size1())).T(), n_controls, N).T();
        vector<double> u11(reshape(res["x"](Slice(n_status*(N+1), res.at("x").size1())).T(), n_controls, N).T());
        vector<double> xx11(res["x"](Slice(0, (n_status*(N+1)))).T());
        xx1.push_back(xx11);
        u_cl.push_back(u11);
        t.push_back(t0);
        tuple<double, DM, DM> shiftted_val = shift(T, t0, x0, u, f);
        x0 = get<1>(shiftted_val);
        u = get<2>(shiftted_val);
        t0 = get<0>(shiftted_val);
        X0 = reshape(res["x"](Slice(0, n_status*(N+1))).T(), n_status, N+1).T();
        vector<double> xxxo(x0);
        xx0.push_back(xxxo);

        SX x0_rest = X0(Slice(1, X0.size1()), Slice(0, X0.size2()));
        SX x0_last = X0(Slice(X0.size1()-1, X0.size1()), Slice(0, X0.size2()));
        X0 = vertcat(x0_rest, x0_last);
        mpciter = mpciter + 1;
    }

    std::cout<< "Start saving files" << std::endl;
    string  stotage_location = "/root/catkin_ws/src/motion_planning/motion_planning_alg/data/";

    string file_name = stotage_location + "current_state.npy";
    save_double_vector(xx0, file_name, 1);

    file_name = stotage_location + "prediction_horizon_poses.npy";
    std::cout<< xx1.size() << "   " << xx1[0].size() << std::endl;
    save_double_vector(xx1, file_name, 1);

    file_name = stotage_location + "u_cl.npy";
    save_double_vector(u_cl, file_name, 4);

    file_name = stotage_location + "t.npy";
    save_vector(t, file_name, 1);

    file_name = stotage_location + "xs.npy";
    save_dm(xs, file_name, 1);

    return 0;
}


