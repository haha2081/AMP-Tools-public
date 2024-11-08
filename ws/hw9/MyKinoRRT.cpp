#include "MyKinoRRT.h"


Eigen::VectorXd generateRandomNVector(const std::vector<double> &lower_bounds, const std::vector<double> &upper_bounds)
{
    if (lower_bounds.size() != upper_bounds.size())
    {
        throw std::invalid_argument("Erororororororrr");
    }

    int n = lower_bounds.size();
    Eigen::VectorXd random_vector(n);
    std::random_device rd;
    std::mt19937 gen(rd());
    for (int i = 0; i < n; ++i)
    {
        std::uniform_real_distribution<float> dist(lower_bounds[i], upper_bounds[i]);
        random_vector[i] = dist(gen);
    }

    return random_vector;
}




void MySingleIntegrator::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {
    state += dt * control;
}

amp::KinoPath MyKinoRRT::plan(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent) {
    amp::KinoPath path;
    Eigen::VectorXd state = problem.q_init;

//    1:   T ← create tree rooted at 𝑥0 
//    2:   While  solution not found  do        
//    3:    𝑥𝑟𝑎𝑛𝑑 ← StateSample() 
//    4:    𝑥𝑛𝑒𝑎𝑟   ←  nearest state in 𝑇 to 𝑥𝑟𝑎𝑛𝑑  according to distance 𝜌       
//    5:   𝜆 ← GenerateLocalTrajectory(𝑥𝑛𝑒𝑎𝑟 , 𝑥𝑟𝑎𝑛𝑑) 

        //1:   for  𝑖 = 1, … , 𝑚  do 
        //2:      𝑢 ← sample random control in 𝑈 
        //3:      𝜆 ← 𝑥 𝑡 = 𝑥𝑛𝑒 𝑎𝑟 + ׬ Δ𝑡 𝑓 𝑥 𝜏 , 𝑢  𝑑𝜏
        //4:      di ← 𝜌(𝑥𝑟𝑎𝑛𝑑 , 𝜆𝑖 (Δ𝑡) 
        //5:   return 𝜆𝑖  with minimum 𝑑𝑖

//    6:   if  IsSubTrajectoryValid(𝜆, 0, 𝑠𝑡𝑒𝑝)  then 
//    7:    𝑥𝑛𝑒𝑤  ←  𝜆(𝑠𝑡𝑒𝑝) 
//    8:    add configuration 𝑥𝑛𝑒𝑤 and edge (𝑥𝑛𝑒𝑎𝑟 , 𝑥𝑛𝑒𝑤) to 𝑇    
//    9:   if  𝜌(𝑥𝑛𝑒𝑤 , 𝑥𝑔𝑜𝑎𝑙 )   ≈  0  then 
//    10:    return  solution trajectory from root to 𝑥𝑛𝑒w


    path.waypoints.push_back(state);
    // for (int i = 0; i < 10; i++) {
    //     Eigen::VectorXd control = Eigen::VectorXd::Random(problem.q_init.size());
    //     agent.propagate(state, control, 1.0);
    //     path.waypoints.push_back(state);
    //     path.controls.push_back(control);
    //     path.durations.push_back(1.0);
    // }
    path.valid = true;

std::cout << "Agent L: " <<problem.agent_dim.length << "Agent W: " <<problem.agent_dim.width<< std::endl;


std::cout <<"q_init: " <<problem.q_init[0] << " " << problem.q_init[1] << std::endl;

std::cout << "q_goal: " << problem.q_goal[0].first  << " " << problem.q_goal[0].second << std::endl;

std::cout << "q_bounds: " << problem.q_bounds[0].first  << " " << problem.q_bounds[0].second << std::endl;













    

    return path;
}

double carLength = 5;
double carWidth = 2;

void MySimpleCar::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {
    RungeKutta4(state, control, dt);

}

void MySimpleCar::RungeKutta4(Eigen::VectorXd& state, const Eigen::VectorXd& control, double dt) {
    Eigen::VectorXd newState(5), w1(5), w2(5), w3(5), w4(5);

    double x = state(0);
    double y = state(1);
    double theta = state(2);
    double v = state(3);
    double phi = state(4);

    double u1 = control(0);
    double u2 = control(1);

    double der1 = v * cos(theta);
    double der2 = v * sin(theta);
    double der3 = v / carLength * tan(phi);
    double der4 = u1;
    double der5 = u2;

    w1 << der1, der2, der3, der4, der5;

    w2 << w1(0) + 0.5 * dt * w1(0),
          w1(1) + 0.5 * dt * w1(1),
          w1(2) + 0.5 * dt * w1(2),
          w1(3) + 0.5 * dt * w1(3),
          w1(4) + 0.5 * dt * w1(4);

    w3 << w1(0) + 0.5 * dt * w2(0),
          w1(1) + 0.5 * dt * w2(1),
          w1(2) + 0.5 * dt * w2(2),
          w1(3) + 0.5 * dt * w2(3),
          w1(4) + 0.5 * dt * w2(4);

    w4 << w1(0) + dt * w3(0),
          w1(1) + dt * w3(1),
          w1(2) + dt * w3(2),
          w1(3) + dt * w3(3),
          w1(4) + dt * w3(4);

    newState = state + (dt / 6.0) * (w1 + 2 * w2 + 2 * w3 + w4);

    state = newState;
}






