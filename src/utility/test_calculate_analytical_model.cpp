#include <algorithm> // for std::max and std::min
#include <iomanip>
#include <iostream>

std::unordered_map<std::string, double> calculate_model_variables(double c_max, double delta_m_commute, double nu_w_work, double nu_m_move,
                                                                  double nu_min, double nu_m_charge, double nu_m_transfer,
                                                                  double xi, double tau, double zeta) 
{
    double c_m_max = tau * c_max;

    // delta_m_rest for case 2
    double delta_m_rest_case_2 = std::max(0.0, 
        c_max / (nu_w_work + nu_min) * (1 - nu_min / nu_m_charge)
        - 2 * delta_m_commute * (1 + nu_m_move / nu_m_charge)
        - c_max / nu_m_charge * ((zeta / xi) * nu_m_transfer + nu_min) / (nu_m_transfer - nu_min)
    );

    // c_m_charged
    double c_m_charged = std::max(0.0, std::min(c_m_max,
        2 * (nu_m_move + nu_min) * delta_m_commute
        + c_max * ((zeta / xi) * nu_m_transfer + nu_min) / (nu_m_transfer - nu_min)
        + nu_min * delta_m_rest_case_2
    ));

    // delta_m_charge
    double delta_m_charge = c_m_charged / (nu_m_charge - nu_min);

    // delta_m_rest for case 1
    double delta_m_rest_case_1 = std::max(0.0, 
        (c_m_charged - 2 * (nu_m_move + nu_min) * delta_m_commute) / nu_min
        - (c_m_charged / (nu_m_charge - nu_min) * nu_m_charge - 2 * delta_m_commute * nu_m_move)
        / (nu_min + (nu_min * nu_min) / (nu_w_work + nu_min) * (nu_m_transfer - nu_min) / (((zeta / xi) * nu_m_transfer) + nu_min))
    );

    // delta_transfer
    double delta_transfer = std::max(0.0, std::min(
        c_max / (nu_m_transfer - nu_min),
        (c_m_charged - 2 * (nu_m_move + nu_min) * delta_m_commute) / (((zeta / xi) * nu_m_transfer) + nu_min)
        - (nu_min / (((zeta / xi) * nu_m_transfer) + nu_min)) * delta_m_rest_case_1
    ));

    // c_w_charged
    double c_w_charged = (nu_m_transfer - nu_min) * delta_transfer;

    // delta_m_rest
    double delta_m_rest = std::max(0.0,
        (c_m_charged - 2 * delta_m_commute * (nu_m_move + nu_min)) / nu_min
        - delta_transfer * (((zeta / xi) * nu_m_transfer) + nu_min) / nu_min
    );

    // delta_w_work
    double delta_w_work = std::max(0.0,
        (c_w_charged - (delta_m_charge + 2 * delta_m_commute + delta_m_rest) * nu_min) / nu_w_work
    );

    // delta_w_rest
    double delta_w_rest = std::max(0.0,
        (c_w_charged - delta_w_work * (nu_w_work + nu_min)) / nu_min
    );

    // // delta_m_wait
    // double delta_m_wait = delta_m_charge + delta_m_rest;

    // c_m_return
    double c_m_return = delta_m_commute * (nu_m_move + nu_min) + delta_m_rest * nu_min;

    std::unordered_map<std::string, double> result;
    result["c_w_charged"] = c_w_charged;
    result["c_m_return"] = c_m_return;
    result["delta_m_charge"] = delta_m_charge;
    result["delta_m_rest"] = delta_m_rest;
    result["delta_w_rest"] = delta_w_rest;
    return result;
}

int main() {
    // Sample input values
    double c_max = 2000;
    double delta_m_commute = 40;
    double nu_w_work = 1;
    double nu_m_move = 1;
    double nu_min = 0.005;
    double nu_m_charge = 50.0/9.0;
    double nu_m_transfer = 50.0/9.0;
    double xi = 0.99;
    double tau = 5;
    double zeta = 5;

    // Call the function
    auto result = calculate_model_variables(c_max, delta_m_commute, nu_w_work, nu_m_move,
                                            nu_min, nu_m_charge, nu_m_transfer,
                                            xi, tau, zeta);


    std::cout << std::fixed << std::setprecision(14);
    std::cout << "c_w_charged: " << result.at("c_w_charged") << std::endl;
    std::cout << "delta_m_wait: " << result.at("delta_m_wait") << std::endl;
    std::cout << "delta_w_rest: " << result.at("delta_w_rest") << std::endl;

    return 0;
}
