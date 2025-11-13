import numpy as np

def calculate_strat2_all_variables(c_max, delta_m_commute, nu_w_work, nu_m_move, nu_min, nu_m_charge, nu_m_transfer, xi, tau, zeta, n_m):
    c_m_max = tau * c_max # chargers capacity is tau times the workers capacity

    # extract delta_m_rest for case 2
    delta_m_rest_case_2 = np.maximum(0, c_max/(nu_w_work+nu_min)*(1-nu_min/nu_m_charge)-2*delta_m_commute*(1+nu_m_move/nu_m_charge) - c_max/nu_m_charge * (zeta/xi * nu_m_transfer + nu_min) / (nu_m_transfer - nu_min))
    #print(f"delta_m_rest_case_2: {delta_m_rest_case_2}")
    # either maximum capacity, or case 2 where it can supply all workers with full capacity even when having less than the maximum capacity, also limit to 0 in the (very unlikely) case that nu_min is way bigger than nu_m_transfer 
    c_m_charged = np.maximum(0, np.minimum(c_m_max, 2 * (nu_m_move + nu_min) * delta_m_commute + c_max * (zeta/xi * nu_m_transfer + nu_min) / (nu_m_transfer - nu_min) + nu_min * delta_m_rest_case_2))
    #print(f"c_m_charged: {c_m_charged}")
    # Directly following
    delta_m_charge = c_m_charged/(nu_m_charge-nu_min)
    print(f"delta_m_charge: {delta_m_charge}")
    delta_m_charge_different = 2 * delta_m_commute * (nu_m_move/nu_m_charge) + c_max/nu_m_charge *((zeta * nu_m_transfer + nu_min)/(xi * nu_m_transfer - nu_min) + nu_min/(nu_w_work + nu_min))
    #print(f"delta_m_charge different: {delta_m_charge_different}")
    c_m_charged_different = delta_m_charge_different * (nu_m_charge - nu_min)
    #print(f"c_m_charged different: {c_m_charged_different}")
    #print(f"difference in resting: {(c_m_charged - c_m_charged_different)/nu_min}")

    # extract delta_m_rest for case 1
    delta_m_rest_case_1 = np.maximum(0, (c_m_charged - 2 * (nu_m_move + nu_min) * delta_m_commute)/nu_min - (c_m_charged/(nu_m_charge - nu_min) * nu_m_charge - 2 * delta_m_commute * nu_m_move) / (nu_min + nu_min**2/(nu_w_work + nu_min) * (nu_m_transfer - nu_min)/(zeta/xi * nu_m_transfer + nu_min)))
    #print(f"delta_m_rest_case_1: {delta_m_rest_case_1}")
    delta_transfer = np.maximum(0, np.minimum(c_max /(nu_m_transfer - nu_min), (c_m_charged - 2 * (nu_m_move + nu_min) * delta_m_commute)/(zeta/xi * nu_m_transfer + nu_min) - nu_min/(zeta/xi * nu_m_transfer + nu_min) * delta_m_rest_case_1)) # Either the time the charger needs to give the worker its remaining capacity, or the time it needs to charge all workers to the maximum capacity (considering that there is a energy loss) 
    print(f"delta_transfer: {delta_transfer}")
    c_w_charged = (nu_m_transfer - nu_min) * delta_transfer # the max boundary for c_w_charged is already encoded in delta_transfer, so we can just multiply it with the rate at which the charger can transfer energy to the worker
    print(f"c_w_charged: {c_w_charged}")
    #print(f" diffeernt delta_m_rest: {c_w_charged/(nu_w_work + nu_min) - delta_m_charge - 2*delta_m_commute}")
    #print(f" diffeernt delta_m_rest 2: {c_w_charged/(nu_w_work + nu_min) - delta_m_charge_different - 2*delta_m_commute}")
    n_w = zeta * n_m 

    delta_m_rest = np.maximum(0, (c_m_charged - 2*delta_m_commute * (nu_m_move + nu_min))/nu_min - delta_transfer * (zeta/xi * nu_m_transfer + nu_min)/nu_min)
    print(f"delta_m_rest: {delta_m_rest}")
    # The maximum of delta_work should already be encapsulated in c_w_charged (delta_transfer, respectively)
    delta_w_work = np.maximum(0, (c_w_charged - (delta_m_charge + 2*delta_m_commute + delta_m_rest) * nu_min) / nu_w_work) # either the remaining capacity divided by the wpork rate when substracting the time the charger needs to get new energy or the time the worker needs to use all its charge for working
    print(f"delta_w_work: {delta_w_work}")
    # Different cases should be already encapsulated in delta_transfer and c_m_charged, so we can calculate it more generally as
    delta_w_rest = np.maximum(0, (c_w_charged - delta_w_work*(nu_w_work+nu_min))/nu_min)
    print(f"delta_w_rest: {delta_w_rest}")  
    cycle_duration = delta_m_charge + 2 * delta_m_commute + delta_transfer + delta_m_rest
    print(f"cycle_duration: {cycle_duration}")
    print(f"cycle_worker: {delta_w_work + delta_w_rest}")
    print(f"cycle_charger: {delta_m_charge + 2* delta_m_commute + delta_m_rest}")
    # shorter equation? still not sure if we can directly compare those with strat1
    energy_efficiency_2 = (n_w * delta_w_work * nu_w_work)/(n_m * (c_m_charged + delta_m_charge*nu_min)) # same as 1
    #energy_efficiency_2 = simplify(energy_efficiency_2)
    #md(f"$${energy_efficiency_Latex}$$")

    # duty cycle of worker; duty cycle of charger is 0? --> even so, we need delta_m_charge for later to get the beta in our equation; ignore the charger for the duty cycle?
    #duty_cycle = n_w * delta_w_work/(n_w * (delta_transfer + delta_w_work + delta_w_rest) + n_m * (delta_m_charge + 2 * delta_m_commute + delta_transfer + delta_m_rest))
    ### old one: duty_cycle = delta_w_work/((delta_transfer + delta_w_work + delta_w_rest + delta_m_charge + 2 * delta_m_commute + delta_transfer + delta_m_rest)/2) # divide by 2 because it should be the same but we need the chargers variables in there; only calculate for a single worker
    duty_cycle = delta_w_work/cycle_duration # divide by the cycle of a charger in the case that the workers cycle is not adding up due to a drained battery
    #duty_cycle_2 = delta_w_work/(delta_transfer + delta_w_work + delta_w_rest) # problematic if all 0 because charger cannot come? (if c_w_charged=0)
    #duty_cycle = simplify(duty_cycle)
    print(f"duty_cycle: {duty_cycle}")

    work_2 = n_w * duty_cycle

    # Time to remain at the base
    delta_m_wait = delta_m_rest + delta_m_charge

    return c_w_charged, delta_m_wait

# ----------------------------
# Test the function
# ----------------------------
if __name__ == "__main__":
    # Example input values
    c_max = 3060
    delta_m_commute = 40
    nu_w_work = 1
    nu_m_move = 1
    nu_min = 0.005
    nu_m_charge = 191.25
    nu_m_transfer = 191.25
    xi = 0.5
    tau = 5
    zeta = 5
    n_m = 1

    c_w_charged, delta_m_wait = calculate_strat2_all_variables(c_max, delta_m_commute, nu_w_work, nu_m_move,
                                        nu_min, nu_m_charge, nu_m_transfer,
                                        xi, tau, zeta, n_m)
    print(f"c_w_charged: {c_w_charged}")
    print(f"delta_m_wait: {delta_m_wait}")
