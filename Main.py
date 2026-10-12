from constants import Constants
import sim
# main file to run the simulation
def main():
    if Constants.Simulation.ENABLE_SIMULATION:

        sim.sim()

    else:

        print("Simulation is disabled. Running tests instead.")

        from Tests import test_kinematics, test_vision

        if Constants.Simulation.KIND_OF_TEST == 'kinematics':
        
            test_kinematics.test_kinematics(Constants.Simulation.NUMBER_OF_TESTS)
        
        if Constants.Simulation.KIND_OF_TEST == 'vision':
        
            test_vision.test_vision(Constants.Simulation.NUMBER_OF_TESTS)


if __name__ == "__main__":
    main()