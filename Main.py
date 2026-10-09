from constants import Constants
import sim
# main file to run the simulation
def main():
    if Constants.Simulation.ENABLE_SIMULATION:

        sim.sim()

    else:

        print("Simulation is disabled. Running tests instead.")
        import tests
        # tests.test_kinematics(100)
        tests.test_vision(27) # number of tests must be a perfect cube (1, 8, 27, 64, 125, 216, 343, 512, 729, 1000)

if __name__ == "__main__":
    main()