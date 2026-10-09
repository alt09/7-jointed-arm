from constants import Constants
import sim
# main file to run the simulation
def main():
    if Constants.Simulation.ENABLE_SIMULATION:

        sim.sim()

    else:

        print("Simulation is disabled. Running tests instead.")
        import tests
        tests.test_kinematics(10000)

if __name__ == "__main__":
    main()