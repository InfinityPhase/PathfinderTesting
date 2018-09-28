package main.java;

import jaci.pathfinder.Trajectory;

public class Calibration {
    /* Pathfinder */
    public final class Pathfinder {
        public static final double ROBOT_WIDTH = 0.66;
        public static final double KP = 2;
        public static final double KI = 0;
        public static final double KD = 0.0;
        public static final double KV = 1.0/2.3;
        public static final double KA = 0.03;
        public static final double K_KAPPA = 0.09;
        public static final double K_PTHETA_0 = 2.8;
        public static final double K_PTHETA_DECAY = 0.7;
        public static final double K_DTHETA_0 = 0.02;
        public static final double K_DTHETA_DECAY = 0.3;
    }
    public static final Trajectory.Config PATHFINDER_CONFIG = new Trajectory.Config(
        Trajectory.FitMethod.HERMITE_QUINTIC,
        Trajectory.Config.SAMPLES_HIGH,
        0.025, 1.6, 1.1, 3.5 );
}
