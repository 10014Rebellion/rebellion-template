// REBELLION 10014

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static class DashboardConstants {
        public static final boolean DASHBOARD_ENABLED = true;
        public static final String DASHBOARD_PATH = "dashboard";
        public static final int DASHBOARD_PORT = 5800;

        public static final boolean DEPLOY_SERVER_ENABLED = false;
        public static final String DEPLOY_SERVER_PATH = "";
        public static final int DEPLOY_SERVER_PORT = 5801;
    }
}
