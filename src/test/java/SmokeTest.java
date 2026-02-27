// REBELLION 10014

import static org.junit.jupiter.api.Assertions.fail;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Robot;
import java.io.ByteArrayOutputStream;
import java.io.PrintStream;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class SmokeTest {
    static Thread robotThread;
    static AtomicReference<Throwable> threadCrashException;

    // Streams for intercepting WPILib console output
    static ByteArrayOutputStream errContent;
    static PrintStream originalErr;

    @BeforeEach
    void setup() {
        // 1. Setup exception catcher for the robot thread
        threadCrashException = new AtomicReference<>();
        robotThread = new Thread(() -> RobotBase.startRobot(Robot::new));
        robotThread.setUncaughtExceptionHandler((th, ex) -> threadCrashException.set(ex));

        // 2. Intercept System.err to catch DriverStation errors/warnings
        errContent = new ByteArrayOutputStream();
        originalErr = System.err;
        System.setErr(new PrintStream(errContent));

        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
        robotThread.start();
    }

    @Test
    void fiveSecondTest() throws InterruptedException {
        long startTime = System.currentTimeMillis();
        long timeout = 5000; // 5 seconds timeout

        while (System.currentTimeMillis() - startTime < timeout) {
            // Check if the thread threw a fatal exception
            if (threadCrashException.get() != null) {
                fail(
                        "Robot crashed with exception: "
                                + threadCrashException.get().getMessage(),
                        threadCrashException.get());
            }

            // Check if the thread died silently
            if (robotThread.getState() == Thread.State.TERMINATED) {
                fail("Robot thread terminated unexpectedly without a caught exception.");
            }

            // Yield to prevent the while loop from hogging the CPU
            Thread.sleep(100);
        }

        // 3. Analyze the intercepted error stream
        String consoleOutput = errContent.toString();

        // Check for WPILib-specific error tags or general exceptions
        if (consoleOutput.contains("Error") || consoleOutput.contains("Exception")) {
            fail("WPILib reported errors during the smoke test:\n" + consoleOutput);
        }

        // Optional: You can also fail on warnings, but WPILib often prints benign
        // warnings (like missing joysticks in sim), so use this with caution!
        // if (consoleOutput.contains("Warning")) { ... }
    }

    @AfterEach
    void tearDown() {
        // Restore the original System.err so regular testing output isn't hidden
        System.setErr(originalErr);
    }

    @AfterAll
    static void cleanup() {
        // Stop the robot thread
        if (robotThread != null && robotThread.isAlive()) {
            robotThread.interrupt();
        }

        HAL.exitMain();
        HAL.shutdown();
    }
}
