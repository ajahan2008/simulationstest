package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
    public class ArmConstants {
        // ports
        public static final int kMotorPort = 41;
        public static final int kEncoderPort = 42;
        
        // keys
        public static final String kPositionKey = "ArmPosition";
        public static final String kPKey = "ArmP";

        // pid
        public static final double kDefaultkP = 50;
        public static final double kDefaultSetpointDegrees = 75.0;
        public static final double kDefaultHomeDegrees = -75;

        // dpp
        public static final double kEncoderDistPerPulse = 2.0 * Math.PI / 4096;

        // specs
        public static final double kReduction = 200;
        public static final double kMass = 8;
        public static final double kLength = Units.inchesToMeters(30);
        public static final double kMinAngleRads = Units.degreesToRadians(-75);
        public static final double kMaxAngleRads = Units.degreesToRadians(255);
    }
}
