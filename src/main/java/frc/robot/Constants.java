package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
    
    public class ArmConstants {

        public static final int kMotorPort = 0;
        public static final int kEncoderAChannel = 0;
        public static final int kEncoderBChannel = 1;
        
        public static final String kArmPositionString = "ArmPosition";
        public static final String kArmPKey = "ArmP";

        public static final double kDefaultArmKp = 50.0;
        public static final double kDefaultArmKi = 0;
        public static final double kDefaultArmKd = 0;
        
        public static final double kDefaultArmSetpointDegrees = 75.0;

        public static final double kArmEncoderDistPerPuless = 2.0 * Math.PI / 4096;

        public static final double kArmReduction = 200;
        public static final double kArmMass = 8.0; // mass
        public static final double kArmLength = Units.inchesToMeters(30);
        public static final double kMinAngleRads = Units.degreesToRadians(-75);
        public static final double kMaxAngleRads = Units.degreesToRadians(255);

    }

}
