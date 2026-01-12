package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.util.Units;

public class ElevatorConstants {

    public static final int kMotorPort = 17;
    public static final int encoderPort = 18;
        
    public static final double kP = 5;
    public static final String kPkey = "ElevatorP";
    public static final double kI = 0;
    public static final String kIkey = "ElevatorI";
    public static final double kD = 0;
    public static final String kDkey = "ElevatorD";
    public static final double kS = 0.0; // volts
    public static final String kSkey = "ElevatorS";
    public static final double kG = 0.762; // volts
    public static final String kGkey = "ElevatorG";
    public static final double kV = 0.762; // volts per velocity (V/(m/s))
    public static final String kVkey = "ElevatorV";
    public static final double kA = 0.0; // volts per acceleration (V/(m/s²))
    public static final String kAkey = "ElevatorA";

    public static final double kGearing = 10.0;
    public static final double kDrumRadius = Units.inchesToMeters(2.0);
    public static final double kCarriageMass = 4.0; // kg

    public static final double kSetpointMeters = .75;
    public static final double kMinHeightMeters = 0.0;
    public static final double kMaxHeightMeters = 1.25;

    // distance per pulse = (distance per revolution) / (pulses per revolution)
    // = (Pi * D) / ppr
    public static final double kEncoderDistPerPulse = 
        2.0 * Math.PI * kDrumRadius / 4096;

}
