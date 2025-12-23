// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;

public class Arm extends SubsystemBase {

  private double kP = ArmConstants.kDefaultArmKp;
  private double kI = ArmConstants.kDefaultArmKi;
  private double kD = ArmConstants.kDefaultArmKd;

  private double setpointDegrees = ArmConstants.kDefaultArmKp;

  private final DCMotor gearbox = DCMotor.getKrakenX60(1);

  private final PIDController pidController = new PIDController(kP, kI, kD);
  private final Encoder encoder = new Encoder(ArmConstants.kEncoderAChannel, ArmConstants.kEncoderBChannel);
  private final TalonFX motor = new TalonFX(ArmConstants.kMotorPort);

  private final SingleJointedArmSim armSim =
    new SingleJointedArmSim(
      gearbox, 
      ArmConstants.kArmReduction, 
      SingleJointedArmSim.estimateMOI(ArmConstants.kArmLength, ArmConstants.kArmMass), 
      ArmConstants.kArmLength, 
      ArmConstants.kMinAngleRads, 
      ArmConstants.kMaxAngleRads, 
      true, 
      0, 
      ArmConstants.kArmEncoderDistPerPuless,
      0.0);

  private final EncoderSim encoderSim = new EncoderSim(encoder);

  /** Creates a new Arm. */
  public Arm() {
    encoder.setDistancePerPulse(ArmConstants.kArmEncoderDistPerPuless);
    
    Preferences.initDouble(ArmConstants.kArmPositionString, setpointDegrees);
    Preferences.initDouble(ArmConstants.kArmPKey, kP);
  }

  public void simulationPeriodic() {
    armSim.setInput(motor.get() * RobotController.getBatteryVoltage());
    armSim.update(0.020);
    encoderSim.setDistance(armSim.getAngleRads());
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));
  }

  public void loadPreferences() {
    setpointDegrees = Preferences.getDouble(ArmConstants.kArmPositionString, setpointDegrees);
    if (kP != Preferences.getDouble(ArmConstants.kArmPKey, kP)) {
      kP = Preferences.getDouble(ArmConstants.kArmPKey, kP);
      pidController.setP(kP);
    }
  }

  public void reachSetpoint() {
    var pidOutput = 
      pidController.calculate(
        encoder.getDistance(), Units.degreesToRadians(setpointDegrees));
      motor.setVoltage(pidOutput);
  }

  public void stop() {
    motor.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
