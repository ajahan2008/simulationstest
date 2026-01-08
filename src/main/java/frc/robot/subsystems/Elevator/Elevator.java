// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import edu.wpi.first.wpilibj.Preferences;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator.ElevatorConstants;

public class Elevator extends SubsystemBase {

  private double tunablekP = ElevatorConstants.kP;
  private double tunablekI = ElevatorConstants.kI;
  private double tunablekD = ElevatorConstants.kD;
  private double tunablekS = ElevatorConstants.kS;
  private double tunablekG = ElevatorConstants.kG;
  private double tunablekV = ElevatorConstants.kV;
  private double tunablekA = ElevatorConstants.kA;

  private final DCMotor gearBox = DCMotor.getKrakenX60(1);
  private final CANcoder encoder = new CANcoder(ElevatorConstants.encoderPort);
  private final TalonFX motor = new TalonFX(17);
  private final ProfiledPIDController pidController =
    new ProfiledPIDController(
      tunablekP, 
      tunablekI, 
      tunablekD, 
      new TrapezoidProfile.Constraints(2.45, 2.45));
  ElevatorFeedforward feedForward = 
    new ElevatorFeedforward(
      tunablekS, 
      tunablekG, 
      tunablekV, 
      tunablekA);
  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          gearBox,
          ElevatorConstants.kGearing,
          ElevatorConstants.kCarriageMass,
          ElevatorConstants.kDrumRadius,
          ElevatorConstants.kMinHeightMeters,
          ElevatorConstants.kMaxHeightMeters,
          true,
          0,
          0.01,
          0.0);
  private final EncoderSim encoderSim = new EncoderSim(encoder);
  private final TalonFXSimState motorSim = new TalonFXSimState(motor);

  public void reachGoal(double goal) {
    pidController.setGoal(goal);
    double pidOutput = pidController.calculate(encoder.getDistance());
    double feedForwardOutput = feedForward.calculate(pidController.getSetpoint().velocity);
    motor.setVoltage(pidOutput + feedForwardOutput);
  }

  public void stop() {
    pidController.setGoal(0);
    motor.stopMotor();
  }

  public void loadPreferences() {

  }

  private final Mechanism2d mechanism2d = new Mechanism2d(20, 50);
  private final MechanismRoot2d mechanism2dRoot = mechanism2d.getRoot("Elevator Root", 10, 0);
  private final MechanismLigament2d elevatorMechanism2d = mechanism2dRoot.append(new MechanismLigament2d("Elevator", m_elevatorSim.getPositionMeters(), 90));

  /** Creates a new Elevator. */
  public Elevator() {
    encoder.setDistancePerPulse(ElevatorConstants.kEncoderDistPerPulse);
    SmartDashboard.putData("Elevator Sim", mechanism2d);
    loadPreferences();

    Preferences.initDouble(ElevatorConstants.kPkey, tunablekP);
    Preferences.initDouble(ElevatorConstants.kIkey, tunablekI);
    Preferences.initDouble(ElevatorConstants.kDkey, tunablekD);
    Preferences.initDouble(ElevatorConstants.kSkey, tunablekS);
    Preferences.initDouble(ElevatorConstants.kGkey, tunablekG);
    Preferences.initDouble(ElevatorConstants.kVkey, tunablekV);
    Preferences.initDouble(ElevatorConstants.kAkey, tunablekA);
    
  }

  @Override
  public void periodic() {
    dataLogging();
    // This method will be called once per scheduler runP
  }

  public void updateTelemetry() {
    loadPreferences();
    elevatorMechanism2d.setLength(encoder.getDistance());
  }

  public void dataLogging() {
    SmartDashboard.putNumber("Elevator Position", encoder.getDistance());
  }

  public void simulationPeriodic() {
    loadPreferences();
    // set inputs
    m_elevatorSim.setInput(motorSim.getMotorVoltage());

    // update 20ms
    m_elevatorSim.update(0.020);

    // set readings and battery voltage
    encoderSim.setDistance(m_elevatorSim.getPositionMeters());
    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
  }

}
