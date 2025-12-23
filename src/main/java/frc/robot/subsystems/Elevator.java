// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

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
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

  private LoggedNetworkNumber tunablekP = new LoggedNetworkNumber("/Tuning/Elevator/tunablekP", ElevatorConstants.kP);
  private LoggedNetworkNumber tunablekI = new LoggedNetworkNumber("/Tuning/Elevator/tunablekI", ElevatorConstants.kI);
  private LoggedNetworkNumber tunablekD = new LoggedNetworkNumber("/Tuning/Elevator/tunablekD", ElevatorConstants.kD);
  private LoggedNetworkNumber tunablekS = new LoggedNetworkNumber("/Tuning/Elevator/tunablekS", ElevatorConstants.kS);
  private LoggedNetworkNumber tunablekG = new LoggedNetworkNumber("/Tuning/Elevator/tunablekG", ElevatorConstants.kG);
  private LoggedNetworkNumber tunablekV = new LoggedNetworkNumber("/Tuning/Elevator/tunablekV", ElevatorConstants.kV);
  private LoggedNetworkNumber tunablekA = new LoggedNetworkNumber("/Tuning/Elevator/tunablekA", ElevatorConstants.kA);

  private final DCMotor gearBox = DCMotor.getKrakenX60(1);
  private final Encoder encoder = new Encoder(ElevatorConstants.kEncoderAChannel, ElevatorConstants.kEncoderBChannel);
  private final TalonFX motor = new TalonFX(17);
  private final ProfiledPIDController pidController =
    new ProfiledPIDController(
      tunablekP.get(), 
      tunablekI.get(), 
      tunablekD.get(), 
      new TrapezoidProfile.Constraints(2.45, 2.45));
  ElevatorFeedforward feedForward = 
    new ElevatorFeedforward(
      tunablekS.get(), 
      tunablekG.get(), 
      tunablekV.get(), 
      tunablekA.get());
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

  public void updateData() {
    pidController.setP(tunablekP.get());
    pidController.setI(tunablekI.get());
    pidController.setD(tunablekD.get());
    SmartDashboard.putNumber("Elevator Position", m_elevatorSim.getPositionMeters());
    SmartDashboard.putNumber("Elevator PID Error", pidController.getPositionError());
    pidController.setP(tunablekP.get());
    pidController.setI(tunablekI.get());
    pidController.setD(tunablekD.get());
  }

  private final Mechanism2d mechanism2d = new Mechanism2d(20, 50);
  private final MechanismRoot2d mechanism2dRoot = mechanism2d.getRoot("Elevator Root", 10, 0);
  private final MechanismLigament2d elevatorMechanism2d = mechanism2dRoot.append(new MechanismLigament2d("Elevator", m_elevatorSim.getPositionMeters(), 90));

  /** Creates a new Elevator. */
  public Elevator() {
    encoder.setDistancePerPulse(ElevatorConstants.kEncoderDistPerPulse);
    SmartDashboard.putData("Elevator Sim", mechanism2d);
    updateData();
  }

  @Override
  public void periodic() {
    tunablekP.periodic();
    updateData();
    // This method will be called once per scheduler runP
  }

  public void updateTelemetry() {
    updateData();
    elevatorMechanism2d.setLength(encoder.getDistance());
  }

  public double getP() {
    return pidController.getP();
  }

  public double getTunableP() {
    return tunablekP.get();
  }

  public void simulationPeriodic() {
    updateData();
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
