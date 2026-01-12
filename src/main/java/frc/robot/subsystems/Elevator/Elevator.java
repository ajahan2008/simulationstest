// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.Preferences;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

  private double kP = ElevatorConstants.kP;
  private double kI = ElevatorConstants.kI;
  private double kD = ElevatorConstants.kD;
  private double kS = ElevatorConstants.kS;
  private double kG = ElevatorConstants.kG;
  private double kV = ElevatorConstants.kV;
  private double kA = ElevatorConstants.kA;

  private final DCMotor gearBox = DCMotor.getKrakenX60(1);
  private final CANcoder encoder = new CANcoder(ElevatorConstants.encoderPort);
  private final TalonFX motor = new TalonFX(17);
  private final ProfiledPIDController pidController =
    new ProfiledPIDController(
      kP, 
      kI, 
      kD, 
      new TrapezoidProfile.Constraints(2.45, 2.45));
  ElevatorFeedforward feedForward = 
    new ElevatorFeedforward(
      kS, 
      kG, 
      kV, 
      kA);
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
  private final CANcoderSimState encoderSim = new CANcoderSimState(encoder);
  private final TalonFXSimState motorSim = new TalonFXSimState(motor);
  
  private final Mechanism2d mechanism2d = new Mechanism2d(20, 50);
  private final MechanismRoot2d mechanism2dRoot = mechanism2d.getRoot("Elevator Root", 10, 0);
  private final MechanismLigament2d elevatorMechanism2d = mechanism2dRoot.append(new MechanismLigament2d("Elevator", m_elevatorSim.getPositionMeters(), 90));

  /** Creates a new Elevator. */
  public Elevator() {
    SmartDashboard.putData("Elevator Sim", mechanism2d);
    loadPreferences();

    Preferences.initDouble(ElevatorConstants.kPkey, kP);
    Preferences.initDouble(ElevatorConstants.kIkey, kI);
    Preferences.initDouble(ElevatorConstants.kDkey, kD);
    Preferences.initDouble(ElevatorConstants.kSkey, kS);
    Preferences.initDouble(ElevatorConstants.kGkey, kG);
    Preferences.initDouble(ElevatorConstants.kVkey, kV);
    Preferences.initDouble(ElevatorConstants.kAkey, kA);
  }

  public void reachGoal(double goal) {
    pidController.setGoal(goal);
    double pidOutput = pidController.calculate(encoder.getPosition().getValueAsDouble());
    double feedForwardOutput = feedForward.calculate(pidController.getSetpoint().velocity);
    motor.setVoltage(pidOutput + feedForwardOutput);
  }

  public void stop() {
    motor.stopMotor();
  }

  public void loadPreferences() {
    if (kP != Preferences.getDouble(ElevatorConstants.kPkey, kP)) {
      System.out.println("Old kP: " + kP);
      kP = Preferences.getDouble(ElevatorConstants.kPkey, kP);
      pidController.setP(kP);
      System.out.println("New kP: " + kP);
    }
    if (kI != Preferences.getDouble(ElevatorConstants.kIkey, kI)) {
      System.out.println("Old kI: " + kI);
      kI = Preferences.getDouble(ElevatorConstants.kIkey, kI);
      pidController.setI(kI);
      System.out.println("New kI: " + kI);
    }
    if (kD != Preferences.getDouble(ElevatorConstants.kDkey, kD)) {
      System.out.println("Old kD: " + kD);
      kD = Preferences.getDouble(ElevatorConstants.kDkey, kD);
      pidController.setD(kD);
      System.out.println("New kD: " + kD);
    }
    if (kS != Preferences.getDouble(ElevatorConstants.kSkey, kS)) {
      System.out.println("Old kS: " + kS);
      kS = Preferences.getDouble(ElevatorConstants.kSkey, kS);
      System.out.println("New kS: " + kS);
    }
    if (kG != Preferences.getDouble(ElevatorConstants.kGkey, kG)) {
      System.out.println("Old kG: " + kG);
      kG = Preferences.getDouble(ElevatorConstants.kGkey, kG);
      System.out.println("New kG: " + kG);
    }
    if (kV != Preferences.getDouble(ElevatorConstants.kVkey, kV)) {
      System.out.println("Old kV: " + kV);
      kV = Preferences.getDouble(ElevatorConstants.kVkey, kV);
      System.out.println("New kV: " + kV);
    }
    if (kA != Preferences.getDouble(ElevatorConstants.kAkey, kA)) {
      System.out.println("Old kA: " + kA);
      kA = Preferences.getDouble(ElevatorConstants.kAkey, kA);
      System.out.println("New kA: " + kA);
    }
  }

  @Override
  public void periodic() {
    loadPreferences();
    dataLogging();
    // This method will be called once per scheduler runP
  }

  public void updateTelemetry() {
    elevatorMechanism2d.setLength(encoder.getPosition().getValueAsDouble());
  }

  public void dataLogging() {
    SmartDashboard.putNumber("Elevator Position", encoder.getPosition().getValueAsDouble());
  }

  public void simulationPeriodic() {
    // set inputs
    m_elevatorSim.setInput(motorSim.getMotorVoltage());

    // update 20ms
    m_elevatorSim.update(0.020);

    // set readings and battery voltage
    encoderSim.setRawPosition(m_elevatorSim.getPositionMeters());
    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
  }

  public boolean atSetpoint() {
    return pidController.atGoal();
  }

  public Command elevatorUp() {
    return runOnce( () -> {
        reachGoal(ElevatorConstants.kSetpointMeters);
      })
      .andThen(runOnce(() -> motor.stopMotor()));
  }

  public Command elevatorDown() {
    return runOnce( () -> {
        motor.stopMotor();
      });
  }

}
