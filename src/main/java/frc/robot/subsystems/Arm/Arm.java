// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.google.flatbuffers.Constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.ArmHome;
import frc.robot.subsystems.Arm.ArmConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends SubsystemBase {

  // tuning via ascope
  private LoggedNetworkNumber tunablekP = new LoggedNetworkNumber("/Tuning/Arm/TunablekP", ArmConstants.kDefaultkP);
  
  // values
  private double kP = ArmConstants.kDefaultkP;
  private double kI = ArmConstants.kDefaultkI;
  private double kD = ArmConstants.kDefaultkD;
  private double setpointDegrees = ArmConstants.kDefaultSetpointDegrees;

  // devices
  private final DCMotor gearbox = DCMotor.getKrakenX60(1);
  private final TalonFX motor = new TalonFX(ArmConstants.kMotorPort);
  private final CANcoder encoder = new CANcoder(ArmConstants.kEncoderPort);

  // control
  private final PIDController pidController = new PIDController(kP, kI, kD);

  // simulated devices

  private final CANcoderSimState encoderSim = encoder.getSimState();
  private final SingleJointedArmSim armSim = 
    new SingleJointedArmSim(
      gearbox, 
      ArmConstants.kReduction, 
      SingleJointedArmSim.estimateMOI(ArmConstants.kLength, ArmConstants.kMass), 
      ArmConstants.kLength, 
      ArmConstants.kMinAngleRads, 
      ArmConstants.kMaxAngleRads, 
      true, 
      0,
      ArmConstants.kEncoderDistPerPulse,
      0.0);

  // visualiation
  private final Mechanism2d mech2d = new Mechanism2d(60, 60);
  private final MechanismRoot2d armPivot = mech2d.getRoot("ArmPivot", 30, 30);
  private final MechanismLigament2d armTower = 
    armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
  private final MechanismLigament2d arm =
    armPivot.append(new MechanismLigament2d(
      "Arm", 
      30, 
      Units.radiansToDegrees(armSim.getAngleRads()),
      6,
      new Color8Bit(Color.kYellow)));

  /** Creates a new Arm. */
  public Arm() {
    encoder.setPosition(0);
    encoderSim.setRawPosition(0);

    SmartDashboard.putData("Arm Sim", mech2d);
    armTower.setColor(new Color8Bit(Color.kBlue));

    Preferences.initDouble(ArmConstants.kPositionKey, setpointDegrees);
    Preferences.initDouble(ArmConstants.kPKey, kP);
    Preferences.initDouble(ArmConstants.kIKey, kI);
    Preferences.initDouble(ArmConstants.kDKey, kD);
    
  }

  public void simulationPeriodic() {
    loadPreferences();
    armSim.setInput(motor.get() * RobotController.getBatteryVoltage());
    armSim.update(.020);
    encoderSim.setRawPosition(armSim.getAngleRads());
    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));
    arm.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));
  }

  public void dataLogging() {
    SmartDashboard.putNumber("Arm Position", encoder.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Arm Output A", armSim.getCurrentDrawAmps());
    SmartDashboard.putNumber("Arm Motor Output V", motor.getMotorVoltage().getValueAsDouble());
  }

  public void loadPreferences() {
    setpointDegrees = Preferences.getDouble(ArmConstants.kPositionKey, setpointDegrees);
    if (kP != Preferences.getDouble(ArmConstants.kPKey, kP)) {
      System.out.println("Old kP: " + kP);
      kP = Preferences.getDouble(ArmConstants.kPKey, kP);
      pidController.setP(kP);
      System.out.println("New kP: " + kP);
    }
    if (kI != Preferences.getDouble(ArmConstants.kIKey, kI)) {
      System.out.println("Old kI: " + kI);
      kI = Preferences.getDouble(ArmConstants.kIKey, kI);
      pidController.setI(kI);
      System.out.println("Old kI: " + kI);
    }
    if (kD != Preferences.getDouble(ArmConstants.kDKey, kD)) {
      System.out.println("Old kD: " + kD);
      kD = Preferences.getDouble(ArmConstants.kDKey, kD);
      pidController.setD(kD);
      System.out.println("Old kD: " + kD);
    }
  }

  public void reachSetpoint(double setpoint) {
    var pidOutput = 
      pidController.calculate(
        encoder.getPosition().getValueAsDouble(), Units.degreesToRadians(setpoint));
    motor.setVoltage(pidOutput);
  }

  public void stop() {
    motor.set(0);
  }

  public void runVoltage(double V) {
    motor.setVoltage(V);
  }

  @Override
  public void periodic() {
    dataLogging();
    // This method will be called once per scheduler run
  }
}
