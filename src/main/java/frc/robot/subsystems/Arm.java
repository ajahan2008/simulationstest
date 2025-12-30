// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.google.flatbuffers.Constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

  // tuning via ascope
  private LoggedNetworkNumber tunablekP = new LoggedNetworkNumber("/Tuning/Arm/TunablekP", ArmConstants.kDefaultkP);
  
  // values
  private double kP = tunablekP.get();
  private double setpointDegrees = ArmConstants.kDefaultSetpointDegrees;

  // devices
  private final DCMotor gearbox = DCMotor.getKrakenX60(1);
  private final TalonFX motor = new TalonFX(ArmConstants.kMotorPort);
  private final CANcoder encoder = new CANcoder(ArmConstants.kEncoderPort);

  // control
  // TODO: Add feedforward control system(?)
  private final PIDController pidController = new PIDController(kP, 0, 0);

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
  }

  public void simulationPeriodic() {
    armSim.setInput(motor.get() * RobotController.getBatteryVoltage());
    armSim.update(.020);
    encoderSim.setRawPosition(armSim.getAngleRads());
    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));
    arm.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));
  }

  public void loadPreferences() {
    setpointDegrees = Preferences.getDouble(ArmConstants.kPositionKey, setpointDegrees);
    if (kP != Preferences.getDouble(ArmConstants.kPKey, kP)) {
      kP = Preferences.getDouble(ArmConstants.kPKey, kP);
      pidController.setP(kP);
    }
  }

  public void reachSetpoint() {
    var pidOutput = 
      pidController.calculate(
        encoder.getPosition().getValueAsDouble(), Units.degreesToRadians(setpointDegrees));
    motor.setVoltage(pidOutput);
  }

  public void stop() {
    motor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
