// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  private final SingleJointedArmSim armSim = 
    new SingleJointedArmSim(
      gearbox, 
      ArmConstants.kReduction, 
      SingleJointedArimSim.estimateMOI(ArmConstants.kLength, ArmConstants.kMass), 
      ArmConstants.kLength, 
      ArmConstants.kMinAngleRads, 
      ArmConstants.kMaxAngleRads, 
      true, 
      0,
      ArmConstants.kEncoderDistPerPulse,
      0.0);

  /** Creates a new Arm. */
  public Arm() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
