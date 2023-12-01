// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  private TalonFX motor;
  private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    this.motor = new TalonFX(Constants.ArmConstants.KMotorID);

    TalonFXConfiguration configs = new TalonFXConfiguration();
    MotionMagicConfigs mm = new MotionMagicConfigs();

    mm.MotionMagicCruiseVelocity = Constants.ArmConstants.MotionMagicCruiseVelocity;
    mm.MotionMagicAcceleration = Constants.ArmConstants.MotionMagicAcceleration;
    mm.MotionMagicJerk = Constants.ArmConstants.MotionMagicJerk;

    configs.MotionMagic = mm;
    configs.Slot0.kP = Constants.ArmConstants.kp;
    configs.Slot0.kD = Constants.ArmConstants.kd;
    configs.Slot0.kV = Constants.ArmConstants.kv;

    configs.Voltage.PeakForwardVoltage = Constants.ArmConstants.PeakForwardVoltage;
    configs.Voltage.PeakReverseVoltage = Constants.ArmConstants.PeakReverseVoltage ;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i){
      status = motor.getConfigurator().apply(configs);
      if (status.isOK())
        break;
    }
    if (!status.isOK()){
      System.out.println("could not apply configs, error code: " + status.toString()  );
    }

    configs.Feedback.SensorToMechanismRatio = Constants.ArmConstants.Feedback;
  }
 
  public void putArmInPosMM(double position){
    motor.setControl(motionMagic.withPosition(position));
  }
  private static ArmSubsystem instance;

  public static ArmSubsystem getInstance() {
    if (instance == null) {
      instance = new ArmSubsystem();
    }
    return instance;
  }

  public void setSpeed(double speed) {
    this.motor.set(speed);
  }

  public double getMotorPosition() {
    return motor.getPosition().getValue();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}


