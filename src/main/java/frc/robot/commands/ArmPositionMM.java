// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmPositionMM extends CommandBase {

  private ArmSubsystem arm = ArmSubsystem.getInstance();
  Timer timer;
  double position;

  /** Creates a new MotorPosition. */
  public ArmPositionMM(double position) {
    timer = new Timer();
    this.addRequirements(arm);
    this.timer.start();
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.arm.putArmInPosMM(position);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setSpeed(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(arm.getMotorPosition() - this.position) > Constants.ArmConstants.DifferenceFromPosition){
      this.timer.restart();
    }
      return this.timer.hasElapsed(Constants.ArmConstants.timeInPosition);
    
   
    }

  }
