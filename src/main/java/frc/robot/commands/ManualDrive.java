// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.RobotContainer.*;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class ManualDrive extends Command {
  /** Creates a new ManualDrive. */
  private final SwerveSubsystem swerveSubsystem;
  private double xSpeed;
  private double ySpeed;
  private double zSpeed;
  public ManualDrive(SwerveSubsystem _swerveSubsystem) {
    this.swerveSubsystem = _swerveSubsystem;
    addRequirements(swerveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(baseJoystick.getRawButton(6) == true){
      xSpeed = Constants.SwerveConstants.joysickValue(baseJoystick.getRawAxis(1)*0.4, 0.08);
      ySpeed = Constants.SwerveConstants.joysickValue(baseJoystick.getRawAxis(0)*0.4, 0.08);
      zSpeed = Constants.SwerveConstants.joysickValue(baseJoystick.getRawAxis(4)*0.4, 0.08);
    }
    else{
      xSpeed = -Constants.SwerveConstants.joysickValue(baseJoystick.getRawAxis(1), 0.08)*0.2;
      ySpeed = -Constants.SwerveConstants.joysickValue(baseJoystick.getRawAxis(0), 0.08)*0.2;
      zSpeed = Constants.SwerveConstants.joysickValue(baseJoystick.getRawAxis(4), 0.08)*0.8;
    }
    swerveSubsystem.drive(xSpeed, ySpeed, zSpeed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
