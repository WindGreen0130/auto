// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoSubsystem extends SubsystemBase {
  /** Creates a new AutoSubsystem. */
    boolean in = false;
    boolean shoot = false;
  public AutoSubsystem() {

  }
  public void in(){
    in = true;
  }
  public void shoot(){
    shoot = true;
  }
  public void falsevery(){
    in = false;
    shoot=false;

  }
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("in", in);
    SmartDashboard.putBoolean("shoot", shoot);
    // This method will be called once per scheduler run
  }
}
