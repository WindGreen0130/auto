// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.ManualDrive;
import frc.robot.subsystems.AutoSubsystem;
// import frc.robot.subsystems.AutoSubsystem;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem _swervesubsystem = new SwerveSubsystem();
  private final AutoSubsystem _autosubsystem = new AutoSubsystem();
  public ManualDrive manualDrive = new ManualDrive(_swervesubsystem);
  public AutoCommand autocommand = new AutoCommand(_swervesubsystem);
  // public AutoSubsystem autoSubsystem = new AutoSubsystem();
  public static final Joystick baseJoystick = new Joystick(0);
  private final SendableChooser<Command> autoChooser;
  // private SendableChooser<Command> m_Chooser = new SendableChooser<>();
  public RobotContainer() {
    NamedCommands.registerCommand("NoteIn", Commands.run(()->{
      _autosubsystem.inTrue();
    }, _autosubsystem).withTimeout(0.5));
    NamedCommands.registerCommand("NoteShoot", Commands.run(()->{
      _autosubsystem.shootTrue();
    }, _autosubsystem).withTimeout(0.5));
    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto mode", autoChooser);
  }

  private void configureBindings() {
    }
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    _swervesubsystem.falseChiu();
    _autosubsystem.inFalse();
    _autosubsystem.shootFalse();
    // autoSubsystem.falsevery();
    // _swervesubsystem.setPose(inintialPose2d);
    return autoChooser.getSelected();
  }
}
