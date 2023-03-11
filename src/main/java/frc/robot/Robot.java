// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.armPosition;
import frc.robot.commands.autoCommands.Next2HumanCommand;
import frc.robot.subsystems.CartridgeSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.armSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static CTREConfigs ctreConfigs;
  public armSubsystem armsubsystem;
  private Command m_autonomousCommand;
  private CartridgeSubsystem shootingSubsystem;
  private RobotContainer m_robotContainer;
  private static final String centerAuto = "center";
  private static final String test = "test";
  private static final String ballanceRampaAtuo = "ballance Rampa";
  private static final String justShootAtuo = "do nofing";
  private static final String Next2Human = "NextToHoman";
  private static final String FarFromHumanRampa = "Far from human + rampe";
  //private static final String rAuto = "next to human & 1 cube";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    ctreConfigs = new CTREConfigs();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    //m_chooser.setDefaultOption("", centerToRampaAuto);
    m_chooser.setDefaultOption("just shoot", justShootAtuo);
    m_chooser.addOption("Center", centerAuto);
    m_chooser.addOption("Next to Human", Next2Human);
    m_chooser.addOption("Far from human + rampe", FarFromHumanRampa);
    m_chooser.addOption("test", test);
    SmartDashboard.putData("Auto choices", m_chooser);
    m_robotContainer = new RobotContainer();
    armsubsystem = m_robotContainer.getM_ArmSubsystem();
    CameraServer.startAutomaticCapture();

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    m_autoSelected = m_chooser.getSelected();
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_robotContainer.getM_ArmSubsystem().getTab().putInDashboard("choose", m_chooser.getSelected(),true);
    if (m_chooser.getSelected().equals(centerAuto)) m_autonomousCommand = m_robotContainer.getCenterAtuo();
    else if (m_chooser.getSelected().equals(Next2Human)) m_autonomousCommand = m_robotContainer.getNext2Human();
    else if (m_chooser.getSelected().equals(test)) m_autonomousCommand = m_robotContainer.getTest();
    else if (m_chooser.getSelected().equals(FarFromHumanRampa)) m_autonomousCommand = m_robotContainer.getFarFromHumanRampa();
    else m_autonomousCommand = m_robotContainer.getTest();
    
    //m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // armSubsystem.ArmgMotor.setIdleMode(IdleMode.kBrake);

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    m_robotContainer.getS_Swerve().zeroGyro();
   
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    // armSubsystem.ArmgMotor.setIdleMode(IdleMode.kBrake);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  } 

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
