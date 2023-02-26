
// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.RobotButtons;
// import frc.robot.subsystems.Swerve;
// import frc.robot.autos.TestAuto;
// import frc.robot.autos.centerToRampa;
// import frc.robot.autos.next2human;
// import frc.robot.commands.timercommand.timeSetPointCollectCommand;
// import frc.robot.subsystems.CollectSubsystem;
// import frc.robot.subsystems.ShootingSubsystem;
// import frc.robot.subsystems.armSubsystem;
// import frc.robot.commands.timercommand.TimerArmPosition;
// import frc.robot.commands.timercommand.TimerGripperCommand;
// import frc.robot.commands.timercommand.collectAtuoCommand;
// import frc.robot.commands.timercommand.moveInParallel;
// import frc.robot.commands.timercommand.openInParallel;
// import frc.robot.subsystems.collectWheels;


// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class atuo2 extends SequentialCommandGroup {
//   /** Creates a new atuo1. */
//   private final Swerve s_Swerve;
//   private ShootingSubsystem ShootingSubsystem;
//   private CollectSubsystem collectSubsystem;
//   private armSubsystem armSubsystem;
//   private openInParallel openInParallel;
//   private TimerGripperCommand timerGripperCommand;
//   private moveInParallel moveInParallel;
//   private collectAtuoCommand collectAtuoCommand;
//   private collectWheels collectWheels;
//   private TimerArmPosition timerArmPosition;
//   private timeSetPointCollectCommand timeSetPointCollectCommand;

  
//   public atuo2(Swerve swerve,openInParallel openInParallel,
//   timeSetPointCollectCommand timeSetPointCollectCommand,
//   TimerArmPosition timerArmPosition,
//   collectWheels collectWheels,
//   collectAtuoCommand collectAtuoCommand,
//   next2human next2human,
//   armSubsystem armSubsystem,
//   TimerGripperCommand timerGripperCommand,
//   CollectSubsystem collectSubsystem,
//   moveInParallel moveInParallel,
//   ShootingSubsystem ShootingSubsystem
// ) {
//     this.s_Swerve = swerve;
//     this.timeSetPointCollectCommand = timeSetPointCollectCommand; 
//     this.timerArmPosition = timerArmPosition;
//     this.collectAtuoCommand = collectAtuoCommand;
//     this.collectSubsystem = collectSubsystem;
//     this.ShootingSubsystem = ShootingSubsystem;
//     this.armSubsystem = armSubsystem;
//     this.openInParallel = openInParallel;
//     this.moveInParallel = moveInParallel;
//     this.timerGripperCommand = timerGripperCommand;
//     this.collectWheels = collectWheels;
  
//     // Add your commands in the addCommands() call, e.g.
//     // addCommands(new FooCommand(), new BarCommand());
    
  
//     addCommands(new resetCommand(ShootingSubsystem, collectSubsystem, armSubsystem),
//     openInParallel,
//     timerGripperCommand,
//     moveInParallel);
//   }
// }