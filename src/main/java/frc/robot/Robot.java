/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.OI;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Timer;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;


  
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.networktables.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  WPI_VictorSPX _leftMaster = new WPI_VictorSPX(1);
  WPI_VictorSPX _rightMaster = new WPI_VictorSPX(3);
  WPI_VictorSPX  _leftFollow = new WPI_VictorSPX (2);
  WPI_VictorSPX  _rightFollow = new WPI_VictorSPX (4);
  DifferentialDrive _drive = new DifferentialDrive(_leftMaster, _rightMaster);
  private boolean driverVision, tapeVision, cargoVision, cargoSeen, tapeSeen   ;
  private NetworkTableEntry tapeDetected, cargoDetected, tapeYaw, cargoYaw, videoTimestamp, driveWanted,tapeWanted,cargoWanted;
  private double targetAngle;
  NetworkTableInstance instance;
  NetworkTable chickenVision;

  public static ExampleSubsystem m_subsystem = new ExampleSubsystem();
  public static OI m_oi;
  AHRS ahrs;
    Joystick stick;
    Joystick stick2;
    boolean autoBalanceXMode;
    boolean autoBalanceYMode;
    static final double kOffBalanceAngleThresholdDegrees = 10;
    static final double kOonBalanceAngleThresholdDegrees  = 5;
  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    stick = new Joystick(0);
    stick2 = new Joystick(1);
    try {
			/***********************************************************************
			 * navX-MXP:
			 * - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB.            
			 * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * navX-Micro:
			 * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
			 * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * Multiple navX-model devices on a single robot are supported.
			 ************************************************************************/
            ahrs = new AHRS(SPI.Port.kMXP); 
        } catch (RuntimeException ex ) {
            DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
        }
        System.out.println("Starting Navx");
        _leftMaster.configFactoryDefault();
        _rightMaster.configFactoryDefault();
        _leftFollow.configFactoryDefault();
        _rightFollow.configFactoryDefault();
        
        _leftFollow.follow(_leftMaster);
        _rightFollow.follow(_rightMaster);
        
        _leftMaster.setInverted(false); // <<<<<< Adjust this until robot drives forward when stick is forward
        _rightMaster.setInverted(true); // <<<<<< Adjust this until robot drives forward when stick is forward
        _leftFollow.setInverted(InvertType.FollowMaster);
        _rightFollow.setInverted(InvertType.FollowMaster);
        _drive.setRightSideInverted(false); // do not change this

        instance = NetworkTableInstance.getDefault();
 
        chickenVision = instance.getTable("ChickenVision");
 
        tapeDetected = chickenVision.getEntry("tapeDetected");
        cargoDetected = chickenVision.getEntry("cargoDetected");
        tapeYaw = chickenVision.getEntry("tapeYaw");
        cargoYaw = chickenVision.getEntry("cargoYaw");
 
        driveWanted = chickenVision.getEntry("Driver");
        tapeWanted = chickenVision.getEntry("Tape");
        cargoWanted = chickenVision.getEntry("Cargo");
 
        videoTimestamp = chickenVision.getEntry("VideoTimestamp");
 
        tapeVision = cargoVision = false;
        driverVision = true;
        targetAngle = 0;
    m_oi = new OI();
    m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
           
           
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    double kP = 1.2;
    double frontRate            = .4;
    double pitchRate            = 0;
    double rearRate            = .6;
    double yAxisRate            = stick2.getY();
    double pitchAngleDegrees    = ahrs.getPitch();
    double rollAngleDegrees     = ahrs.getRoll();
    double forward = stick.getY();
    double turn = stick.getTwist();
    boolean tapeDesired = stick.getRawButton(1);
    boolean cargoDesired = stick.getRawButton(2);

    // If button 1 is pressed, then it will track cargo
    if (cargoDesired) {
 
      driveWanted.setBoolean(false);
      tapeWanted.setBoolean(false);
      cargoWanted.setBoolean(true);
      cargoSeen = cargoDetected.getBoolean(false);

      if (cargoSeen)
          targetAngle = cargoYaw.getDouble(0);
      else
          targetAngle = 0;

  } else if (tapeDesired) {


      driveWanted.setBoolean(false);
      tapeWanted.setBoolean(true);
      cargoWanted.setBoolean(false);
      // Checks if vision sees cargo or vision targets. This may not get called unless
      // cargo vision detected
      tapeSeen = tapeDetected.getBoolean(false);

      if (tapeSeen)
          targetAngle = tapeYaw.getDouble(0);
      else
          targetAngle = 0;

  } else {


      driveWanted.setBoolean(true);
      tapeWanted.setBoolean(false);
      cargoWanted.setBoolean(false);

      targetAngle = 0;

  }
  // Limit output to 0.3

    if ( !autoBalanceXMode && 
         (Math.abs(pitchAngleDegrees) >= 
          Math.abs(kOffBalanceAngleThresholdDegrees))) {
        autoBalanceXMode = true;
    }
    else if ( autoBalanceXMode && 
              (Math.abs(pitchAngleDegrees) <= 
               Math.abs(kOonBalanceAngleThresholdDegrees))) {
        autoBalanceXMode = false;
    }
    
    // Control drive system automatically, 
    // driving in reverse direction of pitch/roll angle,
    // with a magnitude based upon the angle
    
    if ( autoBalanceXMode ) {
        double pitchAngleRadians = pitchAngleDegrees * (Math.PI / 180.0);
        pitchRate = Math.sin(pitchAngleRadians) * -1;
    }
    
    try {      
      if (pitchRate < 0) {
        frontRate = frontRate*pitchRate*1.5;
      } else if (pitchRate > 0) {
        rearRate = rearRate*pitchRate*1.5;
      }
      //System.out.println("front: "+frontRate);
      //System.out.println("rear: "+rearRate);
       // myRobot.driveCartesian(xAxisRate, yAxisRate, stick.getTwist(),0);
    } catch( RuntimeException ex ) {
        String err_string = "Drive system error:  " + ex.getMessage();
        DriverStation.reportError(err_string, true);
    }

    //ouble liftup = m_oi._operator.getY();
    //double driveLift = m_oi._game2.getRawAxis(5);

    //_leftMaster.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, turn*.35);
    //_rightMaster.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn*.35);
    
    // Limit output to 0.3
    double output = limitOutput(-kP * targetAngle, 0.4);
    System.out.println(targetAngle);
    if (cargoDesired || tapeDesired)    
      _drive.arcadeDrive(-forward, -output);
    else
       _drive.arcadeDrive(-forward, turn);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  public double limitOutput(double number, double maxOutput) {
    if (number > 1.0) {
        number = 1.0;
    }
    if (number < -1.0) {
        number = -1.0;
    }

    if (number > maxOutput) {
        return maxOutput;
    }
    if (number < -maxOutput) {
        return -maxOutput;
    }

    return number;
  }
}
