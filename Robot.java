// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*; //CAN Motors hooked into Phoenix CTRE
// import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax; //Spark Motor
import com.revrobotics.CANSparkMaxLowLevel.MotorType; //Also for Spark Motor
import edu.wpi.first.wpilibj.ADXRS450_Gyro; //gyro
import edu.wpi.first.wpilibj.Encoder; //Encoder for right+leftLeader
import edu.wpi.first.wpilibj.Joystick; //For controller
import edu.wpi.first.wpilibj.SpeedController; //Necessary for Diff. Drive
import edu.wpi.first.wpilibj.SpeedControllerGroup; //Necessary for Diff. Drive
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer; //FOR BACKUP AUTONOMOUS
import edu.wpi.first.wpilibj.drive.DifferentialDrive; //Diff. Drive
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry; //For odometer
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds; //For getting speed from the Talons
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab; //Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import com.ctre.phoenix.motorcontrol.ControlMode; //for MotionMagic
import edu.wpi.first.wpilibj.DriverStation;



/**
 * READ:
 * Ultimate project for testing purposes. Put all new changes in this class.
 * BUILT FOR ROBOT USE!
 * FOR TESTBENCH TESTING, USE TESTBENCH PROJECT!
 * Keep commented out code to a minimum please
 */

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final SendableChooser<String> m_position = new SendableChooser<>();

  // MOTOR+ENCODER SETUP

  WPI_TalonSRX leftLeader = new WPI_TalonSRX(5);
  WPI_TalonSRX rightLeader = new WPI_TalonSRX(15);
  WPI_VictorSPX leftFollower = new WPI_VictorSPX(0);
  WPI_VictorSPX rightFollower = new WPI_VictorSPX(3);
  CANSparkMax intakeMotor = new CANSparkMax(1, MotorType.kBrushless);
  WPI_VictorSPX shooterLeader = new WPI_VictorSPX(7); 
  WPI_VictorSPX shooterFollower = new WPI_VictorSPX(13); 
  WPI_VictorSPX shooterIndex = new WPI_VictorSPX(11);
  
  ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  DifferentialDriveOdometry odometer;
  

   //Encoder rightEncoder = new Encoder(2,3); //TESTBENCH
   Encoder shooterEncoder = new Encoder(0,1); 

  //Shuffleboard
  ShuffleboardTab tab = Shuffleboard.getTab("RobotUltimateTab");


   //DIFFERENTIAL DRIVE
   //This is for the 4 motor drivetrain
  private final SpeedController leftMotorGroup =
  new SpeedControllerGroup(leftFollower, leftLeader);
  
  private final SpeedController rightMotorGroup =
  new SpeedControllerGroup(rightFollower, rightLeader);

  DifferentialDrive drive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);

   //JOYSTICK
   private final Joystick joystick = new Joystick(0);

   /**************************************************
   * Button Mapping Chart
  *   Axis - 
  *          0 = up/down on left stick
  *          1 = right/left on left stick
  *          2 = up/down on right stick //testing
  *          3 = right/left on right stick //testing
  *   Button - 
  *          1 = A
  *          2 = B
  *          3 = X
  *          4 = Y
  *          5 = L1
  *          6 = R1
  *          7 = back
  *          8 = start
  *****************************************************/

   //OTHER VARIABLES (kP, etc.)
   
   //For PID
   final double kP = 0.325;
   final double kI = 0.5;
   final double kD = 0.2;
   final double iLimit = 1;
   double dt = 0; 
   double sensorPosition = 0;
   double error = 0;
   double errorRate = 0;
   double setpoint = 0;
   double errorSum = 0;
   double lastTimestamp = 0;
   double lastError = 0;
    double outputSpeed = 0;

   double wheelCircumference = 6.0 * Math.PI; //6in diameter * pi
   double kDriveTick2Feet = 1.0/4096.0 * wheelCircumference / 12.0;
   double kDriveTick2FeetAutonomous = 1.0/4096.0 * wheelCircumference / 12.0;

   double xSpeed;
   double zRotation;
   double time;
   double startup;

   double Pidsum = 0;

   //For autonomus cases
   final String game1 = "Game 1 Red Scenario";
   final String pidGame1 = "PID Game 1";
   public String gameData;


  //ADDITIONAL METHODS
  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        leftLeader.getSelectedSensorVelocity() * (10.0 / 4096) * wheelCircumference,
        rightLeader.getSelectedSensorVelocity() * (10.0 / 4096) * wheelCircumference);
  }

  public void PidDrive(double newSetpoint)
  {
    setpoint = newSetpoint;
    // get sensor position
    sensorPosition = ((leftLeader.getSelectedSensorPosition() + rightLeader.getSelectedSensorVelocity()) / 2) * kDriveTick2FeetAutonomous;
    while(sensorPosition <= (setpoint - 0.01)) {
      sensorPosition = ((leftLeader.getSelectedSensorPosition() + rightLeader.getSelectedSensorVelocity()) / 2) * kDriveTick2FeetAutonomous;
      System.out.println(sensorPosition);

    // calculations
    error = setpoint - sensorPosition;
    dt = Timer.getFPGATimestamp() - lastTimestamp;

    if (Math.abs(error) < iLimit) {
      errorSum += error * dt;
    }

    errorRate = (error - lastError) / dt;

    outputSpeed = (kP * error) + (kI * errorSum) + (kD * errorRate);

    // output to motors
     rightLeader.set(outputSpeed); leftLeader.set(-outputSpeed); //maybe negative
    //  rightLeader.set(outputSpeed);      

    // update last- variables
    lastTimestamp = Timer.getFPGATimestamp();
    lastError = error;  
   }

  leftLeader.configFactoryDefault();
  rightLeader.configFactoryDefault();
  leftLeader.setSelectedSensorPosition(0, 0, 10);
  rightLeader.setSelectedSensorPosition(0, 0, 10);

  }

  public void turn(char direction, double degrees, double speed) {
    double currentAngle = gyro.getAngle();
    if (direction == 'R') {
      if(gyro.getAngle() - currentAngle < degrees) {
      leftLeader.set(speed);
      rightLeader.set(speed);
    }
    } else {
      if(Math.abs(gyro.getAngle()) - currentAngle < degrees) {
        leftLeader.set(-speed);
        rightLeader.set(-speed);
      }
    }
  }



  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_position.setDefaultOption("Left", "Left");
    m_position.addOption("Right", "Right");
    m_position.addOption("Middle", "Middle");
    SmartDashboard.putData("Position", m_position);
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    gyro.reset();

    //Put Shuffleboard Readouts Below
    Shuffleboard.selectTab("RobotUltimateTab");
    Shuffleboard.getTab("RobotUltimateTab").add("Pi", 3.14);

    SmartDashboard.putNumber("voltage", leftLeader.getBusVoltage());

    rightLeader.configSelectedFeedbackSensor(TalonSRXFeedbackDevice. CTRE_MagEncoder_Relative, 0, 0);
    leftLeader.configSelectedFeedbackSensor(TalonSRXFeedbackDevice. CTRE_MagEncoder_Relative, 0, 0);
  
    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);
    //shooterFollower.follow(shooterLeader);
  
    initDriveMotors();

    
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    
    
    leftLeader.configFactoryDefault();
    rightLeader.configFactoryDefault();
    rightLeader.setInverted(false); //to be changed, potentially to true
    leftLeader.setInverted(false); //to be changed
    drive.setRightSideInverted(false);
    
    time = Timer.getFPGATimestamp();
    m_autoSelected = game1;

    gyro.reset();

    odometer = new DifferentialDriveOdometry(gyro.getRotation2d());

    rightLeader.setSelectedSensorPosition(0, 0, 10);
    leftLeader.setSelectedSensorPosition(0, 0, 10);

    //PID setup 
    errorSum = 0;
    lastError = 0;
    lastTimestamp = Timer.getFPGATimestamp();    
    
    //Running through
    startup = Timer.getFPGATimestamp();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    //setting the setpoint 
    while (Timer.getFPGATimestamp() - startup < 5.0) {
      PidDrive(8.0);
      turn('R', 26.565, 0.2);
      // PidDrive(6.083);
      // urn('L', 116.565, 0.2);
      // rightLeader.set(0);
    }


    // time = Timer.getFPGATimestamp();
    // switch (m_autoSelected) {
    //   case kCustomAuto:
    //     // Put custom auto code here
    //     break;
    //   case kDefaultAuto:
    //   default:
    //     if(leftEncoder.getDistance() <= 3.0) {
    //       leftMotorGroup.set(0.4);
    //       rightMotorGroup.set(0.4);
    //     }
      
    //     if(gyro.getAngle() <= 90){
    //       leftMotorGroup.set(0.4);
    //       rightMotorGroup.set(-0.4);
    //   }
    //     break;
    //     case game1:
    //       if (time < 3) {
    //         leftMotorGroup.set(0.5);
    //         rightMotorGroup.set(0.5);
    //         intakeMotor.set(0.3);
    //       }
    //     break;
    //     case pidGame1:
        //break;
    //}
}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

    //Don't remove/change
    leftLeader.setSelectedSensorPosition(0);
    rightLeader.setSelectedSensorPosition(0);
    leftLeader.configFactoryDefault();
    rightLeader.configFactoryDefault();
    rightLeader.setInverted(false); 
    leftLeader.setInverted(false); 
    drive.setRightSideInverted(false);

    shooterLeader.setInverted(true);

    shooterEncoder.setDistancePerPulse(1./20.); //DISTANCE PER PULSE SUBJECT TO CHANGE
    
    shooterLeader.configFactoryDefault();
    shooterFollower.configFactoryDefault();


    rightLeader.setSelectedSensorPosition(0, 0, 10);
    leftLeader.setSelectedSensorPosition(0, 0, 10);

    gyro.reset();
    odometer = new DifferentialDriveOdometry(gyro.getRotation2d());

    startup = Timer.getFPGATimestamp();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    time = Timer.getFPGATimestamp();

    xSpeed = joystick.getRawAxis(4) * 0.8; //left right - right joystick
    zRotation = joystick.getRawAxis(1) * 0.8; //forwards backwards - left joystick
  
    
    SmartDashboard.putNumber("Gyro Angle Final", gyro.getAngle());
		
    
    //FOR DRIVE WITHOUT USE OF ENCODERS
    drive.arcadeDrive(xSpeed, zRotation);

    //TO USE INTAKE MOTOR
    if (joystick.getRawButton(1))
   {
    intakeMotor.set(-0.3);
   }
   else if (joystick.getRawButton(2))
   {
    intakeMotor.set(0);
   }

   //Shooter motor
   if (joystick.getRawButton(3)) {
     shooterLeader.set(0.95);
     shooterFollower.set(0.95);
     if (Timer.getFPGATimestamp() - startup > 1) {
       shooterIndex.set(0.4);
     }
   } else if (joystick.getRawButton(4)) {
     shooterLeader.set(0);
     shooterFollower.set(0);
     shooterIndex.set(0);
   }


   
  double speedMotor =  rightLeader.getSelectedSensorVelocity();
  
  SmartDashboard.putNumber("Speed", speedMotor);

  System.out.println(xSpeed);

   

   //Getting angle, position, and distance
   odometer.update(gyro.getRotation2d(), leftLeader.getSelectedSensorVelocity(), rightLeader.getSelectedSensorVelocity());
   
  }

  public void initDriveMotors() {

    rightLeader.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 10); //from 0, 0
    leftLeader.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 10); //from 0, 0
  
    //skip for now unitl we implement motion magic
    //leftLeader.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10);

    leftLeader.configNominalOutputForward(0.0, 10); //minimum percent output
    leftLeader.configNominalOutputReverse(0, 10);
    leftLeader.configPeakOutputForward(1, 10); //max
    leftLeader.configPeakOutputReverse(-1, 10);
    leftLeader.setInverted(true);
    leftLeader.setSensorPhase(true); 
    leftLeader.selectProfileSlot(0, 0);


    rightLeader.configNominalOutputForward(0, 10); //minimum percent output
    rightLeader.configNominalOutputReverse(0, 10);
    rightLeader.configPeakOutputForward(1, 10); //max
    rightLeader.configPeakOutputReverse(-1, 10);
    rightLeader.setInverted(false);
    rightLeader.setSensorPhase(true); 
    rightLeader.selectProfileSlot(0, 0);

	
  }


  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
