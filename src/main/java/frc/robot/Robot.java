// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


// import java.nio.file.Paths;

// import edu.wpi.first.wpilibj.GenericHID;
// import edu.wpi.first.wpilibj.PowerDistribution;
// import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkMaxRelativeEncoder;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private double uptime;
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "MyAuto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private Timer auto;
  
  private DifferentialDrive m_myRobot;
  private CANSparkMax m_frontLeft;
  private CANSparkMax m_frontRight;
  private CANSparkMax m_rearLeft;
  private CANSparkMax m_rearRight;

 
  //motors for elevator and intake
  private PWMVictorSPX m_elevator;
  private PWMVictorSPX m_intake;
  
  //climb motor
  private PWMVictorSPX m_climb;

  private MotorControllerGroup m_left;
  //m_left needs to be positive to go forward

  private MotorControllerGroup m_right;
  //m_right needs to be negative to go forward

  
  private final XboxController c_driveController = new XboxController(0);
  private final XboxController c_stuffController = new XboxController(1);

  //private DifferentialDriveOdometry m_odometry;

  AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  SlewRateLimiter BallAndChain = new SlewRateLimiter(1);
  SlewRateLimiter BallAndChain2 = new SlewRateLimiter(1);

  
  Rotation2d rotation;

  double yaw;
  double speed2;
  double voltage;

  double wheelCircumference = 6*Math.PI;
  double leftPreviousPos=0;
  double rightPreviousPos=0;

  boolean backwards;
  boolean hasLimiter;
  boolean spitting;

  String ally; 

  //private PowerDistribution m_pdp;

  //Odometry mess
   // RelativeEncoder frontLeftEncoder = m_frontLeft.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    // RelativeEncoder rearLeftEncoder = m_rearLeft.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
 //   RelativeEncoder frontRightEncoder = m_frontRight.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    // RelativeEncoder rearRightEncoder = m_rearRight.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    

  
  








  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    ally = DriverStation.getAlliance().toString();

    yaw = Math.toRadians(m_gyro.getYaw());
    rotation = new Rotation2d(yaw);

    //m_odometry = new DifferentialDriveOdometry(rotation);
    //m_odometry = new DifferentialDriveOdometry(rotation, new Pose2d(0, 0, new Rotation2d()));
    
    // RelativeEncoder frontLeftEncoder = m_frontLeft.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    // RelativeEncoder rearLeftEncoder = m_rearLeft.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    // RelativeEncoder frontRightEncoder = m_frontRight.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    // RelativeEncoder rearRightEncoder = m_rearRight.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);


    // frontLeftEncoder.setPosition(0);
    // frontRightEncoder.setPosition(0);



    

    auto = new Timer();
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    //SmartDashboard.putData("Auto choices", m_chooser);

    //drive motors
    m_frontLeft = new CANSparkMax(4, MotorType.kBrushless);
    m_frontRight = new CANSparkMax(2, MotorType.kBrushless);
    m_rearLeft = new CANSparkMax(3, MotorType.kBrushless);
    m_rearRight = new CANSparkMax(5, MotorType.kBrushless);
    
    m_frontLeft.restoreFactoryDefaults();
    m_frontRight.restoreFactoryDefaults();
    m_rearLeft.restoreFactoryDefaults();
    m_rearRight.restoreFactoryDefaults();

    m_left = new MotorControllerGroup(m_frontLeft, m_rearLeft);
    m_right = new MotorControllerGroup(m_frontRight, m_rearRight);
    m_left.setInverted(true);
    m_myRobot = new DifferentialDrive(m_left, m_right);

    
    //Stuff
    m_intake = new PWMVictorSPX(2);
    m_elevator = new PWMVictorSPX(4);
    m_climb = new PWMVictorSPX(3);

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);

    // m_frontLeft.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42).getVelocity();

    //Booleans  
    backwards = false;
    hasLimiter = false;
    spitting = false;
    
    System.out.println(ally);
    
    //m_pdp = new PowerDistribution(0, ModuleType.kCTRE);


    

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
    
    //voltage = m_pdp.getVoltage();
    uptime = Timer.getFPGATimestamp();

    yaw = Math.toRadians(m_gyro.getYaw());
    // rotation = new Rotation2d(yaw);
     
    SmartDashboard.putNumber("Uptime", uptime);
    // SmartDashboard.putNumber("Front Left Motor RPM", m_frontLeft.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42).getVelocity());
    // SmartDashboard.putNumber("Front Right Motor RPM", m_frontRight.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42).getVelocity());
    // SmartDashboard.putNumber("Rear Left Motor RPM", m_rearLeft.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42).getVelocity());
    // SmartDashboard.putNumber("Rear Right Motor RPM", m_rearRight.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42).getVelocity());
    // SmartDashboard.putBoolean("Is Tank Drive (LJ)", isTankDrive);
    // SmartDashboard.putBoolean("Is Trigger Happy? (RB)", triggerHappy);
    // SmartDashboard.putNumber("Total Voltage", voltage);
    // SmartDashboard.putBoolean("Is sucking? (LB)", sucking);
    // SmartDashboard.putBoolean("Is doing your mom?", doingYourMom);
    // SmartDashboard.putBoolean("Is blue?", isBlue);

    //Trigger values
    
    
    

    
    //leftDistanceTurned = frontLeftEncoder.getPosition()*wheelCircumference-leftPreviousPos;
   // leftPreviousPos = frontLeftEncoder.getPosition()*wheelCircumference;
    // rightDistanceTurned = frontRightEncoder.getPosition()*wheelCircumference-rightPreviousPos;
   // rightPreviousPos = frontRightEncoder.getPosition()*wheelCircumference;
    
  //  m_odometry.update(rotation, leftPreviousPos, rightPreviousPos);



  }

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
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
    switch (m_autoSelected) {
      case kCustomAuto:
      
        // Put custom auto code here
        auto.start();

        

        auto.stop();
        break;
      case kDefaultAuto:
      default:
              // Put default auto code here

        auto.start();

        while(auto.get()<2.5){
          m_left.set(0.25);
          m_right.set(0.25);
        }
        m_left.set(0);
        m_right.set(0);
        while(auto.get()>=2.5 && auto.get()<6){
          m_elevator.set(-75);
          m_intake.set(-1);
        }
        
        m_elevator.set(0);
        m_intake.set(0);
        auto.stop();
        break;
      
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //Drive
    //boolean switchers
   
    if(c_driveController.getRightBumperPressed()){
      backwards = !backwards;
    }
    if(c_driveController.getLeftStickButtonPressed()){
      hasLimiter = !hasLimiter;
    }
    if(c_stuffController.getRightBumperPressed()){
      spitting = !spitting;
    }

    // Drive options
    if(hasLimiter){
      if (backwards) {
        m_myRobot.tankDrive(-BallAndChain.calculate(c_driveController.getRightY()), -BallAndChain2.calculate(c_driveController.getLeftY()));
      } else {
        m_myRobot.tankDrive(BallAndChain.calculate(c_driveController.getLeftY()), BallAndChain2.calculate(c_driveController.getRightY()));
      }
    } else {
      if (backwards) {
        m_myRobot.tankDrive(-c_driveController.getRightY(), -c_driveController.getLeftY());
      } else {
        m_myRobot.tankDrive(c_driveController.getLeftY(), c_driveController.getRightY());
      }
    }
    
    
    // Elevator/Intake
    if (c_stuffController.getBButton()){
      if(spitting){
        m_elevator.set(-0.75);
        m_intake.set(-1);
      } else {
        m_elevator.set(0.75);
        m_intake.set(1);
      }
    } else {
      m_elevator.set(0);
      m_intake.set(0);
    }

    // Climber
    m_climb.set(-c_stuffController.getLeftY());
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    c_driveController.setRumble(RumbleType.kLeftRumble, 0.0);
    c_driveController.setRumble(RumbleType.kRightRumble, 0.0);
    
  }  

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    //m_myRobot.tankDrive(-m_controller.getLeftY(), m_controller.getRightY());
  }
}
