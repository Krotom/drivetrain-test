package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;

public class Robot extends TimedRobot {
  private int drive_mode;
  private final SendableChooser<Integer> driveChooser = new SendableChooser<>();

  private final XboxController m_driverController = new XboxController(0);

  private final SparkMax m_leftLeader  = new SparkMax(2, SparkMax.MotorType.kBrushed);
  private final SparkMax m_leftFollower = new SparkMax(3, SparkMax.MotorType.kBrushed);

  private final SparkMax m_rightLeader = new SparkMax(0, SparkMax.MotorType.kBrushed);
  private final SparkMax m_rightFollower = new SparkMax(1, SparkMax.MotorType.kBrushed);

  private final PIDController turnPID =
    new PIDController(1.5, 0.0, 0.2);

  private final PIDController drivePID =
    new PIDController(1.0, 0.0, 0.0);

  private DifferentialDrive m_drive;

  private DifferentialDrivetrainSim m_driveSim;
  private Field2d m_fieldSim;

  Pose2d m_zeroZero = new Pose2d(0, 0, new Rotation2d());
  Pose2d m_fieldMiddle = new Pose2d(4.5, 4.0, new Rotation2d());
  Pose2d m_coralGet = new Pose2d(1.0, 7.0, new Rotation2d());
  Pose2d m_coralShoot = new Pose2d(2.8, 4.0, new Rotation2d());

  Pose2d[] autoTargets = new Pose2d[] {
    m_coralShoot,
    m_coralGet,
    m_coralShoot,
    m_coralGet,
    m_coralShoot,
    m_zeroZero
};

int autoIndex;

  public Robot() {
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    SparkMaxConfig rightConfig = new SparkMaxConfig();
    SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
    SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();

    leftFollowerConfig.follow(m_leftLeader);
    rightFollowerConfig.follow(m_rightLeader);

    m_leftLeader.configure(leftConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    m_rightLeader.configure(rightConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    m_leftFollower.configure(leftFollowerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    m_rightFollower.configure(rightFollowerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    m_drive = new DifferentialDrive(m_leftLeader, m_rightLeader);

    m_driveSim = new DifferentialDrivetrainSim(
        DCMotor.getCIM(2),
        10.71,
        7.5,
        50.0,
        Units.inchesToMeters(3),
        Units.inchesToMeters(28),
        VecBuilder.fill(0.005, 0.005, 0.001, 0.05, 0.05, 0.005, 0.005)
    );

    m_fieldSim = new Field2d();

    driveChooser.addOption("Tank Drive", 0);
    driveChooser.setDefaultOption("Arcade Drive", 1);

    Shuffleboard.getTab("Default")
        .add("Drive Mode", driveChooser)
        .withWidget(BuiltInWidgets.kSplitButtonChooser);

    Shuffleboard.getTab("Default")
        .add("Field View", m_fieldSim);
  }

  @Override
  public void robotPeriodic() {
    Integer choice = driveChooser.getSelected();
    if (choice != null) {
      drive_mode = choice;
    }
  }

  @Override
  public void autonomousInit() {
    autoIndex = 0;
  }

  @Override
  public void autonomousPeriodic() {
    if (autoIndex >= autoTargets.length) {
      m_drive.arcadeDrive(0, 0);
      return;
    }

  Pose2d current = autoTargets[autoIndex];

  boolean reached = false;

  if (autoIndex == autoTargets.length - 1) {
      reached = go2Target(current, true);
  } else {
      reached = go2Target(current);
  }

  if (reached) {
      autoIndex++;
  }
  }

  @Override
  public void teleopPeriodic() {
    double left = -m_driverController.getLeftY();
    double rightY = -m_driverController.getRightY();
    double rightX = -m_driverController.getRightX();
    if (drive_mode == 0) {
      m_drive.tankDrive(left, rightY);
    } else if (drive_mode == 1) {
      m_drive.arcadeDrive(left, rightX);
    }

    if(m_driverController.getLeftBumperButton()){
      go2Target(m_coralGet);
    }

    if(m_driverController.getRightBumperButton()){
      go2Target(m_coralShoot);
    }
  }

  @Override
  public void simulationPeriodic() {
    double leftVolts  = m_leftLeader.get()  * RobotController.getBatteryVoltage();
    double rightVolts = m_rightLeader.get() * RobotController.getBatteryVoltage();

    m_driveSim.setInputs(leftVolts, rightVolts);
    m_driveSim.update(0.02);

    m_fieldSim.setRobotPose(m_driveSim.getPose());
  }

  public boolean go2Target(Pose2d target) {return go2Target(target, false);}

  public boolean go2Target(Pose2d target, boolean angleFromTarget) {
    Pose2d current = m_driveSim.getPose();

    double errorX = target.getX() - current.getX();
    double errorY = target.getY() - current.getY();
    double middleErrorX = m_fieldMiddle.getX() - current.getX();
    double middleErrorY = m_fieldMiddle.getY() - current.getY();
    double distance = Math.hypot(errorX, errorY);

    double robotHeading = current.getRotation().getRadians();
    double angleToTarget = Math.atan2(errorY, errorX);
    double angleToMiddle = Math.atan2(middleErrorY, middleErrorX);

    double headingError = -MathUtil.angleModulus(angleToTarget - robotHeading);

    double finalHeadingError;

    if (angleFromTarget) {
      finalHeadingError = -MathUtil.angleModulus(target.getRotation().getRadians() - robotHeading);
    } else {
      finalHeadingError = -MathUtil.angleModulus(angleToMiddle - robotHeading);
    }

    double turnOut;
    double driveOut;

    if (distance > 0.1) {
        turnOut = turnPID.calculate(headingError, 0);
        driveOut = -drivePID.calculate(distance, 0);
    } else{
        turnOut = turnPID.calculate(finalHeadingError, 0);
        driveOut = 0;
    }

    m_drive.arcadeDrive(driveOut, turnOut);

    return distance < 0.1 && Math.abs(finalHeadingError) < 0.1;
  }
}
