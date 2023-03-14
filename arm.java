import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  
  private WPI_TalonFX shoulder;
  private WPI_TalonFX elevator;
  private WPI_TalonFX wrist;
  private WPI_TalonFX wrist2;
  private DutyCycleEncoder shoulderAbsoluteEncoder;
  private DigitalInput bottomLimit;
  
  private static Arm instance;
  
  public Arm() {
    shoulder = new WPI_TalonFX(Constants.shoulderID);
    elevator = new WPI_TalonFX(Constants.elevatorID);
    wrist = new WPI_TalonFX(Constants.wristID);
    wrist2 = new WPI_TalonFX(Constants.wrist2ID);
    shoulderAbsoluteEncoder = new DutyCycleEncoder(Constants.shoulderAbsoluteEncoderPort);
    bottomLimit = new DigitalInput(Constants.bottomLimitPort);
    
    // Set motor types and feedback devices
    elevator.configFactoryDefault();
    elevator.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    wrist.configFactoryDefault();
    wrist.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    wrist2.configFactoryDefault();
    wrist2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    shoulder.configFactoryDefault();

shoulderAbsoluteEncoder.configFactoryDefault();
shoulderAbsoluteEncoder.configSensorDirection(false);
shoulderAbsoluteEncoder.configMagnetOffset(0);
    
    // Set PIDF constants
   
    elevator.config_kP(0, Constants.elevatorKP);
    elevator.config_kI(0, Constants.elevatorKI);
    elevator.config_kD(0, Constants.elevatorKD);
    wrist.config_kP(0, Constants.wristKP);
    wrist.config_kI(0, Constants.wristKI);
    wrist.config_kD(0, Constants.wristKD);
    wrist2.config_kP(0, Constants.wrist2KP);
    wrist2.config_kI(0, Constants.wrist2KI);
    wrist2.config_kD(0, Constants.wrist2KD);
    
    // Set soft limits and enable them
   
    elevator.configForwardSoftLimitThreshold(degreesToTicks(Constants.elevatorForwardSoftLimit));
    wrist.configForwardSoftLimitThreshold(degreesToTicks(Constants.wristForwardSoftLimit));
    wrist2.configForwardSoftLimitThreshold(degreesToTicks(Constants.wrist2ForwardSoftLimit));
    elevator.configReverseSoftLimitThreshold(degreesToTicks(Constants.elevatorReverseSoftLimit));
    wrist.configReverseSoftLimitThreshold(degreesToTicks(Constants.wristReverseSoftLimit));
    wrist2.configReverseSoftLimitThreshold(degreesToTicks(Constants.wrist2ReverseSoftLimit));
    elevator.configForwardSoftLimitEnable(true);
    wrist.configForwardSoftLimitEnable(true);
    wrist2.configForwardSoftLimitEnable(true);
    elevator.configReverseSoftLimitEnable(true);
    wrist.configReverseSoftLimitEnable(true);
    wrist2.configReverseSoftLimitEnable(true);
    
    // Set neutral mode for motors
    shoulder.setNeutralMode(NeutralMode.Coast);
    elevator.setNeutralMode(NeutralMode.Brake);
    wrist.setNeutralMode(NeutralMode.Brake);
    wrist2.setNeutralMode(NeutralMode.Brake);
  }
  
  public static synchronized Arm getInstance() {
    if (instance == null) {
      instance = new Arm();
    }
    return instance;
  }
  
  public double getShoulderPosition() {
    return (shoulderAbsoluteEncoder.getSelectedSensorPosition());
  }
  
  public double getElevatorPosition() {
    return ticksToDegrees(elevator.getSelectedSensorPosition());
  }
  
  public double getWristPosition() {
    return ticksToDegrees(wrist.getSelectedSensorPosition());
  }
  
  public double getWrist2Position() {
    return ticksToDegrees(wrist2.getSelectedSensorPosition());
  }
  
  public boolean getBottomLimit() {
    return bottomLimit.get();
  }
  
  public void setShoulderSpeed(double speed) {
    shoulder.set(ControlMode.PercentOutput, speed);
  }

  public void setElevatorSpeed(double speed) {
    elevator.set(ControlMode.PercentOutput, speed);
  }
  
  public void setWristSpeed(double speed) {
    wrist.set(ControlMode.PercentOutput, speed);
  }
  
  public void setWrist2Speed(double speed) {
    wrist2.set(ControlMode.PercentOutput, -speed);
  }
  
  public void setArmPosition( double elevatorDegrees, double wristDegrees, double wrist2Degrees) {
   // shoulder.set(ControlMode.Position, degreesToTicks(shoulderDegrees));
    elevator.set(ControlMode.Position, degreesToTicks(elevatorDegrees));
    wrist.set(ControlMode.Position, degreesToTicks(wristDegrees));
    wrist2.set(ControlMode.Position, degreesToTicks(wrist2Degrees));
  }
  
  public void resetShoulder() {
    shoulder.setSelectedSensorPosition(0);
  }
  
  public void resetElevator() {
    elevator.setSelectedSensorPosition(0);
  }
  
  public void resetWrist() {
    wrist.setSelectedSensorPosition(0);
  }
  
  public void resetWrist2() {
    wrist2.setSelectedSensorPosition(0);
  }
  
  public double ticksToDegrees(double ticks) {
   return ticks / Constants.Falcon_Ticks_Per_Rev /  360;
  }
  
  public double degreesToTicks(double degrees) {
    return degrees / 360 * Constants.Falcon_Ticks_Per_Rev;
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shoulder Position", shoulderAbsoluteEncoder.get());
    SmartDashboard.putNumber("Elevator Position", getElevatorPosition());
    SmartDashboard.putNumber("Wrist Position", getWristPosition());
    SmartDashboard.putNumber("Wrist2 Position", getWrist2Position());
    
    if (getBottomLimit()) 
      elevator.set(ControlMode.PercentOutput, 0);
    }
  }
  
}

// To use these commands in teleop using RobotContainer and Command-based programming, you can create a button on your joystick/gamepad and assign it to the `MoveShoulderCommand` and `MoveElevatorCommand`. 

// Here's an example code snippet in `RobotContainer` class:

// ```
// public class RobotContainer {
//   private final Joystick joystick = new Joystick(Constants.joystickPort);
//   private final Arm arm = new Arm();
//   private final MoveShoulderCommand moveShoulderCommand = new MoveShoulderCommand(arm);
//   private final MoveElevatorCommand moveElevatorCommand = new MoveElevatorCommand(arm);
  
//   public RobotContainer() {
//     configureButtonBindings();
//   }
  
//   private void configureButtonBindings() {
//     final JoystickButton moveShoulderButton = new JoystickButton(joystick, Constants.moveShoulderButton);
//     moveShoulderButton.whenHeld(moveShoulderCommand);
    
//     final JoystickButton moveElevatorButton = new JoystickButton(joystick, Constants.moveElevatorButton);
//     moveElevatorButton.whenHeld(moveElevatorCommand);
//   }
  
//   public Command getAutonomousCommand() {
//     // ...
//   }
  
//   public Command getTeleopCommand() {
//     return null;
//   }
// }
// ```

// Here, we create two `JoystickButton`s and assign them to the `MoveShoulderCommand` and `MoveElevatorCommand` respectively. 

// You can adjust the `Constants.moveShoulderButton` and `Constants.moveElevatorButton` values to match the button numbers on your joystick/gamepad.

// In the `getTeleopCommand` method, you can return `null` since we are using Command-based programming and the commands are already assigned to buttons.

// In `Robot.java`, you can initialize the `RobotContainer` and the arm subsystem as follows:

// ```
// public class Robot extends TimedRobot {
//   private RobotContainer robotContainer;
  
//   @Override
//   public void robotInit() {
//     robotContainer = new RobotContainer();
//     arm = new Arm();
//   }
  
//   @Override
//   public void robotPeriodic() {
//     // ...
//   }

//   @Override
//   public void autonomousInit() {
//     // ...
//   }

//   @Override
//   public void autonomousPeriodic() {
//     // ...
//   }

//   @Override
//   public void teleopInit() {
//     // ...
//   }

//   @Override
//   public void teleopPeriodic() {
//     // ...
//   }

//   @Override
//   public void disabledInit() {
//     // ...
//   }

//   @Override
//   public void disabledPeriodic() {
//     // ...
//   }
// }
// ```

// That's it! Now, when you hold down the assigned joystick/gamepad button, the `MoveShoulderCommand` or `MoveElevatorCommand` will be executed.
