
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Arm extends SubsystemBase {
  
 private WPI_TalonFX wrist;
 private WPI_TalonFX elevator;
 private WPI_TalonFX shoulder;
 public static Arm currentInstance;
 private DigitalInput topLimit;
 private DigitalInput bottomLimit;
//--------------------------------------------------------------------------------------------------------------------------------

  public Arm() {
    shoulder = = new WPI_TalonFX(Constants.shoulderID);
    elevator = new WPI_TalonFX(Constants.elevatorID);
    wrist = new WPI_TalonFX(Constants.wristID);
    topLimit = new DigitalInput(Constants.topLimitPort);
    bottomLimit = new DigitalInput(Constants.bottomLimitPort);
    
 //--------------------------------------------------------------------------------------------------------------------------------
  
    shoulder.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    elevator.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    wrist.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    
 //--------------------------------------------------------------------------------------------------------------------------------
    
    shoulder.setSelectedSensorPosition(0);
    elevator.setSelectedSensorPosition(0);
    wrist.setSelectedSensorPosition(0);

 // soft limits forward----------------------------------------------------------------------------------------------------------------------------------------
    
    shoudler.configForwardSoftLimitThreshold(degreesToTicks());
    elevator.configForwardSoftLimitThreshold(degreesToTicks());
    wrist.configForwardSoftLimitThreshold(degreesToTicks());
    
 // soft limits reverse----------------------------------------------------------------------------------------------------------------------------------------
    
    shoulder.configReverseSoftLimitThreshold(degreesToTicks());
    elevator.configReverseSoftLimitThreshold(degreesToTicks());
    wrist.configReverseSoftLimitThreshold(degreesToTicks());
    
 // enable forward limits----------------------------------------------------------------------------------------------------------------------------------------
    shoudler.configForwardSoftLimitEnable(true); 
    elevator.configForwardSoftLimitEnable(true); 
    wrist.configForwardSoftLimitEnable(true); 
    
 // enable reverse limits---------------------------------------------------------------------------------------------------------------------------------------
   shoudler.configReverseSoftLimitEnable(true); 
   elevator.configReverseSoftLimitEnable(true); 
   wrist.configReverseSoftLimitEnable(true); 
    
 // //Configure Kp, Ki, Kd----------------------------------------------------------------------------------------------------------------------------------------
 shoudler.config_kP(0, );
 shoudler.config_kI(0, );
 shoulder.config_kD(0, );

 elevator.config_kP(0, );
 elevator.config_kI(0, );
 elevator.config_kD(0, );

 wrist.config_kP(0, );
 wrist.config_kI(0, );
 wrist.config_kD(0, );
    
//-------------------------------------------------------------------------------------------------------------------------------------------------------//-------------------------------------------------------------------------------------------------------------------------------------------------------
  public double getWristPosition(){
    return ticksToDegrees(wrist.getSelectedSensorPosition());
  }
  public double getElevatorPosition(){
    return ticksToDegrees(elevator.getSelectedSensorPosition());
  }
  public double getShoulderPosition(){
    return ticksToDegrees(shoulder.getSelectedSensorPosition());
  }
    
//-------------------------------------------------------------------------------------------------------------------------------------------------------
   public boolean getBottomLimits() {
    return !bottomLimit.get();
  }
    
//-------------------------------------------------------------------------------------------------------------------------------------------------------
  public void setShoudlerSpeed(double speed){
  shoulder.set(ControlMode.PercentOutput, speed);
  }
  public void setWristSpeed(double speed){
  wrist.set(ControlMode.PercentOutput, speed);
  }
  public void setElevatorSpeed(double espeed){
  elevator.set(ControlMode.PercentOutput, espeed);
  }
    
//-------------------------------------------------------------------------------------------------------------------------------------------------------
/* * set the reference position for the should and wrist simulataniously
   * @param shoulderPosition - the position of the shoulder in degrees with 0 being stowed flate
   * @param wristPosition - the position of the shoulder in degrees with 0 being stowed vertically
   * @param elevatorPosition - the position of the shoulder in degrees with 0 being stowed at bottom
 **/
 //-------------------------------------------------------------------------------------------------------------------------------------------------------

public void setArmPosition(double wdegrees, double edegrees, double sdegrees){
    wrist.set(ControlMode.Position, degreesToTicks(wdegrees));
    shoulder.set(ControlMode.Position, degreesToTicks(sdegrees));
    elevator.set(ControlMode.Position, degreesToTicks(edegrees));
}
    
 //-------------------------------------------------------------------------------------------------------------------------------------------------------
  // Reset Encoder Method (we may need to take in a position to set the arm at dependant upon which auto we are running)
    
  public void resetWrist(){
    wrist.setSelectedSensorPosition(0);
  }
   public void resetShoulder(){
    shoulder.setSelectedSensorPosition(0);
  }
    public void resetElevator(){
    elevator.setSelectedSensorPosition(0);
  }
    
 //------------------------------------------------------------------------------------------------------------------------------------------------------- 
    
 public static void init() {
  if (currentInstance == null) {
    currentInstance = new Arm();
   }
}
    
 //-------------------------------------------------------------------------------------------------------------------------------------------------------
    

  public static Arm getInstance() {
    init();
    return currentInstance;
}
    
//-------------------------------------------------------------------------------------------------------------------------------------------------------
    
  // ticksToDegrees Method Take in Ticks Return Degrees
  // ticks / ticksPerRevFalcon / gear Ratio * 360 (I think this math is right)  Return this number
  public double ticksToDegrees(double ticks){
    return ticks / Constants.Falcon_Ticks_Per_Rev / Constants.turret_gear_ratio * 360;
}
    
//-------------------------------------------------------------------------------------------------------------------------------------------------------
    
  // DegreesToTicks Method
  // Degrees / 360 * GearRatio * TicksPerRevFalcon (I think this math is right)  Return this number
  public double degreesToTicks(double degrees){
    return degrees / 360 * Constants.turret_gear_ratio * Constants.Falcon_Ticks_Per_Rev;
  }
    
//-------------------------------------------------------------------------------------------------------------------------------------------------------
    
@Override
public void periodic() {
    if (getBottomLimits()){
      resetElevator();
      elevatorMotor.set(ControlMode.PercentOutput, 0);
    }    
}
  
}

