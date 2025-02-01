package CSP_Lib.Motors;

import com.revrobotics.spark.SparkClosedLoopController;

public interface CSP_Motor {

  public void setInverted(boolean inverted);

  public void init();

  public void setBrake(boolean braking);

  public void setRampRate(double rampRate);

  public void set(double percent);

  public void setVoltage(double volts);
  
  public double getRPM(); 

  public double getPositionDegrees(); 

  public double getTemperature();

  public double getCurrent();

  public int getID();
  
  public void setScalar(double scalar);

}
