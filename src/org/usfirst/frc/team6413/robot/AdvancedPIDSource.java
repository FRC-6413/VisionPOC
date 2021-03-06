package org.usfirst.frc.team6413.robot;

import edu.wpi.first.wpilibj.PIDSource;

/**
 * @author sbaron
 * @co-author ereese
 * Code from https://bitbucket.org/Zelda_Programming/team-702-pixy-code/src
 *
 */
public interface AdvancedPIDSource extends PIDSource {

	public static enum PIDType {
    	ANGLE,
    	OFFSET
    }
	/**
	   * Set what type the process control variable is.
	   *
	   * @param pidType An enum to select the parameter.
	   */
	  void setPIDType(PIDType pidSource);

	  /**
	   * Get which parameter of the device you are using as a process control variable.
	   *
	   * @return the currently selected PID source parameter
	   */
	  PIDType getPIDType();
}
