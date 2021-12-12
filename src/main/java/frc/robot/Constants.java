// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public class Constants {
    /**
     * Which PID slot to pull gains from. Starting 2018, you can choose from
     * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
     * configuration.
     */
    public static final int kSlotIdx = 0;

    /**
     * Talon FX supports multiple (cascaded) PID loops. For
     * now we just want the primary one.
     */
    public static final int kPIDLoopIdx = 0;

    /**
     * Set to zero to skip waiting for confirmation, set to nonzero to wait and
     * report to DS if action fails.
     */
    public static final int kTimeoutMs = 30;


    public static final double kP = 0.1;
    public static final double kI = 0.001;
    public static final double kD = 5;
    public static final double kF = 1023.0/20660.0;

    /**
     * PID Gains may have to be adjusted based on the responsiveness of control loop.
     * kF: 1023 represents output value to Talon at 100%, 20660 represents Velocity units at 100% output
     *

     *
     * 	                                    			  kP   	 kI    kD      kF          Iz    PeakOut */
}