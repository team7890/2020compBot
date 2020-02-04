/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/* FRC Team 7890 SeQuEnCe                                                     */
/*----------------------------------------------------------------------------*/
package frc.robot;

import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class kCANIds {
        public static final int iLeftPrimary = 1;
        public static final int iLeftFollower = 2;
        public static final int iRightPrimary = 3;
        public static final int iRightFollower = 4;
        
    }

    public static final class kRobotDims {
        public static final double dTrackWidth = Units.inchesToMeters(24.75);
        public static final double dWheelDia = Units.inchesToMeters(6.0);
        public static final double dGearBoxRatioHigh = 20.0;
        public static final double dGearBoxRatioLow = 20.0;
    }

    public static final class kFalcons {
        public static final double dCountsPerRev = 2048.0; 
    }
}
