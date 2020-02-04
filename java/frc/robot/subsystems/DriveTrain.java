/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/* FRC Team 7890 SeQuEnCe                                                     */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Falcon Library Import
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
// NavX Import
import com.kauailabs.navx.frc.AHRS;

// This is the import from the Constants.java section also seen in code as the kCANIds.----
import frc.robot.Constants.kCANIds;
import frc.robot.Constants.kRobotDims;
import frc.robot.Constants.kFalcons;

public class DriveTrain extends SubsystemBase {

  // Falcon Declarations
  // kCANIds are found in Constants.java section
  private final WPI_TalonFX leftPrimary = new WPI_TalonFX(kCANIds.iLeftPrimary);
  private final WPI_TalonFX leftFollower = new WPI_TalonFX(kCANIds.iLeftFollower);
  private final WPI_TalonFX rightPrimary = new WPI_TalonFX(kCANIds.iRightPrimary);
  private final WPI_TalonFX rightFollower = new WPI_TalonFX(kCANIds.iRightFollower);

  // If you need to change the track dimentions, change it in the Constants.java section
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(kRobotDims.dTrackWidth);
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

  AHRS gyro = new AHRS (SPI.Port.kMXP);

  public DriveTrain() {

    // The followers will now follow the coding of the primaries
    leftFollower.follow(leftPrimary);
    rightFollower.follow(rightPrimary);

    // The commanded speeds for the falcons will now all be positive for forward motion
    leftPrimary.setInverted(false);
    rightPrimary.setInverted(true);

  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(getSpeedGear("left", false), getSpeedGear("right", false));
  }

  private double getSpeedGear(String side, Boolean highGear){
    if (highGear) {
      if (side.equals("left")) {
        // The sensor velocity is counts per 100 ms so multiply by 10 to convert to seconds
        return leftPrimary.getSelectedSensorVelocity() / kFalcons.dCountsPerRev * 10.0 * Math.PI * kRobotDims.dWheelDia;
      } else{
        return rightPrimary.getSelectedSensorVelocity() * Math.PI;
      }
    
    }else {
      if (side.equals("left")) {
        return leftPrimary.getSelectedSensorVelocity() * Math.PI;
      } else{
        return rightPrimary.getSelectedSensorVelocity() * Math.PI;
      }
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
