package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.Temperature;

import java.lang.Math;


public class Help {
  public static int degreesToTick (int degrees) {
      int tickDegreeRatio = 5;

      return degrees/tickDegreeRatio;
  }
  public static int degreesToTick (float degrees) {
      int tickDegreeRatio = 5;

      return (int) degrees/tickDegreeRatio;
  }
  
  public static double numSign (double num) {
    if (num >= 0) {
      return 1.01;
    }
    else {
      return -1.01;
    }
  }

  public double getServoDirection(double destinationTarget, double currentTarget, double stopRange) {
    double polarity = (destinationTarget>currentTarget) ? 1 : 0;
    polarity = (Math.abs(destinationTarget-currentTarget)>stopRange) ? polarity : 0.5;
    return polarity;
  }

  public static double trueAngleDif(double desiredAngle, double imuAngle) {
    // Case 1: 179, -179, true dif is 2, actual dif is 358
    // Case 2: -2, 2, true dif is 4, actual dif 4
    // Case 1: 179, 1, true dif is 178, actual dif 178

    double actualDif = Math.abs(imuAngle - desiredAngle);
    double polarity = Math.signum(imuAngle - desiredAngle);
    
    //If either imu or desired angle is negative, reverse then calibrate to basic system
    imuAngle+=180;
    desiredAngle+=180;
    /*
    if (imuAngle < 180) { 
      imuAngle = 180 - imuAngle;
    }
    if (desiredAngle < 180) { 
      desiredAngle = 180 - desiredAngle;
    }
    */
    /*
    if (imuAngle < 0) { 
      imuAngle += 180;
    }
    if (desiredAngle < 0) {
      desiredAngle += 180;
    }
    */
    // possible range of values; 0 to 360
    // Check from middle range
    double polarity1;
    double polarity2;
    double distance1 = Math.abs(imuAngle-desiredAngle);
    polarity1 = Math.signum(imuAngle-desiredAngle);
    double distance2;
    if (imuAngle > desiredAngle) {
      distance2 = (360 - imuAngle) + desiredAngle;
      polarity2 = -Math.signum(imuAngle-desiredAngle);
    }
    else {
      distance2 = (360 - desiredAngle) + imuAngle;
      polarity2 = -Math.signum(imuAngle-desiredAngle);
    }
    
    //Check from outer bands
    
    //double actualDif = Math.abs(imu.getAngularOrientation().firstAngle - desiredAngle);
    //double polarity = Math.signum(imu.getAngularOrientation().firstAngle - desiredAngle);
    double chosenDistance = (distance1<distance2) ? distance1 : distance2;
    double chosenPolarity = (distance1<distance2) ? polarity1 : polarity2;
    return (chosenDistance * chosenPolarity);
    /*
    if (actualDif < 180) {
      return actualDif * polarity;
    }
    else {
      return (360.0 - actualDif) * polarity;
    }*/
  }
  
  public static double angleCorrection(double current, double increment) {
    double output = current + increment;
    if (output > 180) {
      double overflow = output - 180;
      output = -180 + overflow;
    }
    else if (output < -180) {
      double overflow = output + 180;
      output = 180 + overflow;
    }
    return output;
  }

  public static double getAverage_aprilTagInfos(double[][] aprilTagInfos, int index) {

    int output = 0;
    int MAX_ITERATIONS = 1;

    for (int i = 0; i < MAX_ITERATIONS; i++) {
      output += aprilTagInfos[i][index];
    }

    output = output / MAX_ITERATIONS;
    return output;
  }
}
