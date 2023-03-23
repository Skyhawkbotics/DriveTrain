/*
Packages and Imports used for the code.
*/
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

@TeleOp(name = "2023-2024test")
public class Code20232024 extends LinearOpMode {
  //Clock
  private ElapsedTime     runtime = new ElapsedTime();

  private BNO055IMU imu;


  //Create Motor Variables
  private DcMotor whl_LB;
  private DcMotor whl_LF;
  private DcMotor whl_RB;
  private DcMotor whl_RF;
  private DcMotorEx arm_ELEVATOR;
  
  
  
  private CRServo claw_GRIP;
  
  double whl_LB_percent;
  double whl_LF_percent;
  double whl_RB_percent;
  double whl_RF_percent;
  float arm_ELEVATOR_angle = 0;
  double claw_GRIP_angle = 0; // 0.28 to 0.85 | closed to fully opened
  
  double last_time = runtime.seconds(); //Used to find how much time has elapsed per iteration in the runtime loop.
  double reset_last_time = runtime.seconds(); //Last time the robot has reset
  
  boolean claw_gripped = true;
  boolean tankDrive = true;
  boolean right_bumper_DOWN = false;

  @Override
  public void runOpMode() {
    //Initalize Motors and Servos
    whl_LB = hardwareMap.get(DcMotor.class, "left/back");
    whl_LF = hardwareMap.get(DcMotor.class, "left/front");
    whl_RB = hardwareMap.get(DcMotor.class, "right/back");
    whl_RF = hardwareMap.get(DcMotor.class, "right/front");
    
    arm_ELEVATOR = hardwareMap.get(DcMotorEx.class, "arm");
    
    claw_GRIP = hardwareMap.get(CRServo.class, "claw");

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters(

    );

    parameters.mode                = BNO055IMU.SensorMode.IMU;
    parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
    parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    parameters.accelPowerMode      = BNO055IMU.AccelPowerMode.NORMAL;
    parameters.loggingEnabled      = false;
    
    imu = hardwareMap.get(BNO055IMU.class, "imu");

    imu.initialize(parameters);
    telemetry.addData("Mode", "calibrating...");
    telemetry.update();
    waitForStart();
    
    if (opModeIsActive()) {
      // Start the loop
      while (opModeIsActive()) {
        //now_time, the time since the start of the program and is used to find time differentials between loop iterations
        double now_time = runtime.seconds();
        gamepadInputHandling(now_time);

        last_time = now_time; //To find time differentials between loops.
        Orientation orientation = imu.getAngularOrientation();
        
        ////----VARIABLE MONITORING----////
        
        telemetry.addData("righttrigger", gamepad1.right_trigger);
        telemetry.addData("lefttrigger", gamepad1.left_trigger);
        telemetry.addData("leftstickx", gamepad1.left_stick_x);
        telemetry.addData("leftsticky", gamepad1.left_stick_y);
        telemetry.addData("rightstickx", gamepad1.right_stick_x);
        telemetry.addData("rightsticky", gamepad1.right_stick_y);
        telemetry.addData("arm_Elevator_angle", arm_ELEVATOR_angle);
        
        telemetry.addData("orientation", orientation);
        telemetry.addData("velocity", imu.getVelocity());
        telemetry.addData("acceleration", imu.getAcceleration());
        telemetry.addData("temperature", imu.getTemperature());

        telemetry.update();
        
        boolean dif = Math.abs((gamepad1.left_stick_y+gamepad1.left_stick_x))>Math.abs((gamepad1.right_stick_x+gamepad1.right_stick_y));
        
        float drv_stick_y2 = gamepad1.right_stick_y;
        float drv_stick_x2 = gamepad1.right_stick_x;

          whl_LB_percent = gamepad1.left_stick_y;
          whl_LF_percent = gamepad1.left_stick_y;
          whl_RB_percent = gamepad1.right_stick_y;
          whl_RF_percent = gamepad1.right_stick_y;

        
        whl_corrections(); // Corrects/Adjusts power for correct results
        
        //Set power of motors to their corresponding variables
        whl_LB.setPower(whl_LB_percent);
        whl_RB.setPower(whl_RB_percent);
        whl_LF.setPower(whl_LF_percent);
        whl_RF.setPower(whl_RF_percent);
        
        arm_ELEVATOR.setPower(Help.degreesToTick(arm_ELEVATOR_angle));
        claw_GRIP.setPower(claw_GRIP_angle);
        
        telemetry.update();
      }
    }
    
    
    
  }
  
  public void gamepadInputHandling(double now_time) {
    if (gamepad1.a&& arm_ELEVATOR_angle < 1000) {
      arm_ELEVATOR_angle = 5;
    }
    else if (gamepad1.y && arm_ELEVATOR_angle > -1000) {
      arm_ELEVATOR_angle = -5;
    }
    else if (!gamepad1.a || !gamepad1.y) {
      arm_ELEVATOR_angle = 0;
    }
    
    if (!gamepad2.right_bumper && right_bumper_DOWN) {
      if (claw_gripped) {
        claw_GRIP_angle = 1;
        claw_gripped = false;
      }
      else if (!claw_gripped) {
        claw_GRIP_angle = -1;
        claw_gripped = true;
      }
    }
    
    if (gamepad2.right_bumper) {
      right_bumper_DOWN = true;
      //CLAW GRIP/RELEASE
      
    }
    else {
      right_bumper_DOWN = false;
    }
    right_bumper_DOWN = gamepad1.right_bumper;
  }
  public void whl_corrections() {
      whl_RF_percent = (float) (whl_RF_percent * 0.6);
      whl_RB_percent = (float) (whl_RB_percent * -0.6);
      whl_LF_percent = (float) (whl_LF_percent * 0.6);
      whl_LB_percent = (float) (whl_LB_percent * 0.6);
  }
  
  public double getServoDirection(double destinationTarget, double currentTarget, double stopRange) {
    double polarity = (destinationTarget>currentTarget) ? 1 : 0;
    polarity = (Math.abs(destinationTarget-currentTarget)>stopRange) ? polarity : 0.5;
    return polarity;
  }
  
}
