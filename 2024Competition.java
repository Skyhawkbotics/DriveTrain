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

@TeleOp(name = "2024 testing")
public class mechanumdrive extends LinearOpMode {
  //Clock Variable
  private ElapsedTime     runtime = new ElapsedTime();

  // IMU - Includes Gyroscope / Acceleromotor / Thermometer and a lot lot more random stuff
  private BNO055IMU imu;


  //Create Motor Variables
  private DcMotorEx whl_LB;
  private DcMotorEx whl_LF;
  private DcMotorEx whl_RB;
  private DcMotorEx whl_RF;
  private DcMotorEx arm_ELEVATOR;
  
  private CRServo claw_GRIP;

  // Max ranges from -1 to 1
  double whl_LB_percent;
  double whl_LF_percent;
  double whl_RB_percent;
  double whl_RF_percent;
  
  float arm_ELEVATOR_speed = 0;
  double claw_GRIP_angle = 0; // 0.28 to 0.85 | closed to fully opened
  float arm_ELEVATOR_POSITION;
  
  double last_time = runtime.seconds(); //Used to find how much time has elapsed per iteration in the runtime loop.
  double reset_last_time = runtime.seconds(); //Last time the robot has reset
  
  double clock_timer_MAX = 900000.0;
  double clock_timer = clock_timer_MAX;
  boolean clock_active = false;
  
  boolean claw_gripped = true;
  boolean right_bumper_DOWN = false;
  
  double startRobotAngle = 0.0;
  Orientation orientation = null;
  
  @Override
  public void runOpMode() {
    //Initalize Motors and Servos
    whl_LB = hardwareMap.get(DcMotorEx.class, "left/back");
    whl_LF = hardwareMap.get(DcMotorEx.class, "left/front");
    whl_RB = hardwareMap.get(DcMotorEx.class, "right/back");
    whl_RF = hardwareMap.get(DcMotorEx.class, "right/front");
    
    //arm_ELEVATOR = hardwareMap.get(DcMotorEx.class, "arm");
    
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
    orientation = imu.getAngularOrientation();
    startRobotAngle = orientation.firstAngle;
    
    telemetry.addData("Mode", "calibrating...");
    telemetry.update();
    
    
    setWheelMode("power");
    
    waitForStart();
    
    if (opModeIsActive()) {
      // Start the loop
      while (opModeIsActive()) {
        //now_time, the time since the start of the program and is used to find time differentials between loop iterations
        double now_time = runtime.seconds();
        if (clock_timer >= 0) {
          gamepadInputHandling(now_time);
        }
        clock(now_time);
        last_time = now_time; //To find time differentials between loops.
        orientation = imu.getAngularOrientation();

        
        ////----VARIABLE MONITORING----////
        
        telemetry.addData("righttrigger", gamepad1.right_trigger);
        telemetry.addData("lefttrigger", gamepad1.left_trigger);
        telemetry.addData("leftstickx", gamepad1.left_stick_x);
        telemetry.addData("leftsticky", gamepad1.left_stick_y);
        telemetry.addData("rightstickx", gamepad1.right_stick_x);
        telemetry.addData("rightsticky", gamepad1.right_stick_y);
        telemetry.addData("arm_Elevator_speed", arm_ELEVATOR_speed);
        telemetry.addData("orientation", orientation);
        telemetry.addData("velocity", imu.getVelocity());
        telemetry.addData("acceleration", imu.getAcceleration());
        telemetry.addData("firstAngle", imu.getAngularOrientation().firstAngle);
        telemetry.addData("", "");
        telemetry.addData("", "");
        telemetry.addData("", "");
        telemetry.addData("", "");
        telemetry.addData("Clock Active", clock_active);
        telemetry.addData("Clock Time", Math.floor(clock_timer));
        telemetry.update();
        
        //twoDriveHandling(gamepad1.left_stick_y, gamepad1.left_stick_x);
        //autoDriveHandling();
        tankDriveHandling();
        whl_corrections(); // Corrects/Adjusts power for correct results
        
        //Set power of motors to their corresponding variables when clock is 0
        if (clock_timer <= 0) {
          whl_LB_percent = 0;
          whl_RB_percent = 0;
          whl_LF_percent = 0;
          whl_RF_percent = 0;
        }
        setPower();
      }
    }
  }
  
  public void setPower() {
    
    whl_LB.setPower(-whl_LB_percent);
    whl_RB.setPower(-whl_RB_percent);
    whl_LF.setPower(whl_LF_percent);
    whl_RF.setPower(whl_RF_percent);
    
    /* -- This code block is to be used during autonomoous mode.
    whl_LB.setTargetPosition((int) whl_LB_percent);
    whl_RB.setTargetPosition((int) whl_RB_percent);
    whl_LF.setTargetPosition((int) whl_LF_percent);
    whl_RF.setTargetPosition((int) whl_RF_percent);
    */
    //arm_ELEVATOR.setTargetPosition(Help.degreesToTick(arm_ELEVATOR_speed));
    claw_GRIP.setPower(claw_GRIP_angle);
    //telemetry.update();
  }
  
  public void gamepadInputHandling(double now_time) {
    
  }
  
  public void clock(double now_time) {
    if (gamepad2.start) {
      clock_active = false;
      clock_timer = clock_timer_MAX;
    }
    else if (!clock_active && !gamepad1.atRest()) {
      clock_active = true;
    }
    
    if (clock_active) {
      clock_timer -= (now_time-last_time);
      if (clock_timer < 0.0)
        clock_timer = 0.0;
    }
  }
  
  public void whl_corrections() {
    //1st mult: individual wheel balance
    //2nd mult: better rotation (weaker front wheels)
    //3rd mult: weaker overall wheels
      whl_RF_percent = (float) (whl_RF_percent * 0.65 * 0.7 * 0.6);
      whl_RB_percent = (float) (whl_RB_percent * -0.55 *1* 0.6);
      whl_LF_percent = (float) (whl_LF_percent * 0.65 *0.7 * 0.6);
      whl_LB_percent = (float) (whl_LB_percent * 0.6 *1*0.6);
      
      /* STRAFE
      whl_RF_percent = (float) (whl_RF_percent * 0.6 * 0.6);
      whl_RB_percent = (float) (whl_RB_percent * -0.5 *0.8);
      whl_LF_percent = (float) (whl_LF_percent * 0.65 *0.6);
      whl_LB_percent = (float) (whl_LB_percent * 0.6 *0.8);
      */
  }
  
  public double getServoDirection(double destinationTarget, double currentTarget, double stopRange) {
    double polarity = (destinationTarget>currentTarget) ? 1 : 0;
    polarity = (Math.abs(destinationTarget-currentTarget)>stopRange) ? polarity : 0.5;
    return polarity;
  }
  
  public void setWheelMode(String mode){
    if (mode == "position") {
      whl_LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      whl_RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      whl_LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      whl_RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  
      whl_LB.setTargetPosition(0);
      whl_RB.setTargetPosition(0);
      whl_LF.setTargetPosition(0);
      whl_RF.setTargetPosition(0);
  
      whl_LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      whl_RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      whl_LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      whl_RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  
      whl_LB.setVelocity(300);
      whl_RB.setVelocity(300);
      whl_LF.setVelocity(300);
      whl_RF.setVelocity(300);
    }
    else if (mode == "power") {
      whl_LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      whl_RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      whl_LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      whl_RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    
  }
  
  public void autoDriveHandling() {
    whl_LB_percent = 1000;
    whl_LF_percent = 1000;
    whl_RB_percent = 1000;
    whl_RF_percent = 1000;
  }
  
  public void rotate(String type, double angle) {
    //set whl mode to power
    setWheelMode("power");
    
    //Based on angle difference, rotate left / right
    double angleDif = trueAngleDif(angle);
    
    //While goal is far enough away
    while (Math.abs(imu.getAngularOrientation().firstAngle - angle) > 5 && opModeIsActive()) {
      angleDif = trueAngleDif(angle);
      telemetry.addData("a", angleDif);
      telemetry.addData("b", imu.getAngularOrientation().firstAngle);
      telemetry.update();
      if (angleDif > 0) {
        //Rotate to the LEFT?
        twoDriveHandling(0, -1.0);
      }
      else if (angleDif < 0) {
        //Rotate to the RIGHT?
        twoDriveHandling(0, 1.0);
      }
      whl_corrections();
      setPower();
    }
    //Goal is reached, function end
    
  }
  
  public double trueAngleDif(double angle) {
    // Case 1: 179, -179, true dif is 2, actual dif is 358
    // Case 2: -2, 2, true dif is 4, actual dif 4
    // Case 1: 179, 1, true dif is 178, actual dif 178
    
    double actualDif = Math.abs(imu.getAngularOrientation().firstAngle - angle);
    double polarity = Math.signum(imu.getAngularOrientation().firstAngle - angle);
    
    if (actualDif < 180) {
      return actualDif * polarity;
    }
    else {
      return (360.0 - actualDif) * polarity;
    }
  }
  
  public void twoDriveHandling(double Y, double X) {
    whl_LB_percent = 0;
    whl_LF_percent = 0;
    whl_RB_percent = 0;
    whl_RF_percent = 0;
    
    whl_LB_percent += Y;
    whl_LF_percent += Y;
    whl_RB_percent += Y;
    whl_RF_percent += Y;
    
    whl_LB_percent -= X*1.5;
    whl_LF_percent += X;
    whl_RB_percent += X*1.5;
    whl_RF_percent -= X;
    
    whl_LB_percent = whl_LB_percent/1.5;
    whl_LF_percent = whl_LF_percent/1.5;
    whl_RB_percent = whl_RB_percent/1.5;
    whl_RF_percent = whl_RF_percent/1.5;
  }
  
  public void halfHalfDriveHandling() {
    whl_LB_percent = 0;
    whl_LF_percent = 0;
    whl_RB_percent = 0;
    whl_RF_percent = 0;
    
    whl_LB_percent += gamepad1.left_stick_y;
    whl_LF_percent += gamepad1.left_stick_y;
    whl_RB_percent += gamepad1.left_stick_y;
    whl_RF_percent += gamepad1.left_stick_y;
    
    whl_LB_percent -= gamepad1.left_stick_x*1.5;
    whl_LF_percent += gamepad1.left_stick_x;
    whl_RB_percent += gamepad1.left_stick_x*1.5;
    whl_RF_percent -= gamepad1.left_stick_x;
    
    whl_LB_percent += gamepad1.right_stick_y;
    whl_LF_percent += gamepad1.right_stick_y;
    whl_RB_percent += gamepad1.right_stick_y;
    whl_RF_percent += gamepad1.right_stick_y;
    
    whl_LB_percent -= gamepad1.right_stick_x*1.5;
    whl_LF_percent += gamepad1.right_stick_x;
    whl_RB_percent += gamepad1.right_stick_x*1.5;
    whl_RF_percent -= gamepad1.right_stick_x;
    
    whl_LB_percent = whl_LB_percent/3;
    whl_LF_percent = whl_LF_percent/3;
    whl_RB_percent = whl_RB_percent/3;
    whl_RF_percent = whl_RF_percent/3;
  }
  
  public void tankDriveHandling() {
    boolean dif = Math.abs((gamepad1.left_stick_y+gamepad1.left_stick_x))>Math.abs((gamepad1.right_stick_x+gamepad1.right_stick_y));
    
    float drv_stick_y2 = gamepad1.right_stick_y;
    float drv_stick_x2 = gamepad1.right_stick_x;
    float truth = (Math.abs(gamepad1.right_stick_y) - Math.abs(gamepad1.left_stick_y) > 0) ? gamepad1.right_stick_y : gamepad1.left_stick_y;
    
      whl_LB_percent = gamepad1.left_stick_y;
      whl_LF_percent = gamepad1.left_stick_y;
      whl_RB_percent = gamepad1.right_stick_y;
      whl_RF_percent = gamepad1.right_stick_y;

    if (gamepad1.left_bumper) {
      whl_LB_percent = truth;
      whl_LF_percent = truth;
      whl_RB_percent = truth;
      whl_RF_percent = truth;
    }
    /*
    if (gamepad1.dpad_right) {
      whl_RF_percent = 2;
      whl_RB_percent = -1.5f;
      whl_LF_percent = -2;
      whl_LB_percent = 1.5f;
    }
    
    else if (gamepad1.dpad_left) {
      whl_LF_percent = 2;
      whl_LB_percent = -1.5f;
      whl_RB_percent = 1.5f;
      whl_RF_percent = -2;
    }*/
    
    if (gamepad1.left_stick_y > 0.9 && gamepad1.right_stick_y < -0.9) {
      whl_RF_percent = 1;
      whl_RB_percent = -1.5f;
      whl_LF_percent = -1;
      whl_LB_percent = 1.5f;
    }
    
    else if (gamepad1.left_stick_y < -0.9 && gamepad1.right_stick_y > 0.9) {
      whl_LF_percent = 1;
      whl_LB_percent = -1.5f;
      whl_RB_percent = 1.5f;
      whl_RF_percent = -1;
    }
   }
  }
  




