/*
Packages and Imports used for the code.
*/
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

import java.lang.Math;

@TeleOp(name = "2023-2024test")
public class mechanumdrive extends LinearOpMode {
  //Clock
  private ElapsedTime     runtime = new ElapsedTime();

  private BNO055IMU imu;


  //Create Motor Variables
  private DcMotor whl_LB;
  private DcMotor whl_LF;
  private DcMotor whl_RB;
  private DcMotor whl_RF;
  
  private CRServo claw_GRIP;
  
  float whl_LB_percent;
  float whl_LF_percent;
  float whl_RB_percent;
  float whl_RF_percent;
  float arm_ELEVATOR_angle = 0;
  double claw_GRIP_angle = 0; // 0.28 to 0.85 | closed to fully opened
  
  double last_time = runtime.seconds(); //Used to find how much time has elapsed per iteration in the runtime loop.
  double reset_last_time = runtime.seconds(); //Last time the robot has reset
  
  boolean claw_gripped = true;
  boolean tankDrive = true;

  @Override
  public void runOpMode() {
    //Initalize Motors and Servos
    whl_LB = hardwareMap.get(DcMotor.class, "left/back");
    whl_LF = hardwareMap.get(DcMotor.class, "left/front");
    whl_RB = hardwareMap.get(DcMotor.class, "right/back");
    whl_RF = hardwareMap.get(DcMotor.class, "right/front");
    
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters(
      new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
            RevHubOrientationOnRobot.UsbFacingDirection.DOWN
      )
    );

    parameters.mode                = BNO055IMU.SensorMode.IMU;
    parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
    parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
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
        
        last_time = now_time; //To find time differentials between loops.
        
        YawPitchRollAngles robotOrientation;
        robotOrientation = imu.getRobotYawPitchRollAngles();

        double Yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);
        double Pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
        double Roll  = robotOrientation.getRoll(AngleUnit.DEGREES);
        
        ////----VARIABLE MONITORING----////
        
        telemetry.addData("righttrigger", gamepad1.right_trigger);
        telemetry.addData("lefttrigger", gamepad1.left_trigger);
        telemetry.addData("leftstickx", gamepad1.left_stick_x);
        telemetry.addData("leftsticky", gamepad1.left_stick_y);
        telemetry.addData("rightstickx", gamepad1.right_stick_x);
        telemetry.addData("rightsticky", gamepad1.right_stick_y);
        
        telemetry.addData("y_rot", Yaw);
        telemetry.addData("x_rot", Pitch);
        telemetry.addData("z_rot", Roll);

        telemetry.update();
        
        boolean dif = Math.abs((gamepad1.left_stick_y+gamepad1.left_stick_x))>Math.abs((gamepad1.right_stick_x+gamepad1.right_stick_y));
        
        float drv_stick_y2 = gamepad1.right_stick_y;
        float drv_stick_x2 = gamepad1.right_stick_x;

        if (!tankDrive) {
          if (!dif) {
            if (Math.abs(gamepad1.right_stick_y) > Math.abs(gamepad1.right_stick_x)) {
              whl_LB_percent = drv_stick_y2;
              whl_LF_percent = drv_stick_y2;
              whl_RB_percent = drv_stick_y2;
              whl_RF_percent = drv_stick_y2;
            }
            else {
              if (drv_stick_x2 > 0.9) {
                whl_RF_percent = 1;
                whl_RB_percent = -0.7f;
                whl_LF_percent = -1;
                whl_LB_percent = 0.7f;
                counterActwheels = 1;
              }
              
              else if (drv_stick_x2 < -0.9) {
                whl_LF_percent = 1;
                whl_LB_percent = -0.7f;
                whl_RB_percent = 0.7f;
                whl_RF_percent = -1;
                counterActwheels = -1;
              }
              
              //If the stick WAS active last iteration, and is now NOT active, we need to apply a counterforce
              else if (counterActwheels != 0){
                counterActwheels_POLARITY = counterActwheels;
                counterActwheels = 0;
                counterActwheels_ACTIVE = now_time;
              }
              
              //Check if the wheels need to be counteracted this iteration
              if (counterActwheels_ACTIVE != 0) {
                
                whl_RB_percent = 0.9f * counterActwheels_POLARITY;
                whl_LB_percent = -0.9f * counterActwheels_POLARITY;
                whl_RF_percent = -1f * counterActwheels_POLARITY;
                whl_LF_percent = 1f * counterActwheels_POLARITY;
                //Make counteract force last only for 0.2 seconds
                if (now_time-counterActwheels_ACTIVE > 0.2){
                  counterActwheels_ACTIVE = 0;
                }
              }
            }
          }
        else {
          float drv_stick_y = gamepad1.left_stick_y;
          float drv_stick_x = gamepad1.left_stick_x;
          if (Math.abs(drv_stick_x)-Math.abs(drv_stick_y)>0.8) {

            whl_LB_percent = - drv_stick_x*0.4f;
            whl_LF_percent = - drv_stick_x*0.4f;
            whl_RB_percent = + drv_stick_x*0.6f;
            whl_RF_percent = + drv_stick_x*0.6f;
          }
          else {
          
            drv_stick_x = drv_stick_x * 0.2f;
            
            whl_LB_percent = drv_stick_y - drv_stick_x;
            whl_LF_percent = drv_stick_y - drv_stick_x;
            whl_RB_percent = drv_stick_y + drv_stick_x;
            whl_RF_percent = drv_stick_y + drv_stick_x;
          
          }
        }
        } else if (tankDrive) {

          whl_LB_percent = gamepad1.left_stick_y;
          whl_LF_percent = gamepad1.left_stick_y;
          whl_RB_percent = gamepad1.right_stick_y;
          whl_RF_percent = gamepad1.right_stick_y;

        }
        
        whl_corrections(); // Corrects/Adjusts power for correct results
        
        //Set power of motors to their corresponding variables
        whl_LB.setPower(whl_LB_percent);
        whl_RB.setPower(whl_RB_percent);
        whl_LF.setPower(whl_LF_percent);
        whl_RF.setPower(whl_RF_percent);
        
        
        telemetry.update();
      }
    }
    
    
    
  }
  
  public void gamepadInputHandling(double now_time) {
    
  }
  public void whl_corrections() {
      whl_RF_percent = (float) (whl_RF_percent * -0.5);
      whl_RB_percent = (float) (whl_RB_percent * 0.5);
      whl_LF_percent = (float) (whl_LF_percent * 0.5);
      whl_LB_percent = (float) (whl_LB_percent * -0.5);
  }
  
  public double getServoDirection(double destinationTarget, double currentTarget, double stopRange) {
    double polarity = (destinationTarget>currentTarget) ? 1 : 0;
    polarity = (Math.abs(destinationTarget-currentTarget)>stopRange) ? polarity : 0.5;
    return polarity;
  }
  
}
