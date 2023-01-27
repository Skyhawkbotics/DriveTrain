
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

import java.lang.Math;

/*
Code Starts Here.
*/

/*
Hello future programmer. My name is Kevin Vu and initially wrote this code.
It is not very readable and some of the design decisions I made are very questionable.
| For the movement of the claw, look at the gamepadInputHandling() claw section and the clawMove() section.
| now_time-last_time is used to find time inbetween loops and normalize degrees per loop to degrees per second.
*/

@TeleOp(name = "2022-2023fullcode")
public class mechanumdrive extends LinearOpMode {
  //Clock
  private ElapsedTime     runtime = new ElapsedTime();

  //Create Motor Variables
  private DcMotor whl_LB;
  private DcMotor whl_LF;
  private DcMotor whl_RB;
  private DcMotor whl_RF;
  private DcMotorEx arm_ELEVATOR1;
  private DcMotorEx arm_ELEVATOR2;
  private DcMotorEx arm_ROT;
  
  private Servo arm_EXT;
  private CRServo claw_GRIP;
  private CRServo wrist_ROT;
  
  private Servo susan_ROT;
  private DistanceSensor elevator_DISTSENSOR;
  
  //time at which the claw rotates for per movement. Modified when restarting the robot.
  //double CLAW_ROTATE_TIME = 0.18;
  
  //double everything_universalscale = 1;  //Multiplier for angle/power
  //double wheel_universalscale = 0.8; //Multiplier for power for wheels
  //double wheel_equalizerscale = 0; //How much the difference between two sides of the wheel should be evened by (0-1)
  float whl_LB_percent;
  float whl_LF_percent;
  float whl_RB_percent;
  float whl_RF_percent;
  float arm_ROT_angle = 0;
  float arm_ELEVATOR_angle = 0; // 0 to 3500 | lowered to fully raised
  double susan_ROT_percent = 0.5;
  
  double claw_GRIP_angle = 0; // 0.28 to 0.85 | closed to fully opened
  double wrist_ROT_percent = 0.5; // >0.5 to <0.5 | move up or move down
  double arm_EXT_percent = 0.5; // 0 to 1
  
  double wrist_ROT_percent_FROMARM = 0;

  
  double susan_ROT_pos = 0;
  double wrist_ROT_pos = 0; // Increases or decreases based on how much movement the claw makes | Utilized for finding how much to readjust the claw by to reset it.
  double arm_EXT_pos = 0;
  double last_time = runtime.seconds(); //Used to find how much time has elapsed per iteration in the runtime loop.
  double reset_last_time = runtime.seconds(); //Last time the robot has reset
  
  boolean stopreset_soon = false; //Is the robot trying to reset all the motors? (Except wheels)
  boolean arm_Sensor = false;
  
  boolean elevator_HasReset = false;
  boolean claw_gripped = true;
  boolean gamepad2_right_bumper_down_LASTTIME = false;

  //private DistanceSensor distance;
  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    //Initalize Motors and Servos
    whl_LB = hardwareMap.get(DcMotor.class, "left/back");
    whl_LF = hardwareMap.get(DcMotor.class, "left/front");
    whl_RB = hardwareMap.get(DcMotor.class, "right/back");
    whl_RF = hardwareMap.get(DcMotor.class, "right/front");
    
    susan_ROT = hardwareMap.get(Servo.class, "susan_ROT"); // Control servo port 0
    arm_ROT = hardwareMap.get(DcMotorEx.class, "arm_ROT");
    arm_ELEVATOR1 = hardwareMap.get(DcMotorEx.class, "Elevator1");
    arm_ELEVATOR2 = hardwareMap.get(DcMotorEx.class, "Elevator2");

    arm_EXT   = hardwareMap.get(Servo.class, "arm_EXT"); //expansion 3
    claw_GRIP = hardwareMap.get(CRServo.class, "claw_GRIP"); //control servo port 2
    wrist_ROT = hardwareMap.get(CRServo.class, "wrist_ROT"); //expansion 4
    
    //Initalize Sensors
    elevator_DISTSENSOR = hardwareMap.get(DistanceSensor.class, "elevatorDistance");
    
    //--These wheels are reversed for desired results--//
    whl_LB.setDirection(DcMotorSimple.Direction.REVERSE);
    whl_LF.setDirection(DcMotorSimple.Direction.REVERSE);
    //--//
    
    arm_ROT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    arm_ELEVATOR1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    arm_ELEVATOR2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    arm_ROT.setTargetPosition(Help.degreesToTick(0));
    arm_ELEVATOR1.setTargetPosition(Help.degreesToTick(0));
    arm_ELEVATOR2.setTargetPosition(Help.degreesToTick(0));

    arm_ROT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    arm_ELEVATOR1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    arm_ELEVATOR2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    arm_ROT.setVelocity(400);
    arm_ELEVATOR1.setVelocity(1200);
    arm_ELEVATOR2.setVelocity(1200);
    
    //setup touch sensor
    //digitalTouch.setMode(DigitalChannel.Mode.INPUT);


    //--//
    
    
    waitForStart();
    
    if (opModeIsActive()) {
      // Start the loop
      while (opModeIsActive()) {
        //now_time, the time since the start of the program and is used to find time differentials between loop iterations
        double now_time = runtime.seconds();
        
        // Initialization Process //
        //5.2
        /*
        if (!elevator_HasReset && elevator_DISTSENSOR.getDistance(DistanceUnit.CM) > 5.2) {
          arm_ELEVATOR_angle -= 200 * (now_time-last_time);
        }
        else if (!elevator_HasReset && elevator_DISTSENSOR.getDistance(DistanceUnit.CM) < 5.2) {
          elevator_HasReset = true;
          arm_ELEVATOR1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          arm_ELEVATOR2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          arm_ELEVATOR_angle = 0;
          arm_ELEVATOR1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          arm_ELEVATOR2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }*/
        // Finished Robot intitalzieaiton//
        ////----INPUTS----////
        //if (elevator_HasReset) {
          gamepadInputHandling(now_time);
        //}
        
        
        //--All things related to resetting the motors--// DEPRECATED FOR NOW
        /*
        if (now_time - reset_last_time > 4 && stopreset_soon) {
          stopreset_soon = false;
        }
        else if (now_time - reset_last_time < 4 && stopreset_soon){
          arm_extender_desiredangle-=500 * (now_time-last_time);
          arm_rotate_desiredangle-=300 * (now_time-last_time) * everything_universalscale;
        }
        
        if (gamepad1.start && !stopreset_soon) {
          stopreset_soon = true;
          
          claw_grip_desiredangle = 0.28;
          clawMove(1,claw_rotate_position, now_time);
          claw_rotate_position = 0;
          
          reset_last_time = runtime.seconds();
        }
        */
        last_time = now_time; //To find time differentials between loops.
        
        ////----VARIABLE MONITORING----////
        
        telemetry.addData("righttrigger", gamepad1.right_trigger);
        telemetry.addData("lefttrigger", gamepad1.left_trigger);
        telemetry.addData("leftstickx", gamepad1.left_stick_x);
        telemetry.addData("leftsticky", gamepad1.left_stick_y);
        telemetry.addData("rightstickx", gamepad1.right_stick_x);
        telemetry.addData("rightsticky", gamepad1.right_stick_y);
        telemetry.addData("arm_ext_percent", arm_EXT_percent);
        telemetry.addData("arm_ELEVATOR_angle", arm_ELEVATOR_angle);
        telemetry.addData("arm_ROT_angle", arm_ROT_angle);
        telemetry.addData("claw_GRIP_angle", claw_GRIP_angle);
        telemetry.addData("wrist", wrist_ROT_percent);

        telemetry.addData("elevatordist", elevator_DISTSENSOR.getDistance(DistanceUnit.CM));
        telemetry.update();
        
        ////ZERO OUT ARM EXTENDER////
        //if (digitalTouch.getState() == false) {
          //arm_ELEVATOR_angle-=400 * (now_time-last_time);
        //}

        
        ////----WHEEL DRIVING----////
        /*
        whl_LB_percent = gamepad1.left_stick_y;
        whl_LF_percent = gamepad1.left_stick_y;
        whl_RB_percent = gamepad1.right_stick_y;
        whl_RF_percent = gamepad1.right_stick_y;
        */
        
        boolean dif = Math.abs((gamepad1.left_stick_y+gamepad1.left_stick_x))>Math.abs((gamepad1.right_stick_x+gamepad1.right_stick_y));
        
        float drv_stick_y2 = gamepad1.right_stick_y;
        float drv_stick_x2 = gamepad1.right_stick_x;
        if (!dif) {
          if (Math.abs(gamepad1.right_stick_y) > Math.abs(gamepad1.right_stick_x)) {
            whl_LB_percent = drv_stick_y2;
            whl_LF_percent = drv_stick_y2;
            whl_RB_percent = drv_stick_y2;
            whl_RF_percent = drv_stick_y2;
          }
          else {
            if (drv_stick_x2 > 0) {
              whl_RF_percent = drv_stick_x2 * 1;
              whl_RB_percent = drv_stick_x2 * -1;
              whl_LF_percent = drv_stick_x2 * -1;
              whl_LB_percent = drv_stick_x2 * 1;
            }
            
            if (drv_stick_x2 < 0) {
              whl_LF_percent = drv_stick_x2 * -1;
              whl_LB_percent = drv_stick_x2 * 1;
              whl_RB_percent = drv_stick_x2 * -1;
              whl_RF_percent = drv_stick_x2 * 1;
            }
          }
        }
        else {
          float drv_stick_y = gamepad1.left_stick_y;
          float drv_stick_x = gamepad1.left_stick_x * 0.2f;
          
          whl_LB_percent = drv_stick_y - drv_stick_x;
          whl_LF_percent = drv_stick_y - drv_stick_x;
          whl_RB_percent = drv_stick_y + drv_stick_x;
          whl_RF_percent = drv_stick_y + drv_stick_x;
        }
        
        whl_corrections(); // Corrects/Adjusts power for correct results
        
        //Set power of motors to their corresponding variables
        whl_LB.setPower(whl_LB_percent);
        whl_RB.setPower(whl_RB_percent);
        whl_LF.setPower(whl_LF_percent);
        whl_RF.setPower(whl_RF_percent);
        
        //Set position of arm and claw motors to their corresponding variables.
        
        claw_GRIP.setPower(claw_GRIP_angle);
        wrist_ROT.setPower(wrist_ROT_percent);
        arm_EXT.setPosition(arm_EXT_percent);

        arm_ROT.setTargetPosition(-Help.degreesToTick(arm_ROT_angle));
        arm_ELEVATOR1.setTargetPosition(+Help.degreesToTick(arm_ELEVATOR_angle));
        arm_ELEVATOR2.setTargetPosition(-Help.degreesToTick(arm_ELEVATOR_angle));

        susan_ROT.setPosition(susan_ROT_percent);
        
        

        
        
        telemetry.update();
      }
    }
    
    
    
  }
  
  public void gamepadInputHandling(double now_time) {

    if (!gamepad2.right_bumper && gamepad2_right_bumper_down_LASTTIME) {
      if (claw_gripped) {
        claw_GRIP_angle = 0.1;
        claw_gripped = false;
      }
      else if (!claw_gripped) {
        claw_GRIP_angle = -0.3;
        claw_gripped = true;
      }
    }
    
    if (gamepad2.right_bumper) {
      gamepad2_right_bumper_down_LASTTIME = true;
      //CLAW GRIP/RELEASE
      
    }
    else {
      gamepad2_right_bumper_down_LASTTIME = false;
    }
    
    if (gamepad1.dpad_down) {
      //ARM EXTENDER ga
      arm_EXT_percent = 0.2;
    }
    else if (gamepad1.dpad_up) {
      arm_EXT_percent = 0.8;
    }
    else {
      arm_EXT_percent = 0.5;
    }

     if (gamepad2.left_trigger > 0.1) {
      //ARM EXTENDER
      arm_EXT_percent = (gamepad2.left_trigger / 2) + 0.5;
    }
    else if (gamepad2.right_trigger > 0.1) {
      arm_EXT_percent = (gamepad2.right_trigger / -2) + 0.5;
      //arm_EXT_percent = (gamepad2.left_stick_x / 2) + 0.5;
    }
    else {
      arm_EXT_percent = 0.5;
    }
    

    if (gamepad2.left_stick_y != 0) {
      //ARM ROTATION
      arm_ROT_angle+=gamepad2.left_stick_y * 1000 * (now_time-last_time);
      wrist_ROT_percent_FROMARM = (gamepad2.left_stick_y / 8);
    }

    if (gamepad2.left_stick_x > 0.1 || gamepad2.left_stick_x < -0.1) {
      //SUSAN ROTATION
      susan_ROT_percent = (gamepad2.left_stick_x / -6) + 0.5;
      susan_ROT_pos -= (now_time-last_time);

      if (susan_ROT_percent > 1) {
        susan_ROT_percent = 1;
      }
      else if (susan_ROT_percent < 0) {
        susan_ROT_percent = 0;
      }
    } else {
      susan_ROT_percent = 0.5;
    }

    //ELEVATOR
    if (gamepad2.dpad_up) {
      arm_ELEVATOR_angle += 30;
    }
    else if (gamepad2.dpad_down) {
      arm_ELEVATOR_angle -= 30;
    }
    

      wrist_ROT_percent = (gamepad2.right_stick_y / 4) + wrist_ROT_percent_FROMARM;
    wrist_ROT_pos -= (now_time-last_time);



    

    ////----BOUNDARIES----////

/*
    if (arm_EXT_angle < -123123123) { 
      arm_EXT_angle = 0;
      
    }
    else if (arm_EXT_angle > 123123123) {
      arm_EXT_angle = 1300;
    }*/

    //Boundaries of the arm vertical rotation
    /*
    if (arm_ELEVATOR_angle > 2480) {
      arm_ELEVATOR_angle = 2480;
    }
    else if (arm_ELEVATOR_angle < 0) {
      arm_ELEVATOR_angle = 0;
    }

    // Boundaries of the claw
    if (claw_GRIP_angle < 0.28) {
      claw_GRIP_angle = 0.28;
    }
    else if (claw_GRIP_angle > 0.85) {
      claw_GRIP_angle = 0.85;
    }*/

  }
  public void whl_corrections() {
      whl_RF_percent = (float) (whl_RF_percent * -0.5);
      whl_RB_percent = (float) (whl_RB_percent * -0.5);
      whl_LF_percent = (float) (whl_LF_percent * -0.5);
      whl_LB_percent = (float) (whl_LB_percent * -0.5);
  }
  
  
}

class Help {
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
}
