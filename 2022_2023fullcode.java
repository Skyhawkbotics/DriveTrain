
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
  private DcMotorEx arm_EXT;
  private DcMotorEx arm_ELEVATOR1;
  private DcMotorEx arm_ELEVATOR2;
  private DcMotorEx arm_ROT;
  private Servo claw_GRIP;
  private Servo wrist_ROT;
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
  float arm_EXT_angle = 0; // 0 to 1300 | retracted to fully extended
  float arm_ROT_angle = 0;
  float arm_ELEVATOR_angle = 0; // 0 to 3500 | lowered to fully raised
  double susan_ROT_percent = 0.5;
  double claw_GRIP_angle = 0.28; // 0.28 to 0.85 | closed to fully opened
  double wrist_ROT_percent = 0.5; // >0.5 to <0.5 | move up or move down
  
  double susan_ROT_pos = 0;
  double wrist_ROT_pos = 0; // Increases or decreases based on how much movement the claw makes | Utilized for finding how much to readjust the claw by to reset it.
  
  double last_time = runtime.seconds(); //Used to find how much time has elapsed per iteration in the runtime loop.
  double reset_last_time = runtime.seconds(); //Last time the robot has reset
  
  boolean stopreset_soon = false; //Is the robot trying to reset all the motors? (Except wheels)
  boolean arm_Sensor = false;
  
  boolean elevator_HasReset = false;

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
    arm_EXT = hardwareMap.get(DcMotorEx.class, "arm_EXT"); //Expansion port 1
    arm_ROT = hardwareMap.get(DcMotorEx.class, "arm_ROT")
    arm_ELEVATOR1 = hardwareMap.get(DcMotorEx.class, "Elevator1");
    arm_ELEVATOR2 = hardwareMap.get(DcMotorEx.class, "Elevator2");

    claw_GRIP = hardwareMap.get(Servo.class, "claw_GRIP"); //control servo port 2
    wrist_ROT = hardwareMap.get(Servo.class, "wrist_ROT"); //control servo port 1
    
    //Initalize Sensors
    elevator_DISTSENSOR = hardwareMap.get(DistanceSensor.class, "elevatorDistance");
    
    //--These wheels are reversed for desired results--//
    whl_LB.setDirection(DcMotorSimple.Direction.REVERSE);
    whl_LF.setDirection(DcMotorSimple.Direction.REVERSE);
    //--//
    
    arm_ROT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    arm_EXT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    arm_ELEVATOR1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    arm_ELEVATOR2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    arm_ROT.setTargetPosition(Help.degreesToTick(0));
    arm_EXT.setTargetPosition(Help.degreesToTick(0));
    arm_ELEVATOR1.setTargetPosition(Help.degreesToTick(0));
    arm_ELEVATOR2.setTargetPosition(Help.degreesToTick(0));

    arm_ROT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    arm_EXT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    arm_ELEVATOR1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    arm_ELEVATOR2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    arm_ROT.setVelocity(750);
    arm_EXT.setVelocity(750);
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
        }
        // Finished Robot intitalzieaiton//
        ////----INPUTS----////
        if (elevator_HasReset) {
          gamepadInputHandling(now_time);
        }
        
        
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
        telemetry.addData("arm_ext_desiredangle", arm_EXT_angle);
        telemetry.addData("arm_ELEVATOR_angle", arm_ELEVATOR_angle);
        telemetry.addData("arm_ROT_angle", arm_ROT_angle);
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
        float drv_stick_y = gamepad1.left_stick_y;
        float drv_stick_x = gamepad1.left_stick_x;

        if (Math.abs(gamepad1.left_stick_y) > Math.abs(gamepad1.left_stick_x)) {
          whl_LB_percent = drv_stick_y;
          whl_LF_percent = drv_stick_y;
          whl_RB_percent = drv_stick_y;
          whl_RF_percent = drv_stick_y;
        }
        else {
          if (drv_stick_x > 0) {
            whl_RF_percent = drv_stick_x * 1;
            whl_RB_percent = drv_stick_x * -1;
            whl_LF_percent = drv_stick_x * -1;
            whl_LB_percent = drv_stick_x * 1;
          }
          
          if (drv_stick_x < 0) {
            whl_LF_percent = drv_stick_x * -1;
            whl_LB_percent = drv_stick_x * 1;
            whl_RB_percent = drv_stick_x * -1;
            whl_RF_percent = drv_stick_x * 1;
          }
        }
        whl_corrections(); // Corrects/Adjusts power for correct results
        
        //Set power of motors to their corresponding variables
        whl_LB.setPower(whl_LB_percent);
        whl_RB.setPower(whl_RB_percent);
        whl_LF.setPower(whl_LF_percent);
        whl_RF.setPower(whl_RF_percent);
        
        //Set position of arm and claw motors to their corresponding variables.
        
        claw_GRIP.setPosition(claw_GRIP_angle);
        wrist_ROT.setPosition(wrist_ROT_percent);
        arm_EXT.setTargetPosition(-Help.degreesToTick(arm_EXT_angle));
        arm_EXT.setTargetPosition(-Help.degreesToTick(arm_EXT_angle));
        arm_ELEVATOR1.setTargetPosition(+Help.degreesToTick(arm_ELEVATOR_angle));
        arm_ELEVATOR2.setTargetPosition(-Help.degreesToTick(arm_ELEVATOR_angle));

        susan_ROT.setPosition(susan_ROT_percent);
        
        

        
        
        telemetry.update();
      }
    }
    
    
    
  }
  
  public void gamepadInputHandling(double now_time) {
    /*
    if (gamepad1.left_bumper) {
      everything_universalscale = 0.4;
      wheel_universalscale = 0.3;
    }
    else {
      everything_universalscale = 1;
      wheel_universalscale = 0.8;
    }
    if (gamepad1.right_bumper) {
      wheel_equalizerscale = 0.3;
    }
    else {
      wheel_equalizerscale = 0;
    }*/

    //dpad left/right wrist rotation
    if (gamepad1.dpad_left) {
      wrist_ROT_percent = 0.8;
      wrist_ROT_pos += (now_time-last_time);
    }
    else if (gamepad1.dpad_right) {
      wrist_ROT_percent = 0.3;
      wrist_ROT_pos -= (now_time-last_time);
    }
    else {
      wrist_ROT_percent = 0.5;
    }

    //dpad up/down claw open/close
    if (gamepad1.dpad_up) {
      claw_GRIP_angle += 0.5 * (now_time-last_time);
    }
    else if (gamepad1.dpad_down) {
      claw_GRIP_angle -= 0.5 * (now_time-last_time);
    }

    // Y A arm ROT up down
    if (gamepad1.y) {
      arm_ELEVATOR_angle-=400 * (now_time-last_time);
    } 
    else if (gamepad1.a) {
      arm_ELEVATOR_angle+=400 * (now_time-last_time);
    }

    // B X arm EXT forward back
    if (gamepad1.b) {
      arm_EXT_angle-=450 * (now_time-last_time);
    } 
    else if (gamepad1.x) {
      arm_EXT_angle+=450 * (now_time-last_time);
    }

     if (gamepad1.left_bumper) {
      susan_ROT_percent = 0.8;
      susan_ROT_pos += (now_time-last_time);
    }
    else if (gamepad1.right_bumper) {
      susan_ROT_percent = 0.3;
      susan_ROT_pos -= (now_time-last_time);
    }
    else {
      susan_ROT_percent = 0.5;
    }
    
    ////----BOUNDARIES----////


    if (arm_EXT_angle < -123123123) { 
      arm_EXT_angle = 0;
      
    }
    else if (arm_EXT_angle > 123123123) {
      arm_EXT_angle = 1300;
    }

    //Boundaries of the arm vertical rotation
    
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
    }

  }
  public void whl_corrections() {
      whl_RF_percent = (float) (whl_RF_percent * -0.5);
      whl_RB_percent = (float) (whl_RB_percent * -0.6 );
      whl_LF_percent = (float) (whl_LF_percent * -0.5);
      whl_LB_percent = (float) (whl_LB_percent * -0.6);
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
