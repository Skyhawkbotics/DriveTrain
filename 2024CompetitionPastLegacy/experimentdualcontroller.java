
/*
Packages and Imports used for the code.
*/
package org.firstinspires.ftc.teamcode                                 ;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode            ;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DigitalChannel                  ;
import com.qualcomm.robotcore.hardware.Servo                           ;
import com.qualcomm.robotcore.util.ElapsedTime                         ;
import com.qualcomm.robotcore.hardware.DcMotorEx                       ;
import com.qualcomm.robotcore.hardware.DistanceSensor                  ;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp                  ;
import com.qualcomm.robotcore.hardware.DcMotor                         ;
import com.qualcomm.robotcore.hardware.DcMotorSimple                   ;
import com.qualcomm.robotcore.hardware.TouchSensor                     ;
import com.qualcomm.robotcore.hardware.CRServo                         ;

import java.util.*                                                     ;
import java.lang.Math                                                  ;

/*
String array which holds all ports
Number array which holds all power/positions
String dictionary which holds all inputs and their corresponding output and their multiplying factor
*/

@TeleOp(name = "2022-2023fullcode")
public class mechanumdrive extends LinearOpMode {
  //Clock
  private ElapsedTime runtime = new ElapsedTime();
  
  //Create input dictionary
  private Hashtable<String, boolean> gamepad1_INPUT_DICTIONARY = new Hashtable<String, boolean>(); // NOT IN USE YET
  //gamepad1_INPUT_DICTIONARY.put("a", gamepad1.a);
  
  //Create devices
  private HashMap<String, device> devices = new HashMap<String, device>();
  //devices.put("A", "e");

  private DistanceSensor elevator_DISTSENSOR;
  double last_time = runtime.seconds(); //Used to find how much time has elapsed per iteration in the runtime loop.
  //private DistanceSensor distance;
  @Override
  public void runOpMode() {
    
    devices.put("whl_LB", new device("DcMotor", "float", "whl_LB"));
          devices.get("whl_LB").object.setDirection(DcMotorSimple.Direction.REVERSE);
  devices.put("whl_LF", new device("DcMotor", "float", "whl_LF"))                   ;
          devices.get("whl_LF").object.setDirection(DcMotorSimple.Direction.REVERSE);
  devices.put("whl_RB", new device("DcMotor", "float", "whl_RB"));
  devices.put("whl_RF", new device("DcMotor", "float", "whl_RF"));
  
  int[] ELEVATOR_properties = {1200};
  int[]
  devices.put("arm_ELEVATOR1", new device("DcMotorEx", "float", "arm_ELEVATOR1", properties ));
  devices.put("arm_ELEVATOR2", new device("DcMotorEx", "float", "arm_ELEVATOR2", properties ));
          devices.get("arm_ELEVATOR2").object.setDirection(DcMotorSimple.Direction.REVERSE);
  devices.put("arm_ROT",       new device("DcMotorEx", "float", "arm_ROT",       int[] properties = {400}  ));

  devices.put("arm_EXT",   new device("Servo", "double", "arm_EXT"  ));
  devices.put("susan_ROT", new device("Servo", "double", "susan_ROT"));

  devices.put("claw_GRIP", new device("CRServo", "double", "claw_GRIP"));
  devices.put("wrist_ROT", new device("CRServo", "double", "wrist_ROT"));
    waitForStart();
    if (opModeIsActive()) {
      while (opModeIsActive()) {
        //now_time, the time since the start of the program and is used to find time differentials between loop iterations
        double now_time = runtime.seconds();
        gamepadInputHandling(now_time);
        last_time = now_time; //To find time differentials between loops.
        
        telemetry.addData("elevatordist", elevator_DISTSENSOR.getDistance(DistanceUnit.CM));
        telemetry.update();
        
        //Wheels
        boolean dif = Math.abs((gamepad1.left_stick_y+gamepad1.left_stick_x))>Math.abs((gamepad1.right_stick_x+gamepad1.right_stick_y));
        
        float drv_stick_y2 = gamepad1.right_stick_y;
        float drv_stick_x2 = gamepad1.right_stick_x;
        if (!dif) {
          if (Math.abs(gamepad1.right_stick_y) > Math.abs(gamepad1.right_stick_x)) {
            devices.get("whl_LB").num = drv_stick_y2;
            devices.get("whl_LF").num = drv_stick_y2;
            devices.get("whl_RB").num = drv_stick_y2;
            devices.get("whl_RF").num = drv_stick_y2;
          }
          else {
            if (drv_stick_x2 > 0) {
              devices.get("whl_RF").num = drv_stick_x2 * 1;
              devices.get("whl_RB").num = drv_stick_x2 * -1;
              devices.get("whl_LF").num = drv_stick_x2 * -1;
              devices.get("whl_LB").num = drv_stick_x2 * 1;
            }
            
            if (drv_stick_x2 < 0) {
              devices.get("whl_RF").num = drv_stick_x2 * -1;
              devices.get("whl_LB").num = drv_stick_x2 * 1;
              devices.get("whl_RB").num = drv_stick_x2 * -1;
              devices.get("whl_RF").num = drv_stick_x2 * 1;
            }
          }
        }
        else {
          float drv_stick_y = gamepad1.left_stick_y * 0.2f;
          float drv_stick_x = gamepad1.left_stick_x;
          
          devices.get("whl_LB").num = drv_stick_y - drv_stick_x;
          devices.get("whl_LF").num = drv_stick_y - drv_stick_x;
          devices.get("whl_RB").num = drv_stick_y + drv_stick_x;
          devices.get("whl_RF").num = drv_stick_y + drv_stick_x;
        }
        
        whl_corrections(); // Corrects/Adjusts power for correct results
        
        //Set power of motors to their corresponding variables
        devices.get("whl_LB").setPower();
        devices.get("whl_LF").setPower();
        devices.get("whl_RB").setPower();
        devices.get("whl_RF").setPower();
        
        //Set position of arm and claw motors to their corresponding variables.
        devices.get("claw_GRIP").setPower();
        devices.get("wrist_ROT").setPower();
        devices.get("arm_EXT").setPower();
        devices.get("susan_ROT").setPower();
        devices.get("arm_ROT").setPower();
        devices.get("arm_ELEVATOR1").setPower();
        devices.get("arm_ELEVATOR2").setPower();

        telemetry.update();
      }
    }
  }
  
  public void gamepadInputHandling(double now_time) {

    //CLAW GRIP/RELEASE
    if (claw_gripped) {
      devices.get("claw_GRIP").num = 0.5;
    }
    else if (!claw_gripped) {
      devices.get("claw_GRIP").num = -0.5;
    }

    //ARM EXTENDER
    if (gamepad2.left_trigger) {
      devices.get("arm_EXT").num -= (gamepad2.right_trigger / 2) + 0.5;
    }
    else if (gamepad2.right_trigger) {
      devices.get("arm_EXT").num -= (gamepad2.right_trigger / 2) + 0.5;
    }

    //ARM ROTATION
    if (gamepad2.left_stick_y != 0) {
      devices.get("arm_ROT").num += gamepad2.left_stick_y * 1000 * (now_time-last_time);
    }

    //SUSAN ROTATION
    if (gamepad2.left_stick_x != 0) {
      devices.get("susan_ROT").num = (gamepad2.left_stick_x / 2) + 0.5;

      if (devices.get("susan_ROT").num > 1) {
        devices.get("susan_ROT").num = 1;
      }
      else (devices.get("susan_ROT").num < 0) {
        devices.get("susan_ROT").num = 0;
      }
    }

    //ELEVATOR
    if (gamepad2.right_stick_x != 0) {
      devices.get("arm_ELEVATOR1").num += gamepad2.right_stick_x * 100 * (now_time-last_time);
      devices.get("arm_ELEVATOR1").num += gamepad2.right_stick_x * 100 * (now_time-last_time);
    }

    //WRIST
    if (gamepad2.right_stick_y != 0) {
      devices.get("wrist_ROT").num = (gamepad2.right_stick_y / 2) + 0.5;
    }

  }
  public void whl_corrections() {
      devices.get("whl_RF").num = (float) (devices.get("whl_RF").num * -0.5);
      devices.get("whl_RB").num = (float) (devices.get("whl_RB").num * -0.6 );
      devices.get("whl_LF").num = (float) (devices.get("whl_LF").num * -0.8);
      devices.get("whl_LB").num = (float) (devices.get("whl_LB").num * -0.9);
  }
}
//In input/outputs, be able to loop through and...
/*
  Recognize what variable type to use (float, double) and what kind of device it is (servo, motor)
  Be able to output based on info
  Initialize
  Be given a name
*/
class device {
  private String type;
  private String NUM_TYPE;
  private String name;
  private DcMotor object; //Unstable. Type set during initialization
  private int num;        //Unstable.
  
  public device(String type, String NUM_TYPE, String name, int[] properties) {
    this.type = type;
    this.NUM_TYPE = NUM_TYPE; 
    this.name = name;
    
    //Decide object type
    if (type == "DcMotor") {
      object = null;
      DcMotor object = hardwareMap.get(DcMotor.class, name);
    }
    else if (type == "DcMotorEx") {
      object = null;
      DcMotorEx object = hardwareMap.get(DcMotorEx.class, name);
      object.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      object.setTargetPosition(0);
      object.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      object.setVelocity(properties[0]);
    }
    else if (type == "Servo") {
      object = null;
      Servo object = hardwareMap.get(Servo.class, name);
    }
    else if (type == "CRServo") {
      object = null;
      CRServo object = hardwareMap.get(CRServo.class, name);
    }
    
    //Decide num type
    if (NUM_TYPE == "int") {
      num = null; int num = 0;
    }
    else if (NUM_TYPE == "float") {
      num = null; float num = 0.0;
    }
    else if (NUM_TYPE == "double") {
      num = null; double num = 0.0;
    }
  }
  
  public void setPower() {
    if (this.type == "DcMotor" || this.type == "DcMotorEx") {
        this.object.SetTargetPosition(this.num);
    }
    else if (this.type == "Servo") {
        this.object.SetPosition(this.num);
    }
    else if (this.type == "CRServo") {
        this.object.SetPower(this.num);
    }
        
  }
}
