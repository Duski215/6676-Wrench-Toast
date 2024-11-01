package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "motorEncoders")
public class OuttakeAndHangSystem extends OpMode {
    //outake slides
    private DcMotor motorLeftVert;
    private DcMotor motorRightVert;
    private Servo outtakeServoLeft;
    private Servo outtakeServoRight;
    private Servo gateServo;
    //hang slide
    private DcMotor motorHang;
    double ticksPerRotation = 537.6;
    double wheelCircumference = 11.8737374;
    double spoolCircumference = 4.40945;
    //finds the amount of ticks per inch of rotation
    // 2786.2 ticks /1.4 inch = 1990 ticks per inch of rotation
    double newTarget;
    double midBinHeightInches = 16;
    double topBinHeightInches = 43;
    double midHangInches = 20;
    double topHangInches = 36;

    @Override
    public void init() {
        motorLeftVert = hardwareMap.get(DcMotor.class, "exMotor0");
        motorLeftVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorRightVert = hardwareMap.get(DcMotor.class, "exMotor1");
        motorRightVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorHang = hardwareMap.get(DcMotor.class, "exMotor2");
        motorHang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        outtakeServoLeft = hardwareMap.get(Servo.class,"exServo0");
        outtakeServoRight = hardwareMap.get(Servo.class,"exServo1");
        gateServo = hardwareMap.get(Servo.class,"exServo2");


        motorLeftVert.setDirection(DcMotor.Direction.REVERSE);
        motorHang.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeServoLeft.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if (gamepad2.dpad_up) {
            //telling the motor to rotate one inch
            positionTopOuttake();
        }

        if (gamepad2.dpad_right){
            positionMidOuttake();
        }
        //continues to keep track of ticks after pressing the button, it'll keep the position until a separate input tells it otherwise.

        //bring it back one inch, this will also hold the slides in the same position.
        if (gamepad2.dpad_down) {
            resetOuttakeSlides();
        }

        //hang commands
        if (gamepad2.y) {
            positionTopHang();
        }
        if (gamepad2.a){
            resetHang();
        }
        if (gamepad2.x) {
            positionMidHang();
        }

        //Outtake servo code here
        if (gamepad2.right_trigger > 0){
            outtakeServoLeft.setPosition(0);
            outtakeServoRight.setPosition(0);
        }

        if (gamepad2.right_trigger > 0){
            outtakeServoLeft.setPosition(1);
            outtakeServoRight.setPosition(1);
        }

        //The intake has a gate that prevents it from spitting the sample from it's backside.
        if (gamepad2.left_bumper){
            gateServo.setPosition(1);
        } else {
            gateServo.setPosition(0);
        }

        telemetry.addData("Motor ticks 1", motorLeftVert.getCurrentPosition());
        telemetry.addData("Motor ticks 2", motorRightVert.getCurrentPosition());
        telemetry.addData("Hang Ticks", motorHang.getCurrentPosition());
    }
    //conversions
    private double ticksToInchesSpool(double inches) {
        return inches * ticksPerRotation / spoolCircumference;
    }

    private double ticksToInchesWheel(double inches) {
        return inches * ticksPerRotation / wheelCircumference;
    }

    //Tells the robot the position you want it to go to with how much power.
    public void positionTopOuttake() {
//        motorLeftVert.setDirection(DcMotor.Direction.REVERSE);
//        motorLeftVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorRightVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        newTarget = ticksToInchesSpool(topBinHeightInches);

        motorLeftVert.setTargetPosition((int) newTarget);
        motorRightVert.setTargetPosition((int) newTarget);

        motorLeftVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLeftVert.setPower(0.8);
        motorRightVert.setPower(0.8);
    }

    public void positionMidOuttake() {
//        motorLeftVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorRightVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        newTarget = ticksToInchesSpool(midBinHeightInches);

        motorLeftVert.setTargetPosition((int) newTarget);
        motorRightVert.setTargetPosition((int) newTarget);

        motorLeftVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLeftVert.setPower(0.8);
        motorRightVert.setPower(0.8);
    }


    public void positionMidHang() {
//        motorHang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        newTarget = ticksToInchesSpool(midHangInches);
        motorHang.setTargetPosition((int) newTarget);
        motorHang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorHang.setPower(0.4);
    }

    public void positionTopHang() {
//        motorHang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        newTarget = ticksToInchesSpool(topHangInches);
        motorHang.setTargetPosition((int) newTarget);
        motorHang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorHang.setPower(0.4);
    }

    //reset position
    public void resetOuttakeSlides() {
        motorLeftVert.setTargetPosition(0);
        motorRightVert.setTargetPosition(0);

        motorLeftVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLeftVert.setPower(0.5);
        motorRightVert.setPower(0.5);
    }
    public void resetHang() {
//       motorHang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        /*wheelRevolution = 96mm = 3.78 inches if for wheels
          wheelRevolution will equal inner diameter of spool if for linear spool */
        motorHang.setTargetPosition(0);
        motorHang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorHang.setPower(0.6);
    }

}