package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="ServoTest")
public class IntakeSystem extends OpMode {
    private Servo intakeServoLeft;
    private Servo intakeServoRight;
    private Servo horizontalSlideLeft;
    private Servo horizontalSlideRight;
    private CRServo passoverServoLeft;
    private CRServo passoverServoRight;

    /*toggle works poorly because you have to be really quick with the input so it doesn't
     double on it. AKA it goes from being false to true and back to being false
     */

    boolean toggleExtend = false;
    boolean togglePassover = false;

    @Override
    public void init() {
        intakeServoLeft = hardwareMap.get(Servo.class, "servo0");
        intakeServoRight = hardwareMap.get(Servo.class, "servo1");
        horizontalSlideLeft = hardwareMap.get(Servo.class, "servo2");
        horizontalSlideRight = hardwareMap.get(Servo.class, "servo3");
        passoverServoLeft = hardwareMap.get(CRServo.class, "passoverServo4");
        passoverServoRight = hardwareMap.get(CRServo.class,"passoverServo5");

        intakeServoLeft.setDirection(Servo.Direction.REVERSE);
        horizontalSlideRight.setDirection(Servo.Direction.REVERSE); // this aint a typo it supposed to be right idk why
        passoverServoLeft.setDirection(CRServo.Direction.REVERSE);

//        horizontalSlideRight.scaleRange(0.2,0.8);
    }

    @Override
    public void loop() {
        //intake
        if (gamepad1.right_trigger > 0) {
            intakeServoLeft.setPosition(1);
            intakeServoRight.setPosition(1);
        }

        //outtake
        if (gamepad1.left_trigger > 0) {
            intakeServoLeft.setPosition(0);
            intakeServoRight.setPosition(0);
        }

        // extend
        if (gamepad1.a) {
            //SUBJECT TO CHANGE DUE TO MECHANICAL AND SERVO RANGE
            //also set servo range to the specific rotation we need

            horizontalSlideRight.setPosition(0.5);
            horizontalSlideLeft.setPosition(0.5);
       /*
       Should make this a sequence from returning to reset position to passing to outtake so we
       we don't need to press an input for each individual part
       */
        }

        // retract
        if (gamepad1.b) {
            horizontalSlideLeft.setPosition(0.3);
            horizontalSlideRight.setPosition(0.3);
        }
        //intake position
        if (gamepad2.right_trigger>0){
            passoverServoLeft.setPower(0.8);
            passoverServoRight.setPower(0.8);
        } else {
            passoverServoLeft.setPower(0);
            passoverServoRight.setPower(0);
        }
        //passover position
        if (gamepad2.left_trigger>0){
            passoverServoRight.setPower(-1);
            passoverServoLeft.setPower(-1);
        } else {
            passoverServoLeft.setPower(0);
            passoverServoRight.setPower(0);
        }

        telemetry.addData("Horizontal Slide Left Position",horizontalSlideLeft.getPosition());
        telemetry.addData("Horizontal Slide Right Position",horizontalSlideRight.getPosition());
//        telemetry.addData("Passover Slide Left Position",passoverServoLeft.getPosition());
//        telemetry.addData("Passover Slide Right Position",passoverServoRight.getPosition());
        telemetry.update();
    }
}

