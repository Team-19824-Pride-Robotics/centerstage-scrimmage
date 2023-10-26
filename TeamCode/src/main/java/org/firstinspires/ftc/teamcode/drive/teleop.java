package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp(group = "drive")
@Config
public class teleop extends OpMode {

    //lift
    public static int top = 0;
    public static int bottom = 100;
    public static int var  = 100;
    public static double power = 1;

//motor
    private DcMotorEx lift;
    private DcMotorEx RF;
    private DcMotorEx RB;
    private DcMotorEx LF;
    private DcMotorEx LB;


    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //drive
        RF = hardwareMap.get(DcMotorEx.class, "RF");
        RB= hardwareMap.get(DcMotorEx.class, "RB");
        LF= hardwareMap.get(DcMotorEx.class, "LF");
        LB = hardwareMap.get(DcMotorEx.class, "LB");

        LF.setDirection(DcMotorEx.Direction.REVERSE);
        LB.setDirection(DcMotorEx.Direction.REVERSE);


        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

/*        //lift
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
*/


    }

    @Override
    public void loop() {
//drive
        double drive = -gamepad1.left_stick_y;
        double rotate = gamepad1.right_stick_x;
        double d_power = .8-.4*gamepad1.left_trigger+(.5*gamepad1.right_trigger);

        RF.setPower(drive + rotate);
        RB.setPower(drive + rotate);
        LF.setPower(drive + rotate);
        LB.setPower(drive + rotate);

        if (gamepad1.dpad_up) {
            RF.setPower(d_power);
            RB.setPower(d_power);
            LF.setPower(d_power);
            LB.setPower(d_power);
        }
        if (gamepad1.dpad_down) {
            RF.setPower(-d_power);
            RB.setPower(-d_power);
            LF.setPower(-d_power);
            LB.setPower(-d_power);
        }
        if (gamepad1.dpad_left) {
            RF.setPower(d_power);
            RB.setPower(-d_power);
            LF.setPower(-d_power);
            LB.setPower(d_power);
        }
        if (gamepad1.dpad_right) {
            RF.setPower(-d_power);
            RB.setPower(d_power);
            LF.setPower(d_power);
            LB.setPower(-d_power);
        }
/*

        //lift
        if (gamepad2.y) {
            top = top + var;
        }
        if (gamepad2.a) {
            top = top - var;
        }

        if (gamepad2.b) {
            lift.setTargetPosition(top);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(power);
        }
        if (gamepad2.x) {
            lift.setTargetPosition(bottom);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(power);
        }

        telemetry.addData("lift pos", lift.getCurrentPosition());
        telemetry.addData("lift power", lift.getPower());
        telemetry.update(); */
    }


}