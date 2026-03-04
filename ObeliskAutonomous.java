package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.json.JSONArray;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URL;

@Autonomous(name="Auto Align + Obelisk", group="Main")
public class ObeliskAutonomous extends LinearOpMode {

    private static final double TURN_KP = 0.02;
    private static final double TURN_MAX = 0.4;

    private String httpGet(String urlStr) {
        try {
            URL url = new URL(urlStr);
            HttpURLConnection conn = (HttpURLConnection) url.openConnection();
            conn.setRequestMethod("GET");
            conn.setConnectTimeout(50);
            conn.setReadTimeout(50);

            BufferedReader in = new BufferedReader(new InputStreamReader(conn.getInputStream()));
            StringBuilder response = new StringBuilder();
            String line;

            while ((line = in.readLine()) != null) {
                response.append(line);
            }

            in.close();
            return response.toString();

        } catch (Exception e) {
            return "";
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor fl = hardwareMap.dcMotor.get("fl");
        DcMotor bl = hardwareMap.dcMotor.get("bl");
        DcMotor fr = hardwareMap.dcMotor.get("fr");
        DcMotor br = hardwareMap.dcMotor.get("br");

        DcMotor fw1 = hardwareMap.dcMotor.get("fw1");
        DcMotor fw2 = hardwareMap.dcMotor.get("fw2");
        DcMotor fw3 = hardwareMap.dcMotor.get("fw3");

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        fw1.setDirection(DcMotorSimple.Direction.REVERSE);
        fw2.setDirection(DcMotorSimple.Direction.FORWARD);
        fw3.setDirection(DcMotorSimple.Direction.FORWARD);

        fw1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fw2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fw3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        if (isStopRequested()) return;

        // ---------------------------------------------------------
        // STEP 1: Move backward for 1.5 seconds
        // ---------------------------------------------------------
        fl.setPower(-0.4);
        bl.setPower(-0.4);
        fr.setPower(-0.4);
        br.setPower(-0.4);

        sleep(1500);

        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);

        // ---------------------------------------------------------
        // STEP 2: Auto-align using AprilTag (Limelight)
        // ---------------------------------------------------------
        boolean aligned = false;
        long alignStart = System.currentTimeMillis();

        while (opModeIsActive() && !aligned && System.currentTimeMillis() - alignStart < 3000) {

            boolean tagFound = false;
            double tx = 0;

            try {
                String json = httpGet("http://172.28.0.1:5807/llpython");

                if (!json.isEmpty()) {
                    JSONArray arr = new JSONArray(json);

                    tagFound = arr.getDouble(0) == 1.0;
                    tx = arr.getDouble(1);
                }
            } catch (Exception ignored) {}

            if (tagFound) {
                double turnPower = tx * TURN_KP;
                turnPower = Math.max(-TURN_MAX, Math.min(TURN_MAX, turnPower));

                fl.setPower(turnPower);
                bl.setPower(turnPower);
                fr.setPower(-turnPower);
                br.setPower(-turnPower);

                if (Math.abs(tx) < 1.0) {
                    aligned = true;
                }

            } else {
                fl.setPower(0);
                bl.setPower(0);
                fr.setPower(0);
                br.setPower(0);
            }

            telemetry.addData("Aligning...", tx);
            telemetry.update();
        }

        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);

        // ---------------------------------------------------------
        // STEP 3: Spin fw1 + fw2 to 6000 RPM
        // ---------------------------------------------------------
        fw1.setPower(1.0);
        fw2.setPower(1.0);

        sleep(1500);

        // ---------------------------------------------------------
        // STEP 4: After 1.5 seconds, turn on fw3
        // ---------------------------------------------------------
        fw3.setPower(1.0);

        // Autonomous ends here — motors stay running
        while (opModeIsActive()) {
            telemetry.addLine("Autonomous Complete");
            telemetry.update();
        }
    }
}
