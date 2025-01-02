package org.firstinspires.ftc.teamcode.wrappers;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.json.JSONException;

import java.util.List;

public class AutoAlign {
    public Limelight3A limelight;
    public Servo rotate;

    public AutoAlign(HardwareMap hardwareMap) {
        rotate = hardwareMap.servo.get("rotate");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    public void update(Telemetry tele) throws JSONException {
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
        for(LLResultTypes.ColorResult colorResult: colorResults) {



            List<List<Double>> pts = colorResult.getTargetCorners();
            //colorResult.





            int numPts = pts.size();
            double[] slopes = new double[(numPts*numPts + numPts)/2];
            double[] distances = new double[(numPts*numPts + numPts)/2];
            double[] similarities = new double[(numPts*numPts + numPts)/2];
            for(int i = 0; i < pts.size(); i++) {
                for(int j = i; j < pts.size(); j++) {
                    double yDiff = pts.get(i).get(1) - pts.get(j).get(1);
                    double xDiff = pts.get(i).get(0) - pts.get(j).get(0);
                    double slope = yDiff/xDiff;
                    slopes[i] = (Math.toDegrees(Math.atan(slope)));
                    distances[i] = (Math.sqrt(yDiff*yDiff+xDiff*xDiff));
                }
            }


















            tele.addData("Color Blob #", colorResults.indexOf(colorResult));
            for(List<Double> pt: pts) {
                tele.addData("    x: ", pt.get(0));
                tele.addData("    y: ", pt.get(1));
            }
        }




    }
}
