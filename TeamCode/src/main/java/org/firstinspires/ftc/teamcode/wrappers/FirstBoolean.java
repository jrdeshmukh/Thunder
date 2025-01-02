package org.firstinspires.ftc.teamcode.wrappers;

public class FirstBoolean {
    boolean recent = false;
    boolean returnBool = false;
    public boolean betterboolean(boolean input) {
        returnBool = !recent && input;
        recent = input;
        return returnBool;
    }

}