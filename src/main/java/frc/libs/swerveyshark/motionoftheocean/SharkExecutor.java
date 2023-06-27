package frc.libs.swerveyshark.motionoftheocean;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.Consumer;

public class SharkExecutor {

    private static ArrayList<SharkState> executable;

    private static Consumer<double[]> toPose;

    private static int iterator;

    public static double startTime;

    // TODO:: use this in autons dont forget to throw IOException lol with a try catch 
    public static void loadAndConfigurePath(String filePath, Consumer<double[]> toPose) throws IOException {
        BufferedReader reader = null;

        try {
            reader = new BufferedReader(new FileReader(filePath));
        } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
        }

        executable = new ArrayList<>();
        iterator = 0;
        SharkExecutor.toPose = toPose;

        reader.skip(2);

        while (reader.ready()) {
            String raw = reader.readLine();

            String[] data = raw.split(",");

            SharkState state = new SharkState(
                Double.parseDouble(data[0]),
                Double.parseDouble(data[1]),
                Double.parseDouble(data[2]),
                Double.parseDouble(data[3]),
                Double.parseDouble(data[4]),
                Double.parseDouble(data[5])
            );

            executable.add(state);
        }

        reader.close();

    }

    public static void executeNextAvailableStep(double currentTime) {
        boolean hasExecuted = false;

        while(!hasExecuted) {
            SharkState nexState = executable.get(iterator);
            iterator++;
            if (nexState.getTime() < currentTime) {
                continue;
            }

            toPose.accept(nexState.getAsArray());

            hasExecuted = true;
        }
    }

    public static boolean isFinished() {
        return iterator >= executable.size();
    }

    public static double[] toPose(double[] state) {
        return state;
    }

    public static SharkState getState() {
        return executable.get(iterator);
    }
    
    // TODO:: use this method at the start of every auton 1st line ooga
    public static void setStartTime(double time) {
        startTime = time;
    }
}
