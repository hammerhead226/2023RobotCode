package frc.libs.swerveyshark.sharkexe;


import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.function.Consumer;


public class SharkExecutor {

    private static HashMap<String, ArrayList<SharkState>> executables = new HashMap<>();

    private static ArrayList<SharkState> executable;

    private static Consumer<double[]> toPose;

    private static int iterator;

    public static double startTime;
    
    private static HashMap<String, Runnable> runnableHash = new HashMap<>();;

    public static void loadAndConfigurePath(String name, String filePath, Consumer<double[]> toPose) throws IOException {
        BufferedReader reader = null;

        try {
            reader = new BufferedReader(new FileReader(filePath));
        } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
        }

        executable = new ArrayList<>();
        
        iterator = 0;
        SharkExecutor.toPose = toPose;

        // reader.skip(2); I'm manually reformatting rn

        while (reader.ready()) {
            String raw = reader.readLine();

            String[] data = raw.split(",");

            SharkState state = new SharkState(
                    Double.parseDouble(data[0]),
                    Double.parseDouble(data[1]),
                    Double.parseDouble(data[2]),
                    Double.parseDouble(data[3]),
                    Double.parseDouble(data[4]),
                    Double.parseDouble(data[5]),
                    Double.parseDouble(data[6]),
                    data.length > 7 ? data[7] : ""
            );

            executable.add(state);
        }

        reader.close();

        executables.put(name, executable);

    }
    
    public static void createRunnable(String name, Runnable runnable) {
        runnableHash.put(name, runnable);
    }

    public static void executeNextAvailableStep(String name, double currentTime) {
        boolean hasExecuted = false;
        ArrayList<String> eventStack = new ArrayList<>();
        while(!hasExecuted) {
            if(isFinished(name)) break;
            SharkState nexState = executables.get(name).get(iterator);
            if(nexState.hasEvent()) {
                eventStack.add(nexState.getEvent());
            }
            iterator++;
            if (nexState.getTime() < currentTime) {
                continue;
            }

            for(String event : eventStack) {
                if(runnableHash.containsKey(event)) {
                    runnableHash.get(event).run();
                }
            }

            

            toPose.accept(nexState.getAsArray());
            

            hasExecuted = true;
        }
    }

    public static boolean isFinished(String name) {
        return iterator >= executables.get(name).size();
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
