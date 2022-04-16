package teamcode.common;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintStream;
import java.util.ArrayList;

public class Logger {

    ArrayList<File> loggerFiles = new ArrayList<>();
    ArrayList<String> loggerStrings = new ArrayList<>();
    //ArrayList<BufferedWriter> loggerWriter = new ArrayList<>();
    public Logger(String[] names){

        for(int i = 0; i < names.length; i++){
            if(!names[i].endsWith(".txt")){
                names[i] += ".txt";
            }
            createLogFile(names[i]);
        }
    }

    public void createLogFile(String name) {
        File f = AppUtil.getInstance().getSettingsFile(name);
        loggerFiles.add(f);
        loggerStrings.add("");
//        try {
//            loggerWriter.add(new BufferedWriter(new FileWriter(f)));
//        } catch (IOException e) {
//            e.printStackTrace();
//        }
//        Debug.log(loggerFiles.size());
//        Debug.log(loggerWriter.size());
//        Debug.log(loggerStrings.size());

    }

    //TODO I need to fix this, just use the index one for now
    public void writeToLogString(String fileName, String message){
        for(int i = 0; i < loggerFiles.size(); i++){
            if(fileName.equalsIgnoreCase(loggerFiles.get(i).getName())){
                writeToLogString(i, message);
                break;
            }
        }
    }

    public void writeToLogString(int index, String message){
        //File curr = loggerFiles.get(index);
//        try {
//            loggerWriter.get(index).append((message + "\n"));
//        } catch (IOException e) {
//            e.printStackTrace();
//        }
          String curr = loggerStrings.get(index);
        curr += message;
        loggerStrings.set(index, curr);
//
    }


    public void writeCoordinatesToLogString(int index, double x, double y){
        String curr = loggerStrings.get(index);
        curr += x + ", " + y + "\n";
        loggerStrings.set(index, curr);
    }


    public void clearLoggerStrings(){
        loggerStrings.clear();
        for(int i = 0; i < loggerFiles.size(); i++){
//            try {
//                loggerWriter.get(i).flush();
//            } catch (IOException e) {
//                e.printStackTrace();
//            }
        }
    }

    public void writeLoggerToFile() {
        for (int i = 0; i < loggerFiles.size(); i++) {
            try {
                PrintStream ps = new PrintStream(loggerFiles.get(i));
                ps.println(loggerStrings.get(i));
            } catch (FileNotFoundException e) {
                e.printStackTrace();
            }
        }
    }

    public void resetLogger(){
        loggerFiles.clear();
        loggerStrings.clear();
    }

}
