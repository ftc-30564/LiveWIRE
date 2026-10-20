package org.firstinspires.ftc.teamcode.livewire.path;

import static org.firstinspires.ftc.teamcode.Constants.IS_RED;

import org.firstinspires.ftc.teamcode.livewire.path.points.Waypoint;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.InputStreamReader;

public class PathParser {
    public static final String PATH_DIRECTORY = "/sdcard/FIRST/paths/";

    // Adds red_ or blue_ to the beginning of the path and parses it
    public static Path parse(String pathName, boolean isRed) {
        return parse((isRed ? "red_" : "blue_") + pathName);
    }

    public static Path parse(String pathName) {
        Path ret = new Path();
        File file = new File(PATH_DIRECTORY + pathName + ".json");

        if (!file.exists() || file.length() == 0) {
            throw new RuntimeException("Path file does not exist or is empty: " + file.getAbsolutePath());
        }

        StringBuilder sb = new StringBuilder();

        try {
            FileInputStream fis = new FileInputStream(file);
            BufferedReader reader = new BufferedReader(new InputStreamReader(fis, "UTF-8"));
            String line;
            while ((line = reader.readLine()) != null) {
                sb.append(line).append("\n");
            }
            reader.close();
            fis.close();
        } catch (Exception e) {
            throw new RuntimeException("Failed to read file: " + file.getAbsolutePath(), e);
        }

        String json = sb.toString();

        try {
            JSONObject rootObject = new JSONObject(json);
            JSONArray waypoints = rootObject.getJSONArray("waypoints");

            for (int i = 0; i < waypoints.length(); i++) {
                ret.addWaypoint(new Waypoint(waypoints.getJSONObject(i)));
            }
        } catch (Exception e) {
            throw new RuntimeException("Invalid JSON format in file: " + file.getAbsolutePath(), e);
        }

        return ret;
    }
}