package org.firstinspires.ftc.teamcode.system;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import fi.iki.elonen.NanoHTTPD;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;


// Server that runs on the robot to connect with the Overlake Robotics Path Planner and allow
// path uploading.
@Config
public class PathServer extends NanoHTTPD {
    private static final int PORT = 8099;
    private static final String JSON = "application/json";

    public static volatile double[][] RAW_POINTS = {{0, 0, 0}};
    public static volatile double VELOCITY_IN_S = 0.0;
    public static volatile double TOLERANCE_IN  = 0.0;
    public static volatile String ALLIANCE = "unknown";

    private static volatile double ROBOT_X_IN  = 0.0;
    private static volatile double ROBOT_Y_IN  = 0.0;
    private static volatile double ROBOT_H_DEG = 0.0;
    private static volatile long ROBOT_TS_MS = 0L;

    public static final class Tag implements Comparable<Tag> {
        public final int index;
        public final String name;
        public final double value;

        public Tag(String name, double value, int index) {
            this.name = name;
            this.value = value;
            this.index = index;
        }

        @Override public int compareTo(Tag other) { return this.index - other.index; }
    }

    private static volatile Tag[] TAGS = new Tag[0];
    private static volatile PathServer instance;

    // Starts the embedded HTTP server.
    public static void startServer() {
        if (instance != null) return;
        try {
            instance = new PathServer();
            System.out.println("PathServer started on port " + PORT);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    // Stops the embedded HTTP server.
    public static void stopServer() {
        PathServer s = instance;
        if (s == null) return;
        s.stop();
        instance = null;
        System.out.println("PathServer stopped.");
    }

    // Returns the uploaded path points as Pose2D waypoints, excluding the start pose.
    public static Pose2D[] getPath() {
        double[][] pts = RAW_POINTS;
        if (pts == null || pts.length <= 1) return new Pose2D[0];

        Pose2D[] poses = new Pose2D[pts.length - 1];
        for (int i = 1; i < pts.length; i++) {
            poses[i - 1] = new Pose2D(
                    DistanceUnit.INCH, pts[i][0], pts[i][1],
                    AngleUnit.DEGREES, pts[i][2]
            );
        }
        return poses;
    }

    // Returns the current start pose (index 0 of RAW_POINTS), or (0,0,0) if missing.
    public static Pose2D getStartPose() {
        double[][] pts = RAW_POINTS;
        if (pts == null || pts.length == 0) {
            return new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
        }
        double[] s = pts[0];
        return new Pose2D(DistanceUnit.INCH, s[0], s[1], AngleUnit.DEGREES, s[2]);
    }

    // Returns the configured path velocity.
    public static double getVelocity() {
        return VELOCITY_IN_S;
    }

    // Returns the configured path tolerance.
    public static double getTolerance() {
        return TOLERANCE_IN;
    }

    // Returns a snapshot of the current tags array.
    public static Tag[] getTags() {
        Tag[] src = TAGS;
        Tag[] copy = new Tag[src.length];
        System.arraycopy(src, 0, copy, 0, src.length);
        return copy;
    }

    // Returns the currently selected alliance string.
    public static String getAlliance() {
        return ALLIANCE;
    }

    // Updates the robot pose reported by the /pose endpoint.
    public static void setRobotPose(Pose2D pose) {
        ROBOT_X_IN  = pose.getX(DistanceUnit.INCH);
        ROBOT_Y_IN  = pose.getY(DistanceUnit.INCH);
        ROBOT_H_DEG = pose.getHeading(AngleUnit.DEGREES);
        ROBOT_TS_MS = System.currentTimeMillis();
    }

    // Handles HTTP requests for points upload and telemetry/config reads.
    @Override
    public Response serve(IHTTPSession session) {
        try {
            if (Method.OPTIONS.equals(session.getMethod())) return preflightResponse();

            String uri = session.getUri();
            Method method = session.getMethod();

            if (Method.POST.equals(method) && "/points".equals(uri)) {
                JSONObject payload = readJsonBody(session);
                applyPayload(payload);

                try { FtcDashboard.getInstance().updateConfig(); } catch (Throwable ignored) {}

                JSONObject ok = new JSONObject()
                        .put("ok", true)
                        .put("points", RAW_POINTS.length)
                        .put("velocity", VELOCITY_IN_S)
                        .put("tolerance", TOLERANCE_IN)
                        .put("tags", TAGS.length);
                return withCors(newFixedLengthResponse(Response.Status.OK, JSON, ok.toString()));
            }

            if (Method.GET.equals(method) && "/pose".equals(uri)) {
                JSONObject out = new JSONObject()
                        .put("ok", true)
                        .put("x", ROBOT_X_IN)
                        .put("y", ROBOT_Y_IN)
                        .put("h", ROBOT_H_DEG)
                        .put("t", ROBOT_TS_MS);
                return withCors(newFixedLengthResponse(Response.Status.OK, JSON, out.toString()));
            }

            if (Method.GET.equals(method) && "/start".equals(uri)) {
                Pose2D start = getStartPose();
                JSONObject out = new JSONObject()
                        .put("ok", true)
                        .put("x", start.getX(DistanceUnit.INCH))
                        .put("y", start.getY(DistanceUnit.INCH))
                        .put("h", start.getHeading(AngleUnit.DEGREES));
                return withCors(newFixedLengthResponse(Response.Status.OK, JSON, out.toString()));
            }

            if (Method.GET.equals(method) && "/config".equals(uri)) {
                JSONObject out = new JSONObject()
                        .put("ok", true)
                        .put("velocity", VELOCITY_IN_S)
                        .put("tolerance", TOLERANCE_IN)
                        .put("alliance", ALLIANCE)
                        .put("tags", tagsToJson(TAGS));
                return withCors(newFixedLengthResponse(Response.Status.OK, JSON, out.toString()));
            }

            return withCors(newFixedLengthResponse(Response.Status.NOT_FOUND, "text/plain", "not found"));
        } catch (Exception e) {
            e.printStackTrace();
            return withCors(newFixedLengthResponse(
                    Response.Status.INTERNAL_ERROR,
                    "text/plain",
                    "error: " + e.getMessage()
            ));
        }
    }

    // Creates and starts the NanoHTTPD server instance.
    private PathServer() throws IOException {
        super(PORT);
        start(SOCKET_READ_TIMEOUT, false);
    }

    // Parses the request body as JSON and returns the root object.
    private static JSONObject readJsonBody(IHTTPSession session) throws IOException, ResponseException, JSONException {
        Map<String, String> files = new HashMap<>();
        session.parseBody(files);

        String body = files.get("postData");
        if (body == null) body = "";
        body = body.trim();

        if (!body.startsWith("{")) throw new IllegalArgumentException("Expected JSON object body");
        return new JSONObject(body);
    }

    // Applies the uploaded JSON payload to RAW_POINTS/config fields.
    private static void applyPayload(JSONObject obj) throws JSONException {
        JSONArray start = obj.optJSONArray("start");
        double sx = (start != null) ? start.optDouble(0, 0.0) : 0.0;
        double sy = (start != null) ? start.optDouble(1, 0.0) : 0.0;
        double sh = (start != null) ? start.optDouble(2, 0.0) : 0.0;

        JSONArray ptsArr = obj.optJSONArray("points");
        int n = (ptsArr != null) ? ptsArr.length() : 0;

        double[][] pts = new double[n + 1][3];
        pts[0][0] = sx; pts[0][1] = sy; pts[0][2] = sh;

        for (int i = 0; i < n; i++) {
            JSONArray p = ptsArr.getJSONArray(i);
            pts[i + 1][0] = p.optDouble(0, 0.0);
            pts[i + 1][1] = p.optDouble(1, 0.0);
            pts[i + 1][2] = (p.length() >= 3) ? p.optDouble(2, 0.0) : 0.0;
        }
        RAW_POINTS = pts;

        VELOCITY_IN_S = obj.optDouble("velocity", VELOCITY_IN_S);
        TOLERANCE_IN  = obj.optDouble("tolerance", TOLERANCE_IN);

        String alliance = obj.optString("alliance", ALLIANCE);
        if (alliance == null || alliance.trim().isEmpty()) alliance = "unknown";
        ALLIANCE = alliance.trim().toLowerCase();

        JSONArray tagsArr = obj.optJSONArray("tags");
        if (tagsArr == null) {
            TAGS = new Tag[0];
            return;
        }

        Tag[] out = new Tag[tagsArr.length()];
        for (int i = 0; i < tagsArr.length(); i++) {
            JSONObject t = tagsArr.getJSONObject(i);
            out[i] = new Tag(
                    t.optString("name", ""),
                    t.optDouble("value", 0.0),
                    t.optInt("index", 0)
            );
        }
        TAGS = out;
    }

    // Converts the tag array into a JSON array for the /config endpoint.
    private static JSONArray tagsToJson(Tag[] tags) throws JSONException {
        JSONArray arr = new JSONArray();
        for (Tag t : tags) {
            arr.put(new JSONObject()
                    .put("index", t.index)
                    .put("name", t.name)
                    .put("value", t.value));
        }
        return arr;
    }

    // Builds the CORS response for browser preflight requests.
    private Response preflightResponse() {
        Response r = newFixedLengthResponse(Response.Status.NO_CONTENT, "text/plain", "");
        r.addHeader("Access-Control-Allow-Origin", "*");
        r.addHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
        r.addHeader("Access-Control-Allow-Headers", "Content-Type");
        r.addHeader("Access-Control-Allow-Private-Network", "true");
        r.addHeader("Access-Control-Max-Age", "600");
        return r;
    }

    // Adds CORS headers to a normal response.
    private Response withCors(Response r) {
        r.addHeader("Access-Control-Allow-Origin", "*");
        r.addHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
        r.addHeader("Access-Control-Allow-Headers", "Content-Type");
        r.addHeader("Access-Control-Allow-Private-Network", "true");
        return r;
    }
}
