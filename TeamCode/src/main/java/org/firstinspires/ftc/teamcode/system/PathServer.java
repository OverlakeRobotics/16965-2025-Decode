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

@Config
public class PathServer extends NanoHTTPD {
    private static final int PORT = 8099;

    // ---- Path data (start + points) ----
    // RAW_POINTS[0] is always the START pose [x,y,h]. The rest are user points.
    public static double[][] RAW_POINTS = {{0, 0, 0}};
    public static double     VELOCITY_IN_S = 0.0;
    public static double     TOLERANCE_IN  = 0.0;

    // ---- Realtime robot pose (for webapp polling) ----
    private static volatile double ROBOT_X_IN  = 0.0;
    private static volatile double ROBOT_Y_IN  = 0.0;
    private static volatile double ROBOT_H_DEG = 0.0;
    private static volatile long   ROBOT_TS_MS = 0L;

    // ---- Tags ----
    public static class Tag implements Comparable<Tag> {
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
    private static Tag[] TAGS = new Tag[0];

    private static PathServer instance;

    private PathServer() throws IOException {
        super(PORT);
        start(SOCKET_READ_TIMEOUT, false);
    }

    // ---- Server lifecycle ----
    public static void startServer() {
        if (instance == null) {
            try {
                instance = new PathServer();
                System.out.println("PathServer started on port " + PORT);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }
    public static void stopServer() {
        if (instance != null) {
            instance.stop();
            instance = null;
            System.out.println("PathServer stopped.");
        }
    }

    // ---- Public API for robot code ----
    public static Pose2D[] getPath() {
        Pose2D[] poses = new Pose2D[RAW_POINTS.length - 1];
        for (int i = 1; i < RAW_POINTS.length; i++) {
            poses[i - 1] = new Pose2D(
                    DistanceUnit.INCH,
                    RAW_POINTS[i][0],
                    RAW_POINTS[i][1],
                    AngleUnit.DEGREES,
                    RAW_POINTS[i][2]
            );
        }
        return poses;
    }

    /** Returns just the start pose (index 0). */
    public static Pose2D getStartPose() {
        if (RAW_POINTS == null || RAW_POINTS.length == 0) {
            return new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
        }
        double[] s = RAW_POINTS[0];
        return new Pose2D(DistanceUnit.INCH, s[0], s[1], AngleUnit.DEGREES, s[2]);
    }

    /** Optional raw access if you need numbers directly. */
    public static double[] getStartAsArray() {
        if (RAW_POINTS == null || RAW_POINTS.length == 0) return new double[]{0,0,0};
        return new double[]{ RAW_POINTS[0][0], RAW_POINTS[0][1], RAW_POINTS[0][2] };
    }

    public static double getVelocity() { return VELOCITY_IN_S; }
    public static double getTolerance(){ return TOLERANCE_IN; }
    public static Tag[]  getTags() {
        Tag[] copy = new Tag[TAGS.length];
        System.arraycopy(TAGS, 0, copy, 0, TAGS.length);
        return copy;
    }

    public static void setRobotPose(Pose2D pose) {
        ROBOT_X_IN  = pose.getX(DistanceUnit.INCH);
        ROBOT_Y_IN  = pose.getY(DistanceUnit.INCH);
        ROBOT_H_DEG = pose.getHeading(AngleUnit.DEGREES);
        ROBOT_TS_MS = System.currentTimeMillis();
    }

    // ---- HTTP handling ----
    @Override
    public Response serve(IHTTPSession session) {
        try {
            // CORS preflight
            if (Method.OPTIONS.equals(session.getMethod())) return preflightResponse();

            final String uri = session.getUri();
            final Method method = session.getMethod();

            // Upload path & params (accepts the new object format and legacy array)
            if (Method.POST.equals(method) && "/points".equals(uri)) {
                Map<String, String> files = new HashMap<>();
                session.parseBody(files);
                String body = String.valueOf(files.get("postData"));
                if (body == null) body = "";
                body = body.trim();

                if (body.startsWith("{")) {
                    applyObjectPayload(new JSONObject(body));       // NEW format
                } else if (body.startsWith("[")) {
                    applyLegacyArrayPayload(new JSONArray(body));   // Legacy array of [x,y,h]
                } else {
                    throw new IllegalArgumentException("Unsupported body format");
                }

                // Push latest to Dashboard config view (optional)
                try { FtcDashboard.getInstance().updateConfig(); } catch (Throwable ignored) {}

                JSONObject ok = new JSONObject()
                        .put("ok", true)
                        .put("points", RAW_POINTS.length)
                        .put("velocity", VELOCITY_IN_S)
                        .put("tolerance", TOLERANCE_IN)
                        .put("tags", TAGS.length);
                return withCors(newFixedLengthResponse(Response.Status.OK, "application/json", ok.toString()));
            }

            // Live robot pose for polling from the webapp
            if (Method.GET.equals(method) && "/pose".equals(uri)) {
                JSONObject out = new JSONObject()
                        .put("ok", true)
                        .put("x", ROBOT_X_IN)
                        .put("y", ROBOT_Y_IN)
                        .put("h", ROBOT_H_DEG)   // degrees
                        .put("t", ROBOT_TS_MS);  // epoch ms
                return withCors(newFixedLengthResponse(Response.Status.OK, "application/json", out.toString()));
            }

            // NEW: Start pose as a lightweight GET endpoint
            if (Method.GET.equals(method) && "/start".equals(uri)) {
                double[] s = getStartAsArray();
                JSONObject out = new JSONObject()
                        .put("ok", true)
                        .put("x", s[0])
                        .put("y", s[1])
                        .put("h", s[2]);
                return withCors(newFixedLengthResponse(Response.Status.OK, "application/json", out.toString()));
            }

            // Optional: quick peek at config (velocity/tolerance/tags)
            if (Method.GET.equals(method) && "/config".equals(uri)) {
                JSONObject out = new JSONObject()
                        .put("ok", true)
                        .put("velocity", VELOCITY_IN_S)
                        .put("tolerance", TOLERANCE_IN)
                        .put("tags", toJson(TAGS));
                return withCors(newFixedLengthResponse(Response.Status.OK, "application/json", out.toString()));
            }

            return withCors(newFixedLengthResponse(Response.Status.NOT_FOUND, "text/plain", "not found"));
        } catch (Exception e) {
            e.printStackTrace();
            return withCors(newFixedLengthResponse(Response.Status.INTERNAL_ERROR, "text/plain", "error: " + e.getMessage()));
        }
    }

    // ---- Parsers for new & legacy upload ----
    private static void applyObjectPayload(JSONObject obj) throws JSONException {
        // start: [x, y, h]
        JSONArray start = obj.optJSONArray("start");
        double sx = 0.0, sy = 0.0, sh = 0.0;
        if (start != null) {
            sx = start.optDouble(0, 0.0);
            sy = start.optDouble(1, 0.0);
            sh = start.optDouble(2, 0.0);
        }

        // points: [[x,y,h], ...]
        JSONArray ptsArr = obj.optJSONArray("points");
        int n = (ptsArr != null) ? ptsArr.length() : 0;

        // RAW_POINTS = [ start, ...points ]
        double[][] pts = new double[n + 1][3];
        pts[0][0] = sx; pts[0][1] = sy; pts[0][2] = sh;

        for (int i = 0; i < n; i++) {
            JSONArray p = ptsArr.getJSONArray(i);
            // robust to missing heading slot
            double px = p.optDouble(0, 0.0);
            double py = p.optDouble(1, 0.0);
            double ph = (p.length() >= 3) ? p.optDouble(2, 0.0) : 0.0;
            pts[i + 1][0] = px;
            pts[i + 1][1] = py;
            pts[i + 1][2] = ph;
        }
        RAW_POINTS = pts;

        VELOCITY_IN_S = obj.optDouble("velocity", VELOCITY_IN_S);
        TOLERANCE_IN  = obj.optDouble("tolerance", TOLERANCE_IN);

        // tags: [{index, name, value}, ...]
        JSONArray tagsArr = obj.optJSONArray("tags");
        if (tagsArr != null) {
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
        } else {
            TAGS = new Tag[0];
        }
    }

    private static void applyLegacyArrayPayload(JSONArray arr) throws JSONException {
        // Legacy: entire array is the path (no separate start). Keep as-is.
        double[][] pts = new double[arr.length()][3];
        for (int i = 0; i < arr.length(); i++) {
            JSONArray p = arr.getJSONArray(i);
            pts[i][0] = p.getDouble(0);
            pts[i][1] = p.getDouble(1);
            pts[i][2] = p.getDouble(2);
        }
        RAW_POINTS = pts;
    }

    // ---- Helpers ----
    private static JSONArray toJson(Tag[] tags) throws JSONException {
        JSONArray arr = new JSONArray();
        for (Tag t : tags) {
            JSONObject o = new JSONObject()
                    .put("index", t.index)
                    .put("name", t.name)
                    .put("value", t.value);
            arr.put(o);
        }
        return arr;
    }

    // ---- CORS helpers ----
    private Response preflightResponse() {
        Response r = newFixedLengthResponse(Response.Status.NO_CONTENT, "text/plain", "");
        r.addHeader("Access-Control-Allow-Origin", "*");
        r.addHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
        r.addHeader("Access-Control-Allow-Headers", "Content-Type");
        r.addHeader("Access-Control-Allow-Private-Network", "true");
        r.addHeader("Access-Control-Max-Age", "600");
        return r;
    }
    private Response withCors(Response r) {
        r.addHeader("Access-Control-Allow-Origin", "*");
        r.addHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
        r.addHeader("Access-Control-Allow-Headers", "Content-Type");
        r.addHeader("Access-Control-Allow-Private-Network", "true");
        return r;
    }
}
