package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.InetAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.attribute.FileTime;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;

/**
 * Minimal disabled-only HTTP server to expose AdvantageKit logs.
 *
 * <p>Endpoints: GET / -> 200 text/plain GET /status.json -> 200 application/json { enabled, time }
 * GET /logs.json -> 200 application/json [ filenames... ] GET /<filename> -> 200
 * application/octet-stream (raw bytes)
 *
 * <p>Server only runs while Disabled; Robot must start/stop per mode transitions.
 */
public final class LogServer implements Runnable {
  private final int port;
  private final Path logDir;
  private volatile boolean running = false;
  private volatile ServerSocket serverSocket = null;
  private final Set<String> allowedExt;

  // Safe filename pattern: letters, digits, dot, underscore, hyphen only.
  private static final String SAFE_NAME_REGEX = "^[A-Za-z0-9._-]+$";

  public LogServer(int port, Path logDir) {
    this.port = port;
    this.logDir = logDir;
    this.allowedExt = new HashSet<>();
    this.allowedExt.add(".wpilog");
    this.allowedExt.add(".hoot");
  }

  @Override
  public void run() {
    running = true;
    try (ServerSocket ss = new ServerSocket(port)) {
      this.serverSocket = ss;
      ss.setReuseAddress(true);
      System.out.printf("[LogServer] Listening on %d (dir=%s)%n", port, logDir.toString());
      while (running) {
        try {
          Socket client = ss.accept();
          client.setSoTimeout(3000);
          // Tiny per-connection thread. Keep handler minimal.
          Thread t = new Thread(() -> handle(client), "LogServer-conn");
          t.setDaemon(true);
          t.start();
        } catch (IOException acceptEx) {
          if (running) {
            // Unexpected; keep trying.
            System.out.printf("[LogServer] accept() error: %s%n", acceptEx.getMessage());
          }
        }
      }
    } catch (IOException bindEx) {
      System.out.printf("[LogServer] Failed to bind port %d: %s%n", port, bindEx.getMessage());
    } finally {
      serverSocket = null;
      running = false;
      System.out.println("[LogServer] Stopped");
    }
  }

  public void stop() {
    running = false;
    // Wake accept() by creating a local connection if still bound.
    ServerSocket ss = this.serverSocket;
    if (ss != null && !ss.isClosed()) {
      try (Socket s = new Socket(InetAddress.getLoopbackAddress(), port)) {
        // Immediately close after connecting.
      } catch (IOException ignored) {
      }
      try {
        ss.close();
      } catch (IOException ignored) {
      }
    }
  }

  // ---- Core request handling ----
  private void handle(Socket socket) {
    try (socket) {
      InputStream in = socket.getInputStream();
      OutputStream out = socket.getOutputStream();

      // Parse the request line and headers (very small, defensive limits).
      BufferedReader reader =
          new BufferedReader(new InputStreamReader(in, StandardCharsets.US_ASCII));

      String requestLine = reader.readLine();
      if (requestLine == null || requestLine.length() > 1024) {
        writeResponse(out, 400, "Bad Request", "text/plain", bytesOf("Bad Request"), false);
        return;
      }

      String[] parts = requestLine.split(" ");
      if (parts.length < 3) {
        writeResponse(out, 400, "Bad Request", "text/plain", bytesOf("Bad Request"), false);
        return;
      }
      String method = parts[0];
      String rawPath = parts[1];

      // Read headers until blank line.
      Map<String, String> headers = new HashMap<>();
      for (int i = 0; i < 100; i++) { // cap to 100 headers
        String line = reader.readLine();
        if (line == null) break;
        if (line.isEmpty()) break;
        int idx = line.indexOf(':');
        if (idx > 0) {
          String key = line.substring(0, idx).trim().toLowerCase(Locale.ROOT);
          String val = line.substring(idx + 1).trim();
          headers.put(key, val);
        }
      }

      boolean isHead = method.equals("HEAD");
      boolean isGet = method.equals("GET");
      if (!(isGet || isHead)) {
        byte[] body = bytesOf("Method Not Allowed\n");
        writeResponse(out, 405, "Method Not Allowed", "text/plain", body, isHead);
        return;
      }

      // Optional double-safety: if robot is enabled, deny.
      if (DriverStation.isEnabled()) {
        byte[] body = bytesOf("Server disabled while robot is enabled\n");
        writeResponse(out, 503, "Service Unavailable", "text/plain", body, isHead);
        return;
      }

      // Normalize path (strip query if any); we only handle top-level endpoints.
      String path = rawPath;
      int q = path.indexOf('?');
      if (q >= 0) path = path.substring(0, q);

      switch (path) {
        case "/":
          {
            byte[] body = bytesOf("Robot log server is online.\n");
            writeResponse(out, 200, "OK", "text/plain", body, isHead);
            return;
          }
        case "/status.json":
          {
            boolean enabled = DriverStation.isEnabled();
            String json =
                "{\"enabled\":"
                    + (enabled ? "true" : "false")
                    + ",\"time\":\""
                    + Instant.now().toString()
                    + "\"}";
            byte[] body = json.getBytes(StandardCharsets.UTF_8);
            writeResponse(out, 200, "OK", "application/json", body, isHead);
            return;
          }
        case "/logs.json":
          {
            byte[] body = buildLogsIndexJson();
            writeResponse(out, 200, "OK", "application/json", body, isHead);
            return;
          }
        default:
          // Expecting "/<filename>"
          if (path.startsWith("/")) path = path.substring(1);
          if (!path.matches(SAFE_NAME_REGEX)) {
            writeResponse(out, 404, "Not Found", "text/plain", bytesOf("Not Found\n"), isHead);
            return;
          }
          String fileName = path;
          String lower = fileName.toLowerCase(Locale.ROOT);
          boolean allowed = false;
          for (String ext : allowedExt) {
            if (lower.endsWith(ext)) {
              allowed = true;
              break;
            }
          }
          if (!allowed) {
            writeResponse(out, 404, "Not Found", "text/plain", bytesOf("Not Found\n"), isHead);
            return;
          }

          Path target = logDir.resolve(fileName).normalize();
          if (!target.startsWith(logDir)) {
            writeResponse(out, 404, "Not Found", "text/plain", bytesOf("Not Found\n"), isHead);
            return;
          }
          if (!Files.exists(target) || !Files.isRegularFile(target)) {
            writeResponse(out, 404, "Not Found", "text/plain", bytesOf("Not Found\n"), isHead);
            return;
          }
          long len = 0L;
          try {
            len = Files.size(target);
          } catch (IOException ignored) {
          }

          // Headers first
          StringBuilder sb = new StringBuilder();
          sb.append("HTTP/1.1 200 OK\r\n");
          sb.append("Content-Type: application/octet-stream\r\n");
          sb.append("Content-Length: ").append(Long.toString(len)).append("\r\n");
          sb.append("Connection: close\r\n\r\n");
          out.write(sb.toString().getBytes(StandardCharsets.US_ASCII));
          out.flush();

          if (!isHead) {
            try (InputStream fis = Files.newInputStream(target)) {
              fis.transferTo(out);
            }
          }
          out.flush();
      }
    } catch (IOException ignored) {
      // Ignore per-connection errors to keep server robust.
    }
  }

  private byte[] buildLogsIndexJson() {
    // List top-level only, filter by allowed extensions, sort by mtime desc.
    List<Path> files = new ArrayList<>();
    try {
      if (Files.isDirectory(logDir)) {
        try (var stream = Files.list(logDir)) {
          stream.forEach(
              p -> {
                String name = p.getFileName().toString();
                String lower = name.toLowerCase(Locale.ROOT);
                boolean allowed = false;
                for (String ext : allowedExt) {
                  if (lower.endsWith(ext)) {
                    allowed = true;
                    break;
                  }
                }
                if (allowed && Files.isRegularFile(p)) {
                  files.add(p);
                }
              });
        }
      }
    } catch (IOException ignored) {
    }

    files.sort(
        new Comparator<Path>() {
          @Override
          public int compare(Path a, Path b) {
            try {
              FileTime ta = Files.getLastModifiedTime(a);
              FileTime tb = Files.getLastModifiedTime(b);
              return -Long.compare(ta.toMillis(), tb.toMillis()); // newest first
            } catch (IOException e) {
              return 0;
            }
          }
        });

    StringBuilder json = new StringBuilder();
    json.append('[');
    for (int i = 0; i < files.size(); i++) {
      if (i > 0) json.append(',');
      String name = files.get(i).getFileName().toString();
      json.append('"').append(escapeJson(name)).append('"');
    }
    json.append(']');
    return json.toString().getBytes(StandardCharsets.UTF_8);
  }

  private static String escapeJson(String s) {
    // Minimal escaping for filenames (quotes and backslashes).
    StringBuilder sb = new StringBuilder(s.length() + 8);
    for (int i = 0; i < s.length(); i++) {
      char c = s.charAt(i);
      if (c == '"' || c == '\\') {
        sb.append('\\');
      }
      sb.append(c);
    }
    return sb.toString();
  }

  private static byte[] bytesOf(String s) {
    return s.getBytes(StandardCharsets.UTF_8);
  }

  private static void writeResponse(
      OutputStream out, int code, String reason, String contentType, byte[] body, boolean head)
      throws IOException {
    StringBuilder sb = new StringBuilder();
    sb.append("HTTP/1.1 ").append(code).append(' ').append(reason).append("\r\n");
    sb.append("Content-Type: ").append(contentType).append("\r\n");
    sb.append("Content-Length: ").append(Integer.toString(body.length)).append("\r\n");
    sb.append("Connection: close\r\n\r\n");
    out.write(sb.toString().getBytes(StandardCharsets.US_ASCII));
    if (!head && body.length > 0) {
      out.write(body);
    }
    out.flush();
  }
}
