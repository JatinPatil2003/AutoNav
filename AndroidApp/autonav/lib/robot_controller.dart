import 'dart:async';
import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:http/http.dart' as http;
import 'package:web_socket_channel/web_socket_channel.dart';
import 'package:flutter_joystick/flutter_joystick.dart';

import 'config.dart';
import 'setting_screen.dart';
import 'robot_video_webrtc.dart';
import 'package:shared_preferences/shared_preferences.dart';

WebSocketChannel? channel;

class RobotController extends StatefulWidget {
  const RobotController({super.key});

  @override
  State<RobotController> createState() => _RobotControllerState();
}

class _RobotControllerState extends State<RobotController> {
  String status = "Disconnected";
  String robotStatusText = "";
  Timer? statusTimer;

  /// Default multipliers
  double linearMultiplier = 0.3;
  double angularMultiplier = 0.7;

  bool emergencyActive = false;
  bool videoEnabled = true;

  final GlobalKey<RobotVideoWidgetState> videoKeyRef =
    GlobalKey<RobotVideoWidgetState>();


  @override
  void initState() {
    super.initState();
    loadSettings();
    checkRobotStatus(); // first check
    statusTimer = Timer.periodic(const Duration(seconds: 3), (timer) {
      checkRobotStatus();
    });
  }

  @override
  void dispose() {
    statusTimer?.cancel();
    channel?.sink.close();
    super.dispose();
  }

  /// API call (generic)
  Future<void> callApi(String url) async {
    try {
      final response = await http.get(Uri.parse(url));
      if (response.statusCode == 200) {
        // print("✅ Success: ${response.body}");
      } else {
        // print("❌ Error: ${response.statusCode}");
      }
    } catch (e) {
      // print("⚠️ Failed: $e");
    }
  }

  Future<void> callPostApi(String url, Map<String, dynamic> body) async {
    try {
      final response = await http.post(
        Uri.parse(url),
        headers: {"Content-Type": "application/json"},
        body: jsonEncode(body),
      );

      if (response.statusCode == 200) {
        // Success
      } else {
        // Handle failure
      }
    } catch (e) {
      // Handle network errors
    }
  }

  void startRobot() => callApi("$apiFullUrl/robot/start");
  void stopRobot() => callApi("$apiFullUrl/robot/stop");
  void emergencySet(bool value) {
    callPostApi("$apiFullUrl/emergency", {"status": value});
  }

  Future<void> checkRobotStatus() async {
    try {
      final response = await http.get(Uri.parse("$apiFullUrl/robot/status"));
      if (response.statusCode == 200) {
        final data = jsonDecode(response.body)["status"];
        if (status != "Connected") {
          setState(() {
            status = "Connected";
            robotStatusText = mapRobotStatus(data); // or extract specific fields if available
          });
          connectWebSocket(); // connect only when status changes to Connected
          // videoKeyRef.currentState?.startConnection();
        } 
        else {
          setState(() {
            robotStatusText = mapRobotStatus(data);
          });
        }
      } else {
        setState(() {
          status = "Disconnected";
          robotStatusText = "N/A";
        });
        channel?.sink.close();
      }
    } catch (e) {
      setState(() {
        status = "Disconnected";
        robotStatusText = "N/A";
      });
      channel?.sink.close();
    }
  }

  void connectWebSocket() {
    if (status != "Connected") {
      print("⚠️ Skipping WebSocket connection, status is not Connected.");
      return;
    }

    try {
      channel = WebSocketChannel.connect(Uri.parse("$wsFullUrl/joystick"));

      channel?.stream.listen(
        (message) {
          // print("📩 Received: $message");
        },
        onDone: () {
          print("❌ WebSocket closed. Retrying in 5s...");
          status = "Disconnected";
          // retryWebSocket();
        },
        onError: (error) {
          print("⚠️ WebSocket error: $error. Retrying in 5s...");
          // retryWebSocket();
        },
        cancelOnError: true,
      );
    } catch (e) {
      print("⚠️ Failed to connect WebSocket: $e. Retrying in 5s...");
      // retryWebSocket();
    }
  }

  void retryWebSocket() {
    Future.delayed(const Duration(seconds: 5), () {
      connectWebSocket();
    });
  }

  void sendCommand(Map<String, dynamic> command) {
    final jsonCommand = jsonEncode(command);
    channel?.sink.add(jsonCommand);
    // print("📤 Sent: $jsonCommand");
  }

  Future<void> saveSettings() async {
    final prefs = await SharedPreferences.getInstance();

    await prefs.setDouble("linear", linearMultiplier);
    await prefs.setDouble("angular", angularMultiplier);
    await prefs.setBool("videoEnabled", videoEnabled);
  }

  Future<void> loadSettings() async {
    final prefs = await SharedPreferences.getInstance();

    setState(() {
      linearMultiplier = prefs.getDouble("linear") ?? 0.3;
      angularMultiplier = prefs.getDouble("angular") ?? 0.7;
      videoEnabled = prefs.getBool("videoEnabled") ?? true;
    });

    print("Loaded settings - "
          "Linear: $linearMultiplier, "
          "Angular: $angularMultiplier, "
          "Video Enabled: $videoEnabled");
  }

  String mapRobotStatus(String apiStatus) {
    final s = apiStatus.toLowerCase();

    if (s == "started" || s == "running") return "RUNNING";
    if (s == "stopped" || s == "idle") return "IDLE";

    return apiStatus.toUpperCase(); // fallback
  }

  Color robotStatusColor(String status) {
    if (status == "RUNNING") return Colors.green;
    if (status == "IDLE") return Colors.orange;
    return Colors.white;
  }


  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: const Color.fromARGB(220, 128, 134, 219), // light grey background
      appBar: AppBar(
        toolbarHeight: 40,
        title: Text(
          "AutoNav",
          style: TextStyle(
            color: Colors.black,
            fontSize: 30,
            fontWeight: FontWeight.w900, // heavier than bold
            letterSpacing: 1.2,
            shadows: [
              Shadow(
                offset: Offset(2, 2),
                blurRadius: 4,
                color: Colors.black.withOpacity(0.35),
              ),
              Shadow(
                offset: Offset(-1, -1),
                blurRadius: 2,
                color: Colors.white.withOpacity(0.2),
              ),
            ],
          ),
        ),
        backgroundColor: status == "Connected" ? Colors.green : Colors.red,
        actions: [
          Center(
            child: Padding(
              padding: const EdgeInsets.symmetric(horizontal: 16.0),
              child: Text(status, style: const TextStyle(color: Colors.white)),
            ),
          ),
          IconButton(
            icon: const Icon(Icons.settings),
            onPressed: () async {
              final result = await Navigator.push(
                context,
                MaterialPageRoute(
                  builder: (_) => SettingsScreen(
                    linear: linearMultiplier,
                    angular: angularMultiplier,
                    videoEnabled: videoEnabled,
                  ),
                ),
              );
              if (result != null && result is Map<String, dynamic>) {
                setState(() {
                  linearMultiplier = (result["linear"] as num?)?.toDouble().clamp(0.0, 1.0) ?? linearMultiplier;
                  angularMultiplier =(result["angular"] as num?)?.toDouble().clamp(0.0, 2.0) ?? angularMultiplier;
                  videoEnabled = result["videoEnabled"] as bool? ?? videoEnabled;
                });
                await saveSettings();
              }
            },
          ),
        ],
      ),
      body: Column(
        children: [
          if (videoEnabled)
            Padding(
              padding: EdgeInsets.all(8.0),
              child: RobotVideoWidget(key: videoKeyRef),
            ),
            
          // Buttons row
          Padding(
            padding: const EdgeInsets.fromLTRB(0, 16, 0, 5),
            child: Row(
              mainAxisAlignment: MainAxisAlignment.spaceEvenly,
              children: [
                // Start Button
                ElevatedButton(
                  onPressed: () {
                    startRobot();
                  },
                  style: ButtonStyle(
                    padding: WidgetStateProperty.all(
                      const EdgeInsets.symmetric(horizontal: 40, vertical: 20),
                    ),
                    textStyle: WidgetStateProperty.all(
                      const TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                    ),
                    backgroundColor: WidgetStateProperty.resolveWith<Color>(
                      (states) {
                        if (states.contains(WidgetState.hovered)) {
                          return robotStatusText == "IDLE"
                              ? Colors.green.shade700
                              : Colors.grey.shade700;
                        }
                        return robotStatusText == "IDLE"
                            ? Colors.green
                            : Colors.grey;
                      },
                    ),
                  ),
                  child: const Text("Start", style: TextStyle(color: Colors.white)),
                ),

                // Stop Button
                ElevatedButton(
                  onPressed: () {
                    stopRobot();
                  },
                  style: ButtonStyle(
                    padding: WidgetStateProperty.all(
                      const EdgeInsets.symmetric(horizontal: 40, vertical: 20),
                    ),
                    textStyle: WidgetStateProperty.all(
                      const TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                    ),
                    backgroundColor: WidgetStateProperty.resolveWith<Color>(
                      (states) {
                        if (states.contains(WidgetState.hovered)) {
                          return robotStatusText == "RUNNING"
                              ? Colors.red.shade700
                              : Colors.grey.shade700;
                        }
                        return robotStatusText == "RUNNING"
                            ? Colors.red
                            : Colors.grey;
                      },
                    ),
                  ),
                  child: const Text("Stop", style: TextStyle(color: Colors.white)),
                ),
              ],
            ),
          ),

          if (status == "Connected")
            Padding(
              padding: const EdgeInsets.fromLTRB(0, 10, 0, 20),
              child: RichText(
                text: TextSpan(
                  style: const TextStyle(
                    fontSize: 16,
                    color: Colors.white,
                  ),
                  children: [
                    const TextSpan(
                      text: "Robot Status: ",
                      style: TextStyle(fontWeight: FontWeight.bold),
                    ),
                    TextSpan(
                      text: robotStatusText,
                      style: TextStyle(
                        fontWeight: FontWeight.bold,
                        fontSize: 22,
                        color: robotStatusColor(robotStatusText),
                      ),
                    ),
                  ],
                ),
              ),
            ),
            
          if (status == "Connected")
            Padding(
              padding: const EdgeInsets.only(bottom: 20.0),
              child: ElevatedButton(
                style: ElevatedButton.styleFrom(
                  backgroundColor: emergencyActive
                      ? const Color(0xFF690A0A) // dark red when latched
                      : Colors.red,             // normal red when off
                  foregroundColor: Colors.white,
                  padding: const EdgeInsets.symmetric(horizontal: 48, vertical: 20),
                  shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
                ),
                onPressed: () {
                  setState(() => emergencyActive = !emergencyActive);

                  // Send true when latched, false when released
                  emergencySet(emergencyActive);
                },
                child: Text(
                  emergencyActive ? "EMERGENCY" : "EMERGENCY",
                  style: const TextStyle(
                    fontWeight: FontWeight.bold,
                    fontSize: 22,
                  ),
                ),
              ),
            ),


          const Spacer(),

          // Joystick at bottom center
          Padding(
            padding: const EdgeInsets.only(bottom: 80.0),
            child: SizedBox(
              width: 200,
              height: 200,
              child: Joystick(
                mode: JoystickMode.all,
                listener: (details) {
                  final command = {
                    "type": "joystick",
                    "data": {
                      "linear": double.parse(
                          (details.y * -linearMultiplier).toStringAsFixed(2)),
                      "angular": double.parse(
                          (details.x * -angularMultiplier).toStringAsFixed(2)),
                    }
                  };
                  sendCommand(command);
                },
              ),
            ),
          ),
        ],
      ),
    );
  }
}