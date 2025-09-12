import 'dart:async';
import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:http/http.dart' as http;
import 'package:web_socket_channel/web_socket_channel.dart';
import 'package:flutter_joystick/flutter_joystick.dart';

import 'config.dart';
import 'setting_screen.dart';

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
  double linearMultiplier = 0.5;
  double angularMultiplier = 1.5;

  @override
  void initState() {
    super.initState();
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

  void startRobot() => callApi("$apiFullUrl/robot/start");
  void stopRobot() => callApi("$apiFullUrl/robot/stop");

  Future<void> checkRobotStatus() async {
    try {
      final response = await http.get(Uri.parse("$apiFullUrl/robot/status"));
      if (response.statusCode == 200) {
        final data = jsonDecode(response.body)["status"];
        if (status != "Connected") {
          setState(() {
            status = "Connected";
            robotStatusText = data[0].toUpperCase() + data.substring(1); // or extract specific fields if available
          });
          connectWebSocket(); // connect only when status changes to Connected
        } 
        else {
          setState(() {
            robotStatusText = data[0].toUpperCase() + data.substring(1);
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
      // print("⚠️ Skipping WebSocket connection, status is not Connected.");
      return;
    }

    try {
      channel = WebSocketChannel.connect(Uri.parse("$wsFullUrl/joystick"));
      // print("🔌 WebSocket connecting...");

      channel?.stream.listen(
        (message) {
          // print("📩 Received: $message");
        },
        onDone: () {
          // print("❌ WebSocket closed. Retrying in 5s...");
          retryWebSocket();
        },
        onError: (error) {
          // print("⚠️ WebSocket error: $error. Retrying in 5s...");
          retryWebSocket();
        },
        cancelOnError: true,
      );
    } catch (e) {
      // print("⚠️ Failed to connect WebSocket: $e. Retrying in 5s...");
      retryWebSocket();
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

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: const Color.fromARGB(220, 128, 134, 219), // light grey background
      appBar: AppBar(
        title: const Text("AutoNav"),
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
                  ),
                ),
              );
              if (result != null && result is Map<String, double>) {
                setState(() {
                  linearMultiplier = result["linear"]!.clamp(0.0, 1.0);
                  angularMultiplier = result["angular"]!.clamp(0.0, 2.0);
                });
              }
            },
          ),
        ],
      ),
      body: Column(
        children: [
          // Buttons row
          Padding(
            padding: const EdgeInsets.all(16.0),
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
                          return robotStatusText == "Stopped"
                              ? Colors.green.shade700
                              : Colors.grey.shade700;
                        }
                        return robotStatusText == "Stopped"
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
                          return robotStatusText == "Started"
                              ? Colors.red.shade700
                              : Colors.grey.shade700;
                        }
                        return robotStatusText == "Started"
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
              padding: const EdgeInsets.all(25.0),
              child: RichText(
                text: TextSpan(
                  style: const TextStyle(
                    fontSize: 16,
                    color: Color.fromARGB(255, 255, 255, 255), // ensure text is visible
                  ),
                  children: [
                    const TextSpan(
                      text: "Robot Status: ",
                      style: TextStyle(fontWeight: FontWeight.bold), // ✅ Only this bold
                    ),
                    TextSpan(
                      text: robotStatusText, // normal text
                    ),
                  ],
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