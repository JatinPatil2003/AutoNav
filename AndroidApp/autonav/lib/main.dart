import 'dart:async';
import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:http/http.dart' as http;
import 'package:web_socket_channel/web_socket_channel.dart';
import 'package:flutter_joystick/flutter_joystick.dart';
import 'config.dart';

void main() {
  runApp(const MyApp());
}

late WebSocketChannel channel;

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      debugShowCheckedModeBanner: false,
      home: RobotController(),
    );
  }
}

class RobotController extends StatefulWidget {
  @override
  State<RobotController> createState() => _RobotControllerState();
}

class _RobotControllerState extends State<RobotController> {
  String status = "Disconnected";
  Timer? statusTimer;

  /// Default multipliers
  double linearMultiplier = 0.5;
  double angularMultiplier = 1.0;

  @override
  void initState() {
    super.initState();
    connectWebSocket();
    checkRobotStatus(); // first check
    statusTimer = Timer.periodic(const Duration(seconds: 1), (timer) {
      checkRobotStatus();
    });
  }

  @override
  void dispose() {
    statusTimer?.cancel();
    channel.sink.close();
    super.dispose();
  }

  /// API call (generic)
  Future<void> callApi(String url) async {
    try {
      final response = await http.get(Uri.parse(url));
      if (response.statusCode == 200) {
        print("✅ Success: ${response.body}");
      } else {
        print("❌ Error: ${response.statusCode}");
      }
    } catch (e) {
      print("⚠️ Failed: $e");
    }
  }

  void startRobot() => callApi("$API_FULL_URL/robot/start");
  void stopRobot() => callApi("$API_FULL_URL/robot/stop");

  Future<void> checkRobotStatus() async {
    try {
      final response = await http.get(Uri.parse("$API_FULL_URL/robot/status"));
      if (response.statusCode == 200) {
        setState(() {
          status = "Connected";
        });
      } else {
        setState(() {
          status = "Disconnected";
        });
      }
    } catch (e) {
      setState(() {
        status = "Disconnected";
      });
    }
  }

  void connectWebSocket() {
    channel = WebSocketChannel.connect(Uri.parse("$WS_FULL_URL/joystick"));
    channel.stream.listen((message) {
      print("📩 Received: $message");
    });
  }

  void sendCommand(Map<String, dynamic> command) {
    final jsonCommand = jsonEncode(command);
    channel.sink.add(jsonCommand);
    print("📤 Sent: $jsonCommand");
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
                  linearMultiplier = result["linear"]!;
                  angularMultiplier = result["angular"]!;
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
                ElevatedButton(
                  onPressed: startRobot,
                  style: ElevatedButton.styleFrom(
                    padding: const EdgeInsets.symmetric(horizontal: 40, vertical: 20), // bigger button
                    textStyle: const TextStyle(fontSize: 18, fontWeight: FontWeight.bold), // larger text
                  ),
                  child: const Text("Start"),
                ),
                ElevatedButton(
                  onPressed: stopRobot,
                  style: ElevatedButton.styleFrom(
                    padding: const EdgeInsets.symmetric(horizontal: 40, vertical: 20),
                    textStyle: const TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                  ),
                  child: const Text("Stop"),
                ),
              ],
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

/// Settings Screen
class SettingsScreen extends StatefulWidget {
  final double linear;
  final double angular;

  const SettingsScreen({super.key, required this.linear, required this.angular});

  @override
  State<SettingsScreen> createState() => _SettingsScreenState();
}

class _SettingsScreenState extends State<SettingsScreen> {
  late TextEditingController linearController;
  late TextEditingController angularController;

  @override
  void initState() {
    super.initState();
    linearController =
        TextEditingController(text: widget.linear.toStringAsFixed(2));
    angularController =
        TextEditingController(text: widget.angular.toStringAsFixed(2));
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text("Settings")),
      body: Padding(
        padding: const EdgeInsets.all(20.0),
        child: Column(
          children: [
            TextField(
              controller: linearController,
              decoration: const InputDecoration(
                labelText: "Linear Velocity Multiplier",
                border: OutlineInputBorder(),
              ),
              keyboardType:
                  const TextInputType.numberWithOptions(decimal: true),
            ),
            const SizedBox(height: 20),
            TextField(
              controller: angularController,
              decoration: const InputDecoration(
                labelText: "Angular Velocity Multiplier",
                border: OutlineInputBorder(),
              ),
              keyboardType:
                  const TextInputType.numberWithOptions(decimal: true),
            ),
            const SizedBox(height: 30),
            ElevatedButton(
              onPressed: () {
                final linear =
                    double.tryParse(linearController.text) ?? widget.linear;
                final angular =
                    double.tryParse(angularController.text) ?? widget.angular;

                Navigator.pop(context, {"linear": linear, "angular": angular});
              },
              child: const Text("Save"),
            ),
          ],
        ),
      ),
    );
  }
}
