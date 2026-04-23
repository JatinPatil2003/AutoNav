import 'package:flutter/material.dart';

class SettingsScreen extends StatefulWidget {
  final double linear;
  final double angular;
  final bool videoEnabled;

  const SettingsScreen({
    super.key,
    required this.linear,
    required this.angular,
    required this.videoEnabled,
  });

  @override
  SettingsScreenState createState() => SettingsScreenState();
}

class SettingsScreenState extends State<SettingsScreen> {
  late TextEditingController linearController;
  late TextEditingController angularController;
  late bool videoEnabled;

  @override
  void initState() {
    super.initState();

    linearController =
        TextEditingController(text: widget.linear.toString());
    angularController =
        TextEditingController(text: widget.angular.toString());

    videoEnabled = widget.videoEnabled;

    // Clamp linear (0.0 - 1.0)
    linearController.addListener(() {
      final value = double.tryParse(linearController.text);
      if (value != null) {
        final clamped = value.clamp(0.0, 1.0);
        if (clamped != value) {
          linearController.text = clamped.toString();
          linearController.selection = TextSelection.fromPosition(
            TextPosition(offset: linearController.text.length),
          );
        }
      }
    });

    // Clamp angular (0.0 - 2.0)
    angularController.addListener(() {
      final value = double.tryParse(angularController.text);
      if (value != null) {
        final clamped = value.clamp(0.0, 2.0);
        if (clamped != value) {
          angularController.text = clamped.toString();
          angularController.selection = TextSelection.fromPosition(
            TextPosition(offset: angularController.text.length),
          );
        }
      }
    });
  }

  @override
  void dispose() {
    linearController.dispose();
    angularController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text("Settings")),
      body: Padding(
        padding: const EdgeInsets.all(20.0),
        child: Column(
          children: [

            // 🔹 Linear Velocity
            TextField(
              controller: linearController,
              decoration: const InputDecoration(
                labelText: "Linear Velocity Multiplier (0.0 - 1.0)",
                border: OutlineInputBorder(),
              ),
              keyboardType:
                  const TextInputType.numberWithOptions(decimal: true),
            ),

            const SizedBox(height: 20),

            // 🔹 Angular Velocity
            TextField(
              controller: angularController,
              decoration: const InputDecoration(
                labelText: "Angular Velocity Multiplier (0.0 - 2.0)",
                border: OutlineInputBorder(),
              ),
              keyboardType:
                  const TextInputType.numberWithOptions(decimal: true),
            ),

            const SizedBox(height: 30),

            // 🔹 Video Toggle
            SwitchListTile(
              title: const Text("Enable Video Stream"),
              subtitle: const Text("Turn live camera stream ON/OFF"),
              value: videoEnabled,
              onChanged: (value) {
                setState(() {
                  videoEnabled = value;
                });
                print("Video Enabled: $videoEnabled");
              },
            ),

            const SizedBox(height: 30),

            // 🔹 Save Button
            ElevatedButton(
              onPressed: () {
                Navigator.pop(context, {
                  "linear":
                      double.tryParse(linearController.text) ??
                          widget.linear,
                  "angular":
                      double.tryParse(angularController.text) ??
                          widget.angular,
                  "videoEnabled": videoEnabled,
                });
              },
              child: const Text("Save"),
            ),
          ],
        ),
      ),
    );
  }
}
