import 'package:flutter/material.dart';

class SettingsScreen extends StatefulWidget {
  final double linear;
  final double angular;

  const SettingsScreen({
    super.key,
    required this.linear,
    required this.angular,
  });

  @override
  SettingsScreenState createState() => SettingsScreenState();
}

class SettingsScreenState extends State<SettingsScreen> {
  late TextEditingController linearController;
  late TextEditingController angularController;

  @override
  void initState() {
    super.initState();
    linearController = TextEditingController(text: widget.linear.toString());
    angularController = TextEditingController(text: widget.angular.toString());

    // Add listeners to auto-clamp values
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
                labelText: "Linear Velocity Multiplier (0.0 - 1.0)",
                border: OutlineInputBorder(),
              ),
              keyboardType:
                  const TextInputType.numberWithOptions(decimal: true),
            ),
            const SizedBox(height: 20),
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
            ElevatedButton(
              onPressed: () {
                Navigator.pop(context, {
                  "linear": double.tryParse(linearController.text) ?? widget.linear,
                  "angular": double.tryParse(angularController.text) ?? widget.angular,
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
