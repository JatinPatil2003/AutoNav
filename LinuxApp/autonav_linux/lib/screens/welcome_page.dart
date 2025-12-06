import 'package:flutter/material.dart';
import '../services/api_service.dart';
import './mode_page.dart';
import './setting_page.dart';

class WelcomePage extends StatefulWidget {
  const WelcomePage({super.key});

  @override
  State<WelcomePage> createState() => _WelcomePageState();
}

class _WelcomePageState extends State<WelcomePage> {
  bool robotStarted = false;
  bool isStarting = false; // loading flag for START
  bool isStopping = false; // loading flag for STOP
  final ApiService _apiService = ApiService();
  bool emergencyActive = false;

  @override
  void initState() {
    super.initState();
    _apiService.setLed(5);
  }

  Future<void> _startRobot() async {
    setState(() {
      isStarting = true;
    });

    final success = await _apiService.startRobot();

    if (success) {
      await Future.delayed(const Duration(seconds: 7)); // simulate start delay

      if (!mounted) return;

      final result = await Navigator.push(
        context,
        MaterialPageRoute(builder: (context) => const ModePage()),
      );

      setState(() {
        robotStarted = true;
        isStarting = false;
      });
      // result contains emergencyActive from ModePage
      if (result != null) {
        setState(() {
          emergencyActive = result as bool;
        });
      }
    } else {
      setState(() {
        isStarting = false;
      });
      _showError("Failed to start robot");
    }
  }

  Future<void> _stopRobot() async {
    setState(() {
      isStopping = true;
    });

    final success = await _apiService.stopRobot();

    if (success) {
      await Future.delayed(const Duration(seconds: 1)); // simulate stop delay

      setState(() {
        robotStarted = false;
        isStopping = false;
      });
    } else {
      setState(() {
        isStopping = false;
      });
      _showError("Failed to stop robot");
    }
  }

  void _showError(String message) {
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(content: Text(message)),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.grey[100],
      body: Stack(
        children: [
          Positioned(
            top: 10,
            right: 10,
            child: ElevatedButton(
              style: ElevatedButton.styleFrom(
                backgroundColor: emergencyActive ? const Color(0xFF690A0A) : Colors.red,
                foregroundColor: Colors.white,
                padding: const EdgeInsets.symmetric(horizontal: 32, vertical: 20),
                shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
              ),
              onPressed: () {
                setState(() => emergencyActive = !emergencyActive);
                _apiService.emergencyStop(emergencyActive);
                if (emergencyActive) {
                  _apiService.setLed(2); // Set LED to emergency status
                }
                else {
                  _apiService.setLed(5); // Reset LED to normal status
                }
              },
              child: const Text(
                "EMERGENCY",
                style: TextStyle(fontWeight: FontWeight.bold, fontSize: 22),
              ),
            ),
          ),

          Center(
            child: Container(
              padding: const EdgeInsets.all(24),
              margin: const EdgeInsets.symmetric(horizontal: 32),
              decoration: BoxDecoration(
                color: Colors.white,
                borderRadius: BorderRadius.circular(20),
                boxShadow: [
                  BoxShadow(
                    color: Colors.black.withOpacity(0.1),
                    offset: const Offset(0, 8),
                    blurRadius: 12,
                  )
                ],
              ),
              child: Column(
                mainAxisSize: MainAxisSize.min,
                children: [
                  Text(
                    "Welcome to AutoNav",
                    textAlign: TextAlign.center,
                    style: Theme.of(context).textTheme.headlineMedium?.copyWith(
                          color: Colors.grey[900],
                          fontWeight: FontWeight.bold,
                        ),
                  ),
                  const SizedBox(height: 40),

                  // Show loader if starting or stopping
                  if (isStarting || isStopping)
                    Column(
                      children: [
                        CircularProgressIndicator(
                          color: isStarting ? Colors.green : Colors.redAccent,
                          strokeWidth: 8,
                        ),
                        const SizedBox(height: 16),
                        Text(
                          isStarting
                              ? "Starting Robot..."
                              : "Stopping Robot...",
                          style: const TextStyle(
                            fontSize: 18,
                            color: Colors.grey,
                          ),
                        ),
                      ],
                    )
                  else
                    ElevatedButton(
                      style: ElevatedButton.styleFrom(
                        padding: const EdgeInsets.symmetric(
                            horizontal: 40, vertical: 20),
                        backgroundColor:
                            robotStarted ? Colors.redAccent : Colors.green,
                        foregroundColor: Colors.white,
                        elevation: 6,
                        shadowColor: Colors.black26,
                        shape: RoundedRectangleBorder(
                          borderRadius: BorderRadius.circular(16),
                        ),
                        minimumSize: const Size(180, 60),
                      ),
                      onPressed:
                          robotStarted ? _stopRobot : _startRobot,
                      child: Text(
                        robotStarted ? "STOP" : "START",
                        style: const TextStyle(
                          fontSize: 22,
                          fontWeight: FontWeight.bold,
                        ),
                      ),
                    ),
                ],
              ),
            ),
          ),
          // Settings icon
          Positioned(
            bottom: 16,
            right: 16,
            child: FloatingActionButton(
              heroTag: "settings_btn",
              backgroundColor: Colors.grey[300],
              onPressed: () {
                Navigator.push(
                  context,
                  MaterialPageRoute(
                      builder: (context) => const SettingsPage()),
                );
              },
              child: const Icon(Icons.settings, color: Colors.grey, size: 28),
            ),
          ),
        ],
      ),
    );
  }
}
