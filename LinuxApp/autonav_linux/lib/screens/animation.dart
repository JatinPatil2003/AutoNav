import 'package:flutter/material.dart';
import 'dart:async';
import '../services/api_service.dart';

class AnimationPage extends StatefulWidget {
  final String targetPoint;

  const AnimationPage({super.key, required this.targetPoint});

  @override
  State<AnimationPage> createState() => _AnimationPageState();
}

class _AnimationPageState extends State<AnimationPage>
    with SingleTickerProviderStateMixin {
  late AnimationController _controller;
  late Animation<Offset> _animation;

  final ApiService _apiService = ApiService();

  bool emergencyActive = false;

  @override
  void initState() {
    super.initState();

    emergencyActive = _apiService.emergencyActive;

    if (emergencyActive) {
      _apiService.setLed(2); // Set LED to emergency status
    } else {
      _apiService.setLed(9);
    }

    // Animation from left to right
    _controller = AnimationController(
      vsync: this,
      duration: const Duration(seconds: 10),
    );

    _animation = Tween<Offset>(
      begin: const Offset(-5.0, 0.0),
      end: const Offset(5.0, 0.0),
    ).animate(
      CurvedAnimation(
        parent: _controller,
        curve: Curves.bounceInOut,
      ),
    );

    _controller.repeat(reverse: true);
  }

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      onDoubleTap: () {
        if (emergencyActive) {
          _apiService.setLed(2);
        } else {
          _apiService.setLed(4);
        }
        Navigator.pop(context, _apiService.emergencyActive);
      }, // Double-tap to exit
      child: Scaffold(
        backgroundColor: Colors.black,   // FULL BLACK BACKGROUND
        body: Stack(
          children: [
            // ----------- MAIN CONTENT -----------
            Center(
              child: Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  const Text(
                    'Robot moving to:',
                    style: TextStyle(
                      fontSize: 32,
                      fontWeight: FontWeight.bold,
                      color: Colors.white,   // visible on black
                    ),
                  ),
                  const SizedBox(height: 10),
                  Text(
                    widget.targetPoint,
                    style: const TextStyle(
                      fontSize: 34,
                      fontWeight: FontWeight.bold,
                      color: Colors.lightBlueAccent, // bright on black
                    ),
                  ),
                  const SizedBox(height: 60),
                  SizedBox(
                    height: 100,
                    child: Stack(
                      children: [
                        SlideTransition(
                          position: _animation,
                          child: const Icon(
                            Icons.rocket_launch,
                            size: 70,
                            color: Colors.deepOrangeAccent, // high contrast
                          ),
                        ),
                      ],
                    ),
                  ),
                ],
              ),
            ),

            // ------------- EMERGENCY BUTTON -------------
            Positioned(
              top: 20,
              right: 20,
              child: ElevatedButton(
                style: ElevatedButton.styleFrom(
                  backgroundColor:
                      emergencyActive ? const Color(0xFF690A0A) : Colors.red,
                  foregroundColor: Colors.white,
                  padding:
                      const EdgeInsets.symmetric(horizontal: 32, vertical: 20),
                  shape: RoundedRectangleBorder(
                    borderRadius: BorderRadius.circular(12),
                  ),
                ),
                onPressed: () {
                  setState(() => emergencyActive = !emergencyActive);
                  if (emergencyActive) {
                    _apiService.setLed(2);
                  } else {
                    _apiService.setLed(9);
                  }

                  _apiService.emergencyStop(emergencyActive);
                },
                child: const Text(
                  "EMERGENCY",
                  style: TextStyle(
                    fontWeight: FontWeight.bold,
                    fontSize: 22,
                  ),
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }
}
