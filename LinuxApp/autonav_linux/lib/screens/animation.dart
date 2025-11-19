import 'package:flutter/material.dart';
import 'dart:async';

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

  @override
  void initState() {
    super.initState();

    // Animation from left to right
    _controller =
        AnimationController(vsync: this, duration: const Duration(seconds: 10));
    _animation = Tween<Offset>(
      begin: const Offset(-5.0, 0.0),
      end: const Offset(5.0, 0.0),
    ).animate(CurvedAnimation(parent: _controller, curve: Curves.bounceInOut));

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
        Navigator.pop(context); // go back to previous page
      },
      child: Scaffold(
        // appBar: AppBar(
        //   title: const Text('Navigation Animation'),
        // ),
        body: Stack(
          children: [
            Center(
              child: Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  const Text(
                    'Robot moving to:',
                    style: TextStyle(fontSize: 20, fontWeight: FontWeight.bold),
                  ),
                  const SizedBox(height: 10),
                  Text(
                    widget.targetPoint,
                    style: const TextStyle(
                        fontSize: 24,
                        fontWeight: FontWeight.bold,
                        color: Colors.blue),
                  ),
                  const SizedBox(height: 50),
                  SizedBox(
                    height: 100,
                    child: Stack(
                      children: [
                        SlideTransition(
                          position: _animation,
                          child: const Icon(
                            Icons.rocket,
                            size: 50,
                            color: Colors.red,
                          ),
                        ),
                      ],
                    ),
                  ),
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }
}
