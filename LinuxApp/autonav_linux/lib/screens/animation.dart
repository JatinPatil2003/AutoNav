import 'package:flutter/material.dart';
import 'dart:async';
import 'dart:convert';
import 'package:web_socket_channel/web_socket_channel.dart';

import '../services/api_service.dart';
import '../constants/api_constants.dart';

/// Navigation states from backend
enum NavStatus {
  idle,
  active,
  succeeded,
  canceled,
  failed,
}


class AnimationPage extends StatefulWidget {
  final String targetPoint;
  final String selectedMap;

  const AnimationPage({super.key, required this.selectedMap, required this.targetPoint});

  @override
  State<AnimationPage> createState() => _AnimationPageState();
}

class _AnimationPageState extends State<AnimationPage>
    with SingleTickerProviderStateMixin {
  // Animation
  late AnimationController _controller;
  late Animation<Offset> _animation;
  bool _animationRunning = false;

  // WebSocket
  late WebSocketChannel channel;

  // Services
  final ApiService _apiService = ApiService();
  final String wsbaseUrl = ApiConstants.wsFullUrl;

  late String _targetPoint;
  late String _selectedMap;

  // State
  bool emergencyActive = false;
  NavStatus _navStatus = NavStatus.active;
  NavStatus? _lastHandledStatus;
  bool navstatus = false;

  // Countdown
  Timer? _countdownTimer;
  int _countdownSeconds = 10;

  // UI text
  String _titleText = 'Robot moving to:';
  String _subtitleText = '';

  @override
  void initState() {
    super.initState();

    _targetPoint = widget.targetPoint;
    _selectedMap = widget.selectedMap;

    emergencyActive = _apiService.emergencyActive;
    _apiService.setLed(emergencyActive ? 2 : 9);

    // WebSocket connection
    channel = WebSocketChannel.connect(
      Uri.parse('$wsbaseUrl/autonav'),
    );

    channel.stream.listen(
      (message) {
        final data = jsonDecode(message);
        if (!mounted) return;

        if (data['type'] == 'nav_status') {
          final String status = data['data']['status'];
          _handleNavStatus(status);
        }
      },
      onError: (error) => debugPrint('WS error: $error'),
      onDone: () => debugPrint('WS connection closed'),
    );

    // Animation setup
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
    _animationRunning = true;
  }

  Future<void> startNavigation(String map, String point) async {
    _targetPoint = point;
    print("API CALL: Navigation ST/CH -> $point");

    await _apiService.navigateTo(map, point);

    if (emergencyActive) {
      _apiService.setLed(2); // Set LED to emergency status
    } else {
      _apiService.setLed(9);
    }

    final result = await Navigator.push(
        context,
        MaterialPageRoute(builder: (context) => AnimationPage(selectedMap: map, targetPoint: point)),
      );

    print('STCH navigation pop');

    if (result != null && result is Map<String, bool>) {
      setState(() {
        emergencyActive = result['emergencyActive'] ?? false;
        navstatus = result['navstatus'] ?? false;
      });
    }
  }

  /// Handle nav status from WebSocket (DEDUPLICATED)
  void _handleNavStatus(String status) async{
    // print('Handling nav status: $status');

    if (!mounted) return;

    if (_lastHandledStatus?.name.toUpperCase() == status) return;

    _lastHandledStatus =
        NavStatus.values.firstWhere((e) => e.name.toUpperCase() == status);

    print('Point is $_targetPoint');

    switch (status) {
      case 'ACTIVE':
        _countdownTimer?.cancel();
        if (!_animationRunning) {
          _controller.repeat(reverse: true);
          _animationRunning = true;
        }
        setState(() {
          _navStatus = NavStatus.active;
          _titleText = 'Robot moving to:';
          _subtitleText = '';
        });
        break;

      case 'FAILED':
        if (_targetPoint == 'charger' || _targetPoint == 'standby') {
          print('ST CH Failed');
          channel.sink.close();
          await _apiService.setLed(emergencyActive ? 2 : 2);
          _startCountdown(
            navStatus: NavStatus.failed,
            title: 'Robot failed to move to:',
          );
          await Future.delayed(const Duration(seconds: 5));
          // if (!context.mounted) return;
          Navigator.pop(context, {
            'emergencyActive': emergencyActive,
            'navstatus': false,
          });
          break;
        }
        channel.sink.close();
        _startCountdown(
          navStatus: NavStatus.failed,
          title: 'Robot failed to move to:',
          subtitle: 'Returning to charging in',
          nextAction: _returnToCharging,
        ); 
        await _apiService.setLed(emergencyActive ? 2 : 2);
        break;

      case 'SUCCEEDED':
        if (_targetPoint == 'charger' || _targetPoint == 'standby') {
          print('ST CH Succeeded');
          channel.sink.close();
          await _apiService.setLed(emergencyActive ? 2 : 10);
          _startCountdown(
            navStatus: NavStatus.succeeded,
            title: 'Robot reached point:',
          );
          await Future.delayed(const Duration(seconds: 5));
          // if (!context.mounted) return;
          Navigator.pop(context, {
            'emergencyActive': emergencyActive,
            'navstatus': true,
          });
          break;
        }
        print('Nav Succeeded to normal point');
        channel.sink.close();
        _startCountdown(
          navStatus: NavStatus.succeeded,
          title: 'Robot reached point:',
          subtitle: 'Returning to standby in',
          nextAction: _returnToStandby,
        );
        await _apiService.setLed(emergencyActive ? 2 : 10);
        break;

      default:
        channel.sink.close();
        print('Unknown nav status: $status');
        break;
    }
  }

  /// Countdown logic
  void _startCountdown({
    required NavStatus navStatus,
    required String title,
    String? subtitle,
    VoidCallback? nextAction,
  }) {
    _countdownTimer?.cancel();
    _controller.stop();
    _animationRunning = false;

    setState(() {
      _navStatus = navStatus;
      _titleText = title;
      _subtitleText = subtitle ?? '';
      // _countdownSeconds = 60;
    });

    if (nextAction == null) return;

    _countdownTimer =
        Timer.periodic(const Duration(seconds: 1), (timer) {
      if (!mounted) {
        timer.cancel();
        return;
      }

      setState(() => _countdownSeconds--);

      if (_countdownSeconds <= 0) {
        timer.cancel();
        nextAction();
      }
    });
  }

  void _returnToCharging() async {
    await startNavigation(_selectedMap, 'charger');
    print('ch pop');
    Navigator.pop(context, {
      'emergencyActive': emergencyActive,
      'navstatus': navstatus,
    });
  }

  void _returnToStandby() async{
    print('Returning to Standby function call');
    await startNavigation(_selectedMap, 'standby');
    print('st pop');
    Navigator.pop(context, {
      'emergencyActive': emergencyActive,
      'navstatus': navstatus,
    });
  }

  @override
  void dispose() {
    _countdownTimer?.cancel();
    channel.sink.close();
    _controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      onDoubleTap: () async {
        await _apiService.setLed(emergencyActive ? 2 : 4);
        await _apiService.cancelNavigation();
        channel.sink.close();
        if (!context.mounted) return;
        Navigator.pop(context, {
          'emergencyActive': emergencyActive,
          'navstatus': false,
        });
      },
      child: Scaffold(
        backgroundColor: Colors.black,
        body: Stack(
          children: [
            // ================= CENTER CONTENT =================
            Center(
              child: Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  Text(
                    _titleText,
                    style: const TextStyle(
                      fontSize: 32,
                      fontWeight: FontWeight.bold,
                      color: Colors.white,
                    ),
                  ),
                  const SizedBox(height: 12),
                  Text(
                    _targetPoint,
                    textAlign: TextAlign.center,
                    style: const TextStyle(
                      fontSize: 28,
                      fontWeight: FontWeight.bold,
                      color: Colors.lightBlueAccent,
                    ),
                  ),
                  const SizedBox(height: 60),

                  if (_navStatus == NavStatus.active)
                    SizedBox(
                      height: 100,
                      child: SlideTransition(
                        position: _animation,
                        child: const Icon(
                          Icons.rocket_launch,
                          size: 70,
                          color: Colors.deepOrangeAccent,
                        ),
                      ),
                    ),
                ],
              ),
            ),

            // ================= BOTTOM SUBTITLE =================
            if (_navStatus != NavStatus.active && _subtitleText.isNotEmpty)
              Positioned(
                bottom: 40,
                left: 0,
                right: 0,
                child: Text(
                  '$_subtitleText $_countdownSeconds seconds',
                  textAlign: TextAlign.center,
                  style: const TextStyle(
                    fontSize: 22,
                    fontWeight: FontWeight.w600,
                    color: Colors.white70,
                  ),
                ),
              ),

            // ================= EMERGENCY BUTTON =================
            Positioned(
              top: 20,
              right: 20,
              child: ElevatedButton(
                style: ElevatedButton.styleFrom(
                  backgroundColor: emergencyActive
                      ? const Color(0xFF690A0A)
                      : Colors.red,
                  foregroundColor: Colors.white,
                  padding: const EdgeInsets.symmetric(
                      horizontal: 32, vertical: 20),
                  shape: RoundedRectangleBorder(
                    borderRadius: BorderRadius.circular(12),
                  ),
                ),
                onPressed: () {
                  setState(() => emergencyActive = !emergencyActive);
                  _apiService.setLed(emergencyActive ? 2 : 9);
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
