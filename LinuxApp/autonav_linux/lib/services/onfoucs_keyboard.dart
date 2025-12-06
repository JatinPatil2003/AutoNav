import 'package:flutter/material.dart';
import 'dart:io';

class AutoFocusKeyboard extends StatefulWidget {
  final TextEditingController controller;
  final String hintText;
  final bool obscureText;
  final int maxLines;
  final Function(String)? onChanged;

  const AutoFocusKeyboard({
    super.key,
    required this.controller,
    this.hintText = "",
    this.obscureText = false,
    this.maxLines = 1,
    this.onChanged,
  });

  @override
  State<AutoFocusKeyboard> createState() => _AutoFocusKeyboardState();
}

class _AutoFocusKeyboardState extends State<AutoFocusKeyboard> {
  late FocusNode _focusNode;
  Process? _onboardProcess;

  @override
  void initState() {
    super.initState();
    _focusNode = FocusNode();

    _focusNode.addListener(() {
      if (_focusNode.hasFocus) {
        _showKeyboard();
      } else {
        _hideKeyboard();
      }
    });

    // Auto-focus when the widget is built
    WidgetsBinding.instance.addPostFrameCallback((_) {
      if (mounted) {
        FocusScope.of(context).requestFocus(_focusNode);
      }
    });
  }

  Future<void> _showKeyboard() async {
    if (_onboardProcess != null) return;
    try {
      _onboardProcess = await Process.start("/home/jatin/Desktop/start_keyboard.sh", []);
    } catch (e) {
      print("Failed to launch onboard: $e");
    }
  }

  void _hideKeyboard() {
    if (_onboardProcess != null) {
      Process.start("/home/jatin/Desktop/stop_keyboard.sh", []);
      _onboardProcess = null;
    }
  }

  @override
  void dispose() {
    _hideKeyboard();
    _focusNode.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return TextField(
      controller: widget.controller,
      focusNode: _focusNode,
      obscureText: widget.obscureText,
      maxLines: widget.maxLines,
      decoration: InputDecoration(
        hintText: widget.hintText,
        // border: const OutlineInputBorder(),
      ),
      onChanged: widget.onChanged,
    );
  }
}
