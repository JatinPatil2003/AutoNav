import 'package:flutter/material.dart';
import 'screens/welcome_page.dart';

void main() {
  runApp(const AutoNavApp());
}

class AutoNavApp extends StatelessWidget {
  const AutoNavApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'AutoNav',
      debugShowCheckedModeBanner: false,
      // theme: ThemeData.light(),
      home: const WelcomePage(),
    );
  }
}
