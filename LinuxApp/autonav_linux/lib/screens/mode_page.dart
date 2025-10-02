import 'package:flutter/material.dart';
import './setting_page.dart';
import './mapping_page.dart';
import './navigation_page.dart';
import '../services/api_service.dart';

class ModePage extends StatefulWidget {
  const ModePage({super.key});

  @override
  State<ModePage> createState() => _ModePageState();
}

class _ModePageState extends State<ModePage> {
  bool isLoading = false;
  String loadingMessage = "";

  final ApiService _apiService = ApiService();

  Future<void> _startMapping() async {
    setState(() {
      isLoading = true;
      loadingMessage = "Starting Mapping...";
    });

    await _apiService.startMapping();

    await Future.delayed(const Duration(seconds: 5));

    if (!mounted) return;
    setState(() => isLoading = false);

    Navigator.push(
      context,
      MaterialPageRoute(builder: (context) => const MappingPage()),
    );
  }

  Future<void> _startNavigation() async {
    setState(() {
      isLoading = true;
      loadingMessage = "Loading Maps...";
    });

    await Future.delayed(const Duration(seconds: 3));

    if (!mounted) return;
    setState(() {
      loadingMessage = "Starting Navigation...";
    });

    await Future.delayed(const Duration(seconds: 5));

    if (!mounted) return;
    setState(() => isLoading = false);

    Navigator.push(
      context,
      MaterialPageRoute(builder: (context) => const NavigationPage()),
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
            left: 10,
            child: IconButton(
              icon: const Icon(Icons.arrow_back, color: Colors.black87, size: 30),
              onPressed: () => Navigator.pop(context),
            ),
          ),

          // Main content
          Center(
            child: Container(
              padding: const EdgeInsets.all(24),
              margin: const EdgeInsets.symmetric(horizontal: 32),
              decoration: BoxDecoration(
                color: Colors.white,
                borderRadius: BorderRadius.circular(20),
                boxShadow: [
                  BoxShadow(
                    color: Colors.black.withOpacity(0.08),
                    offset: const Offset(0, 8),
                    blurRadius: 16,
                  )
                ],
              ),
              child: Column(
                mainAxisSize: MainAxisSize.min,
                children: [
                  Text(
                    "Choose Mode",
                    style: Theme.of(context).textTheme.headlineSmall?.copyWith(
                          fontWeight: FontWeight.bold,
                          color: Colors.grey[900],
                        ),
                  ),
                  const SizedBox(height: 40),

                  if (isLoading)
                    Column(
                      children: [
                        const CircularProgressIndicator(
                          color: Colors.blueAccent,
                          strokeWidth: 4,
                        ),
                        const SizedBox(height: 16),
                        Text(
                          loadingMessage,
                          style: const TextStyle(
                            fontSize: 18,
                            color: Colors.grey,
                          ),
                        ),
                      ],
                    )
                  else ...[
                    ElevatedButton(
                      style: ElevatedButton.styleFrom(
                        padding: const EdgeInsets.symmetric(
                            horizontal: 40, vertical: 20),
                        backgroundColor: Colors.blueAccent,
                        foregroundColor: Colors.white,
                        elevation: 6,
                        shadowColor: Colors.black26,
                        shape: RoundedRectangleBorder(
                          borderRadius: BorderRadius.circular(16),
                        ),
                        minimumSize: const Size(220, 60),
                      ),
                      onPressed: _startMapping,
                      child: const Text(
                        "Mapping",
                        style: TextStyle(
                            fontSize: 22, fontWeight: FontWeight.bold),
                      ),
                    ),
                    const SizedBox(height: 30),
                    ElevatedButton(
                      style: ElevatedButton.styleFrom(
                        padding: const EdgeInsets.symmetric(
                            horizontal: 40, vertical: 20),
                        backgroundColor: Colors.green,
                        foregroundColor: Colors.white,
                        elevation: 6,
                        shadowColor: Colors.black26,
                        shape: RoundedRectangleBorder(
                          borderRadius: BorderRadius.circular(16),
                        ),
                        minimumSize: const Size(220, 60),
                      ),
                      onPressed: _startNavigation,
                      child: const Text(
                        "Navigation",
                        style: TextStyle(
                            fontSize: 22, fontWeight: FontWeight.bold),
                      ),
                    ),
                  ]
                ],
              ),
            ),
          ),

          // Settings icon
          Positioned(
            bottom: 16,
            right: 16,
            child: FloatingActionButton(
              heroTag: "settings_btn_mode",
              backgroundColor: Colors.grey[300],
              onPressed: () {
                Navigator.push(
                  context,
                  MaterialPageRoute(
                    builder: (context) => const SettingsPage(),
                  ),
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
