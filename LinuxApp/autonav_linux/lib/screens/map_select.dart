import 'dart:convert';
import 'dart:ui'; // for blur
import 'package:flutter/material.dart';
import 'package:http/http.dart' as http;
import './setting_page.dart';
import '../services/api_service.dart';
import './localization_page.dart';

class MapSelectionPage extends StatefulWidget {
  const MapSelectionPage({super.key});

  @override
  State<MapSelectionPage> createState() => _MapSelectionPageState();
}

class _MapSelectionPageState extends State<MapSelectionPage> {
  int? selectedIndex;
  late Future<List<MapData>> mapsFuture;
  final ApiService _apiService = ApiService();

  @override
  void initState() {
    super.initState();
    mapsFuture = _apiService.fetchMaps();
  }

  Future<void> _onSelectMap(MapData map, int index) async {
    setState(() => selectedIndex = index);

    showDialog(
      context: context,
      barrierDismissible: false,
      builder: (_) => const AlertDialog(
        content: Row(
          children: [
            CircularProgressIndicator(),
            SizedBox(width: 16),
            Text("Loading map..."),
          ],
        ),
      ),
    );

    final response = await _apiService.loadMap(map.id);

    final String currMapName = map.name;

    if (mounted) Navigator.pop(context);

    if (response && mounted) {
      Navigator.push(
        context,
        MaterialPageRoute(
          builder: (_) => LocalizationPage(mapName: currMapName), // Replace with NavigationPage()
        ),
      );
    } else if (mounted) {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(content: Text("Failed to load map")),
      );
    }
  }

  @override
  Widget build(BuildContext context) {
    final screenHeight = MediaQuery.of(context).size.height;
    final screenWidth = MediaQuery.of(context).size.width;

    return Scaffold(
      backgroundColor: Colors.grey[100],
      body: Stack(
        children: [
          // Back button
          Positioned(
            top: 10,
            left: 10,
            child: IconButton(
              icon: const Icon(Icons.arrow_back, color: Colors.black87, size: 30),
              onPressed: () => Navigator.pop(context),
            ),
          ),

          // Main card
          Center(
            child: Container(
              padding: const EdgeInsets.all(20),
              margin: const EdgeInsets.symmetric(horizontal: 20, vertical: 40),
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
                children: [
                  Text(
                    "Select Map",
                    style: Theme.of(context).textTheme.headlineSmall?.copyWith(
                          fontWeight: FontWeight.bold,
                          color: Colors.grey[900],
                        ),
                  ),
                  const SizedBox(height: 20),

                  SizedBox(
                    width: screenWidth * 0.7,
                    height: screenHeight * 0.7,
                    child: FutureBuilder<List<MapData>>(
                      future: mapsFuture,
                      builder: (context, snapshot) {
                        if (snapshot.connectionState == ConnectionState.waiting) {
                          return const Center(child: CircularProgressIndicator());
                        } else if (snapshot.hasError) {
                          return Center(child: Text("Error: ${snapshot.error}"));
                        } else if (!snapshot.hasData || snapshot.data!.isEmpty) {
                          return const Center(child: Text("No maps available"));
                        }

                        final maps = snapshot.data!;
                        return GridView.builder(
                          padding: const EdgeInsets.all(10),
                          gridDelegate:
                              const SliverGridDelegateWithFixedCrossAxisCount(
                            crossAxisCount: 3,
                            crossAxisSpacing: 10,
                            mainAxisSpacing: 10,
                            childAspectRatio: 1.2,
                          ),
                          itemCount: maps.length,
                          itemBuilder: (context, index) {
                            final map = maps[index];
                            final isSelected = selectedIndex == index;

                            return GestureDetector(
                              onTap: () {
                                setState(() => selectedIndex = index);
                              },
                              child: Stack(
                                children: [
                                  // Thumbnail
                                  Container(
                                    decoration: BoxDecoration(
                                      borderRadius: BorderRadius.circular(12),
                                      border: Border.all(
                                        color: isSelected
                                            ? Colors.blueAccent
                                            : Colors.grey.shade300,
                                        width: isSelected ? 3 : 1,
                                      ),
                                    ),
                                    child: ClipRRect(
                                      borderRadius: BorderRadius.circular(12),
                                      child: Column(
                                        children: [
                                          Expanded(
                                            child: Image.network(
                                              map.previewUrl,
                                              fit: BoxFit.cover,
                                              width: double.infinity,
                                              errorBuilder: (_, __, ___) =>
                                                  const Icon(Icons.map, size: 40),
                                            ),
                                          ),
                                          Container(
                                            padding: const EdgeInsets.all(6),
                                            alignment: Alignment.center,
                                            child: Text(
                                              map.name,
                                              style: const TextStyle(
                                                fontWeight: FontWeight.w600,
                                                fontSize: 14,
                                              ),
                                              overflow: TextOverflow.ellipsis,
                                            ),
                                          ),
                                        ],
                                      ),
                                    ),
                                  ),

                                  // Overlay blur + Select button
                                  if (isSelected)
                                    ClipRRect(
                                      borderRadius: BorderRadius.circular(12),
                                      child: BackdropFilter(
                                        filter: ImageFilter.blur(
                                            sigmaX: 2, sigmaY: 2),
                                        child: Container(
                                          color: Colors.black.withOpacity(0.4),
                                          alignment: Alignment.center,
                                          child: ElevatedButton(
                                            style: ElevatedButton.styleFrom(
                                              backgroundColor: Colors.blueAccent,
                                              foregroundColor: Colors.white,
                                              shape: RoundedRectangleBorder(
                                                borderRadius:
                                                    BorderRadius.circular(8),
                                              ),
                                            ),
                                            onPressed: () =>
                                                _onSelectMap(map, index),
                                            child: const Text("Select"),
                                          ),
                                        ),
                                      ),
                                    ),
                                ],
                              ),
                            );
                          },
                        );
                      },
                    ),
                  ),
                ],
              ),
            ),
          ),

          // Settings button
          Positioned(
            bottom: 16,
            right: 16,
            child: FloatingActionButton(
              heroTag: "settings_btn_map",
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

