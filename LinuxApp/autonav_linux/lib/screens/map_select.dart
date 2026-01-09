import 'dart:ui';
import 'package:flutter/material.dart';
import '../services/api_service.dart';
import './localization_page.dart';
import './setting_page.dart';

// --------------------------------------------------------
// Custom Fast Image Loader (No flickering, no late loading)
// --------------------------------------------------------
class FastFadeImage extends StatefulWidget {
  final String url;

  const FastFadeImage({super.key, required this.url});

  @override
  State<FastFadeImage> createState() => _FastFadeImageState();
}

class _FastFadeImageState extends State<FastFadeImage> {
  Image? _image;
  bool _loaded = false;

  @override
  void initState() {
    super.initState();

    _image = Image.network(widget.url, fit: BoxFit.cover);

    // Preload image and fade it in when ready
    _image!.image
        .resolve(const ImageConfiguration())
        .addListener(
          ImageStreamListener(
            (info, _) => setState(() => _loaded = true),
            onError: (_, __) => setState(() => _loaded = true),
          ),
        );
  }

  @override
  Widget build(BuildContext context) {
    return AnimatedOpacity(
      duration: const Duration(milliseconds: 300),
      opacity: _loaded ? 1.0 : 0.3,
      child: _image,
    );
  }
}

// --------------------------------------------------------
// Main Map Selection Page
// --------------------------------------------------------
class MapSelectionPage extends StatefulWidget {
  const MapSelectionPage({super.key});

  @override
  State<MapSelectionPage> createState() => _MapSelectionPageState();
}

class _MapSelectionPageState extends State<MapSelectionPage> {
  int? selectedIndex;
  late Future<List<MapData>> mapsFuture;
  final ApiService _apiService = ApiService();
  bool isLoading = false;
  bool emergencyActive = false;

  @override
  void initState() {
    super.initState();
    emergencyActive = _apiService.emergencyActive;
    mapsFuture = _apiService.fetchMaps();

    if (emergencyActive) {
      _apiService.setLed(2); // Set LED to emergency status
    } else {
      _apiService.setLed(6); // Normal status
    }
  }

  Future<void> _onSelectMap(MapData map, int index) async {
    setState(() => isLoading = true);

    // Show loading dialog
    showDialog(
      context: context,
      barrierDismissible: false,
      builder: (_) => const Center(
        child: SizedBox(
          width: 250,
          height: 150,
          child: Card(
            elevation: 6,
            child: Padding(
              padding: EdgeInsets.all(16),
              child: Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  CircularProgressIndicator(),
                  SizedBox(height: 16),
                  Text("Loading map..."),
                ],
              ),
            ),
          ),
        ),
      ),
    );

    final response = await _apiService.loadMap(map.id);
    final name = map.name;

    if (mounted) Navigator.pop(context);

    if (response && mounted) {
      setState(() => isLoading = false);
      Navigator.push(
        context,
        MaterialPageRoute(builder: (_) => LocalizationPage(mapName: name)),
      );
    } else {
      if (mounted) {
        setState(() => isLoading = false);
        ScaffoldMessenger.of(
          context,
        ).showSnackBar(const SnackBar(content: Text("Failed to load map")));
      }
    }
  }

  @override
  Widget build(BuildContext context) {
    final width = MediaQuery.of(context).size.width;
    final height = MediaQuery.of(context).size.height;

    return Scaffold(
      backgroundColor: Colors.grey[100],
      body: Stack(
        children: [
          // Back Button
          Positioned(
            top: 10,
            left: 10,
            child: IconButton(
              icon: const Icon(Icons.arrow_back, size: 30),
              onPressed: () {
                _apiService.stopNavigation();
                if (emergencyActive) {
                  _apiService.setLed(2); // Set LED to emergency status
                } else {
                  _apiService.setLed(5); // Reset LED to normal status
                }
                Navigator.pop(context);
              },
            ),
          ),

          // Main Selection Card
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
                    offset: const Offset(0, 6),
                    blurRadius: 14,
                  ),
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
                    width: width * 0.7,
                    height: height * 0.7,
                    child: FutureBuilder<List<MapData>>(
                      future: mapsFuture,
                      builder: (context, snapshot) {
                        if (snapshot.connectionState ==
                            ConnectionState.waiting) {
                          return const Center(
                            child: CircularProgressIndicator(),
                          );
                        }
                        if (snapshot.hasError) {
                          return Center(
                            child: Text("Error: ${snapshot.error}"),
                          );
                        }
                        if (!snapshot.hasData || snapshot.data!.isEmpty) {
                          return const Center(child: Text("No maps available"));
                        }

                        final maps = snapshot.data!;

                        return GridView.builder(
                          physics: const BouncingScrollPhysics(),
                          padding: const EdgeInsets.all(12),
                          gridDelegate:
                              const SliverGridDelegateWithFixedCrossAxisCount(
                                crossAxisCount: 3,
                                crossAxisSpacing: 15,
                                mainAxisSpacing: 15,
                                childAspectRatio: 1.2,
                              ),
                          itemCount: maps.length,
                          itemBuilder: (context, index) {
                            final map = maps[index];
                            final selected = selectedIndex == index;

                            return GestureDetector(
                              onTap: () {
                                if (isLoading) return;
                                setState(() => selectedIndex = index);
                              },

                              child: Stack(
                                children: [
                                  // Thumbnail + Name
                                  AnimatedContainer(
                                    duration: const Duration(milliseconds: 200),
                                    decoration: BoxDecoration(
                                      borderRadius: BorderRadius.circular(12),
                                      border: Border.all(
                                        color: selected
                                            ? Colors.blueAccent
                                            : Colors.grey.shade300,
                                        width: selected ? 3 : 1,
                                      ),
                                    ),
                                    child: ClipRRect(
                                      borderRadius: BorderRadius.circular(12),
                                      child: Column(
                                        children: [
                                          Expanded(
                                            child: FastFadeImage(
                                              url: map.previewUrl,
                                            ),
                                          ),
                                          Container(
                                            padding: const EdgeInsets.all(6),
                                            alignment: Alignment.center,
                                            child: Text(
                                              map.name,
                                              overflow: TextOverflow.ellipsis,
                                              style: const TextStyle(
                                                fontWeight: FontWeight.w600,
                                              ),
                                            ),
                                          ),
                                        ],
                                      ),
                                    ),
                                  ),

                                  // Select Button Overlay
                                  if (selected)
                                    Container(
                                      decoration: BoxDecoration(
                                        color: Colors.black.withOpacity(0.45),
                                        borderRadius: BorderRadius.circular(12),
                                      ),
                                      alignment: Alignment.center,
                                      child: SizedBox(
                                        width: 120, // 👈 button width
                                        height: 40, // 👈 button height
                                        child: ElevatedButton(
                                          style: ElevatedButton.styleFrom(
                                            backgroundColor: Colors.blueAccent,
                                            foregroundColor: Colors.white,
                                            shape: RoundedRectangleBorder(
                                              borderRadius:
                                                  BorderRadius.circular(8),
                                            ),
                                          ),
                                          onPressed: () {
                                            if (!isLoading) {
                                              _onSelectMap(map, index);
                                            }
                                          },
                                          child: const Text(
                                            "Select",
                                            style: TextStyle(
                                              fontSize: 18,
                                              fontWeight: FontWeight.bold,
                                            ),
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

          // Settings Button
          Positioned(
            bottom: 16,
            right: 16,
            child: FloatingActionButton(
              heroTag: "settings_btn_map",
              backgroundColor: Colors.grey[300],
              onPressed: () {
                Navigator.push(
                  context,
                  MaterialPageRoute(builder: (context) => const SettingsPage()),
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
