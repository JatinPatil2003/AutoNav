import 'package:flutter/material.dart';
import '../services/api_service.dart';
import './navigation_page.dart';

class LocalizationPage extends StatefulWidget {
  final String mapName;

  const LocalizationPage({super.key, required this.mapName});

  @override
  State<LocalizationPage> createState() => _LocalizationPageState();
}

class _LocalizationPageState extends State<LocalizationPage> {
  String? selectedPoint;
  late String activeMap;
  bool isLoading = false;
  late bool emergencyActive;

  late Future<MapData> mapFuture;
  late Future<MapPoints> pointsFuture; // <-- New future
  final ApiService _apiService = ApiService();
  final ScrollController _scrollController = ScrollController();


  @override
  void initState() {
    super.initState();

    emergencyActive = _apiService.emergencyActive;

    if (emergencyActive) {
      _apiService.setLed(2); // Set LED to emergency status
    } else {
      _apiService.setLed(6);
    }

    activeMap = widget.mapName;

    mapFuture = _apiService.fetchMapByName(widget.mapName);
    pointsFuture = _apiService.fetchLocalizationPoints(widget.mapName);
  }

  Future<void> localizeRobot(String pointName) async {
    if (!mounted) return;

    setState(() => isLoading = true);

    final result = await _apiService.localizeAt(widget.mapName, pointName);
    final bool success = result["success"] ?? false;
    final String message = result["message"] ?? "Unknown response";

    // Short delay so UI feels responsive (optional)
    await Future.delayed(const Duration(milliseconds: 300));

    if (!mounted) return;

    setState(() => isLoading = false);

    // Show Success or Failure SnackBar based on API response
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        behavior: SnackBarBehavior.floating,
        backgroundColor:
            success ? Colors.green.shade600 : Colors.red.shade600,
        elevation: 6,
        margin: const EdgeInsets.only(
          bottom: 10,
          left: 300,
          right: 300,
        ),
        shape: RoundedRectangleBorder(
          borderRadius: BorderRadius.circular(12),
        ),
        content: Text(
          message,   // <--- Use server message
          textAlign: TextAlign.center,
          style: const TextStyle(
            color: Colors.white,
            fontSize: 17,
            fontWeight: FontWeight.bold,
          ),
        ),
      ),
    );

    // If failed, do not continue navigation
    if (!success) return;

    await Future.delayed(const Duration(seconds: 2));

    if (!mounted) return;

    // Navigate to NavigationPage after success
    final res = await Navigator.push(
        context,
        MaterialPageRoute(builder: (context) => NavigationPage(mapName: activeMap)),
      );

    if (res != null) {
      setState(() {
        emergencyActive = res as bool;
      });
    }
  }

  Widget buildPointCard(String title, List<String> points) {
    return Container(
      margin: const EdgeInsets.symmetric(vertical: 10),
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: Colors.grey[50],
        borderRadius: BorderRadius.circular(16),
        border: Border.all(color: Colors.grey.shade300, width: 1),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.04),
            offset: const Offset(0, 4),
            blurRadius: 8,
          ),
        ],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text(
            title,
            style: Theme.of(context).textTheme.titleMedium?.copyWith(
                  fontWeight: FontWeight.bold,
                  color: Colors.grey[900],
                ),
          ),
          const SizedBox(height: 10),
          Column(
            children: points.map((p) => buildPointTile(p)).toList(),
          ),
        ],
      ),
    );
  }

  Widget buildPointTile(String pointName) {
    final isSelected = selectedPoint == pointName;
    return GestureDetector(
      onTap: () => setState(() => selectedPoint = pointName),
      child: Container(
        margin: const EdgeInsets.symmetric(vertical: 6),
        padding: const EdgeInsets.symmetric(vertical: 14, horizontal: 16),
        decoration: BoxDecoration(
          color: isSelected ? Colors.blueAccent : Colors.white,
          borderRadius: BorderRadius.circular(12),
          border: Border.all(
            color: isSelected ? Colors.blueAccent : Colors.grey.shade300,
            width: isSelected ? 2 : 1,
          ),
        ),
        child: Row(
          mainAxisAlignment: MainAxisAlignment.spaceBetween,
          children: [
            Text(
              pointName,
              style: TextStyle(
                color: isSelected ? Colors.white : Colors.black87,
                fontWeight: FontWeight.w600,
                fontSize: 15,
              ),
            ),
            if (isSelected)
              const Icon(Icons.check_circle, color: Colors.white, size: 20),
          ],
        ),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    final width = MediaQuery.of(context).size.width;

    return Scaffold(
      backgroundColor: Colors.grey[100],
      body: Stack(
        children: [
          // Main content
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
                  ),
                ],
              ),
              child: Column(
                children: [
                  // Row: text + map
                  Row(
                    children: [
                      // Left text
                      const SizedBox(width: 150),

                      Expanded(
                        flex: 2,
                        child: Padding(
                          padding: const EdgeInsets.all(8.0),
                          child: Column(
                            crossAxisAlignment: CrossAxisAlignment.start,
                            children: [
                              Text(
                                "Localization",
                                style: Theme.of(context)
                                    .textTheme
                                    .headlineSmall
                                    ?.copyWith(
                                        fontWeight: FontWeight.bold,
                                        color: Colors.grey[900]),
                              ),
                              const SizedBox(height: 4),
                              Text(
                                textAlign: TextAlign.center,
                                "Map: ${widget.mapName}",
                                style: const TextStyle(
                                  fontSize: 15,
                                  color: Colors.black54,
                                  fontWeight: FontWeight.w500,
                                ),
                              ),
                            ],
                          ),
                        ),
                      ),

                      // Right: Map preview
                      Expanded(
                        flex: 3,
                        child: FutureBuilder<MapData>(
                          future: mapFuture,
                          builder: (context, snapshot) {
                            if (snapshot.connectionState ==
                                ConnectionState.waiting) {
                              return const Center(
                                child: SizedBox(
                                  height: 120,
                                  width: 120,
                                  child: CircularProgressIndicator(),
                                ),
                              );
                            } else if (snapshot.hasError) {
                              return Padding(
                                padding: const EdgeInsets.all(8),
                                child: Text(
                                  "Error loading map",
                                  style: TextStyle(color: Colors.red.shade400),
                                ),
                              );
                            } else if (!snapshot.hasData) {
                              return const Padding(
                                padding: EdgeInsets.all(8),
                                child: Text("Map not found"),
                              );
                            }

                            final map = snapshot.data!;
                            return ClipRRect(
                              borderRadius: BorderRadius.circular(16),
                              child: SizedBox(
                                height: 200,
                                child: Image.network(
                                  map.previewUrl,
                                  fit: BoxFit.contain,
                                  alignment: Alignment.center,
                                  errorBuilder: (_, __, ___) => Container(
                                    color: Colors.grey[200],
                                    alignment: Alignment.center,
                                    child: const Icon(Icons.map,
                                        size: 40, color: Colors.grey),
                                  ),
                                ),
                              ),
                            );
                          },
                        ),
                      ),
                    ],
                  ),

                  const SizedBox(height: 20),

                  // Scrollable points list (from API)
                  FutureBuilder<MapPoints>(
                    future: pointsFuture,
                    builder: (context, snapshot) {
                      if (snapshot.connectionState == ConnectionState.waiting) {
                        return const Center(
                            child: CircularProgressIndicator());
                      } else if (snapshot.hasError) {
                        return Padding(
                          padding: const EdgeInsets.all(8.0),
                          child: Text(
                            "Error loading points",
                            style: TextStyle(color: Colors.red.shade400),
                          ),
                        );
                      } else if (!snapshot.hasData) {
                        return const Padding(
                          padding: EdgeInsets.all(8.0),
                          child: Text("No points available"),
                        );
                      }

                      final points = snapshot.data!;
                      return Expanded(
                        child: Scrollbar(
                          controller: _scrollController,     // <-- attach controller
                          thumbVisibility: true,
                          trackVisibility: true,
                          thickness: 14,                     // <-- big scrollbar
                          radius: const Radius.circular(5),
                          child: SingleChildScrollView(
                            controller: _scrollController,   // <-- attach same controller
                            // padding: const EdgeInsets.symmetric(horizontal: 1),
                            child: Column(
                              children: [
                                // ---- Show "No points" if everything is empty ----
                                if (points.localization.isEmpty && points.charging.isEmpty)
                                  Padding(
                                    padding: const EdgeInsets.only(top: 80),
                                    child: Text(
                                      "No points added to this map",
                                      style: TextStyle(
                                        fontSize: 18,
                                        color: Colors.grey[600],
                                        fontWeight: FontWeight.w500,
                                      ),
                                    ),
                                  ),

                                // ---- Show cards only if data exists ----
                                if (points.localization.isNotEmpty)
                                  buildPointCard("Localization Points", points.localization),

                                if (points.charging.isNotEmpty)
                                  buildPointCard("Charging Points", points.charging),
                              ],
                            ),
                          ),
                        ),
                      );
                    },
                  ),
                ],
              ),
            ),
          ),

          // Emergency Button
          Positioned(
            top: 15,
            right: 15,
            child: ElevatedButton(
              style: ElevatedButton.styleFrom(
                backgroundColor: emergencyActive ? const Color(0xFF690A0A) : Colors.red,
                foregroundColor: Colors.white,
                padding:
                    const EdgeInsets.symmetric(horizontal: 25, vertical: 20),
                shape: RoundedRectangleBorder(
                    borderRadius: BorderRadius.circular(12)),
              ),
              onPressed: () {
                setState(() => emergencyActive = !emergencyActive);
                if (emergencyActive) {
                  _apiService.setLed(2); // Set LED to emergency status
                } else {
                  _apiService.setLed(6); // Reset LED to normal status
                }
                _apiService.emergencyStop(emergencyActive);
              },
              child: const Text(
                "EMERGENCY",
                style: TextStyle(fontWeight: FontWeight.bold, fontSize: 18),
              ),
            ),
          ),

          // Back button
          Positioned(
            top: 10,
            left: 10,
            child: IconButton(
              icon: const Icon(Icons.arrow_back, color: Colors.black87, size: 30),
              onPressed: () {
                if (emergencyActive) {
                  _apiService.setLed(2); // Set LED to emergency status
                } else {
                  _apiService.setLed(6); // Reset LED to normal status
                }
                Navigator.pop(context);
              },
            ),
          ),

          // Bottom button
          if (selectedPoint != null)
            Align(
              alignment: Alignment.bottomCenter,
              child: Container(
                width: width,
                color: Colors.white,
                padding: const EdgeInsets.symmetric(horizontal: 400, vertical: 12),
                child: ElevatedButton(
                  style: ElevatedButton.styleFrom(
                    backgroundColor: Colors.blueAccent,
                    padding: const EdgeInsets.symmetric(vertical: 16),
                    shape: RoundedRectangleBorder(
                      borderRadius: BorderRadius.circular(10),
                    ),
                  ),
                  onPressed: isLoading ? null : () async => await localizeRobot(selectedPoint!),
                  child: isLoading
                      ? Row(
                          mainAxisAlignment: MainAxisAlignment.center,
                          mainAxisSize: MainAxisSize.min,
                          children: const [
                            SizedBox(
                              width: 26,
                              height: 26,
                              child: CircularProgressIndicator(
                                strokeWidth: 5,
                                color: Colors.green,
                              ),
                            ),
                            SizedBox(width: 12),
                            Text(
                              "Localizing ...",
                              style: TextStyle(
                                fontSize: 17,
                                color: Colors.green,
                                fontWeight: FontWeight.w600,
                              ),
                            ),
                          ],
                        )
                      : Text(
                          "Localize to $selectedPoint",
                          style: const TextStyle(
                            fontSize: 17,
                            color: Colors.white,
                            fontWeight: FontWeight.w600,
                          ),
                        ),
                ),
              )
            ),
        ],
      ),
    );
  }
}
