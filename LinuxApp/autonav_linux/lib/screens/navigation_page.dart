import 'package:flutter/material.dart';
import '../services/api_service.dart';
import './animation.dart';

class NavigationPage extends StatefulWidget {
  final String mapName;

  const NavigationPage({super.key, required this.mapName});

  @override
  State<NavigationPage> createState() => _NavigationPageState();
}

class _NavigationPageState extends State<NavigationPage> {
  late Future<MapPoints> pointsFuture;

  String? selectedPoint;
  String? selectedMap;
  bool emergencyActive = false;
  bool navstatus = false;

  bool canNavigate = false;
  bool canResume = false;
  bool canStop = false;

  final ApiService _apiService = ApiService();

  @override
  void initState() {
    super.initState();
    emergencyActive = _apiService.emergencyActive;

    if (emergencyActive) {
      _apiService.setLed(2); // Set LED to emergency status
    } else {
      _apiService.setLed(4);
    }

    pointsFuture = _apiService.fetchLocalizationPoints(widget.mapName);
    selectedMap = widget.mapName;
  }

  // ---------------- ACTIONS ----------------
  Future<void> startNavigation(String action) async {
    print("API CALL: $action -> $selectedPoint");

    await _apiService.navigateTo(selectedMap!, selectedPoint!);

    if (emergencyActive) {
      _apiService.setLed(2); // Set LED to emergency status
    } else {
      _apiService.setLed(9);
    }

    // final result = await Navigator.push(
    //   context,
    //   MaterialPageRoute(
    //     builder: (context) => AnimationPage(targetPoint: selectedPoint!),
    //   ),
    // );

    final result = await Navigator.push(
        context,
        MaterialPageRoute(builder: (context) => AnimationPage(selectedMap: selectedMap!, targetPoint: selectedPoint!)),
      );

    if (result != null && result is Map<String, bool>) {
      setState(() {
        emergencyActive = result['emergencyActive'] ?? false;
        navstatus = result['navstatus'] ?? false;
      });
    }

    if (navstatus) {
      print('Navigation Succeeded');
      setState(() {
        selectedPoint = null;
        canNavigate = false;
        canResume = false;
        canStop = false;
      });
    } else {
      print('Navigation Failed/Cancelled');
      setState(() {
        canNavigate = false;
        canResume = true;
        canStop = true;
      });
    }

    await _apiService.setLed(emergencyActive ? 2 : 4);
  }

  void onNavigate() => startNavigation("navigate");
  void onResume() => startNavigation("resume");

  void onStop() {
    print("API CALL: stop navigation");

    if (emergencyActive) {
      _apiService.setLed(2); // Set LED to emergency status
    } else {
      _apiService.setLed(4);
    }

    setState(() {
      selectedPoint = null;
      canNavigate = false;
      canResume = false;
      canStop = false;
    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.white,

      body: Stack(
        children: [
          // ---------- MAIN UI ----------
          FutureBuilder<MapPoints>(
            future: pointsFuture,
            builder: (context, snapshot) {
              if (!snapshot.hasData) {
                return const Center(child: CircularProgressIndicator());
              }

              final mapData = snapshot.data!;

              return Row(
                children: [
                  // ---------- LEFT SIDE ----------
                  Expanded(
                    flex: 2,
                    child: ListView(
                      padding: const EdgeInsets.fromLTRB(20, 70, 10, 20),
                      children: [
                        const Text(
                          "  Select Navigation Point",
                          style: TextStyle(
                            fontSize: 28,
                            fontWeight: FontWeight.bold,
                            color: Colors.black,
                          ),
                        ),

                        const SizedBox(height: 5),
                        
                        if (mapData.goals.isNotEmpty)
                          buildPointCard("Goal Points", mapData.goals, Colors.blue),
                        if (mapData.standby.isNotEmpty)
                          buildPointCard("Standby Points", mapData.standby, Colors.orange),
                        if (mapData.charging.isNotEmpty)
                          buildPointCard("Charging Points", mapData.charging, Colors.green),

                        // Empty area for deselect
                        GestureDetector(
                          behavior: HitTestBehavior.opaque,
                          onTap: () {
                            setState(() {
                              selectedPoint = null;
                              canNavigate = false;
                              canResume = false;
                              canStop = false;
                            });
                          },
                          child: Container(height: 200, color: Colors.transparent),
                        )
                      ],
                    ),
                  ),

                  // ---------- RIGHT SIDE (BOX) ----------
                  Expanded(
                    flex: 1,
                    child: Container(
                      margin: const EdgeInsets.fromLTRB(10, 125, 20, 20) ,
                      padding: const EdgeInsets.all(20),
                      decoration: BoxDecoration(
                        color: Colors.white,
                        borderRadius: BorderRadius.circular(18),
                        border: Border.all(color: Colors.grey.shade300),
                        boxShadow: [
                          BoxShadow(
                            color: Colors.black.withOpacity(0.06),
                            offset: const Offset(0, 4),
                            blurRadius: 8,
                          ),
                        ],
                      ),
                      child: Column(
                        children: [
                          Expanded(
                            child: Center(
                              child: selectedPoint != null
                                  ? Column(
                                      mainAxisSize: MainAxisSize.min,
                                      children: [
                                        const Text(
                                          "Selected Point",
                                          style: TextStyle(
                                            fontSize: 26,
                                            fontWeight: FontWeight.w600,
                                            color: Colors.grey,
                                          ),
                                        ),

                                        const SizedBox(height: 12),

                                        Text(
                                          selectedPoint!,
                                          style: const TextStyle(
                                            fontSize: 32,
                                            fontWeight: FontWeight.bold,
                                            color: Colors.black,
                                          ),
                                        ),
                                      ],
                                    )
                                  : const SizedBox(),
                            ),
                          ),

                          // Navigate button
                          buildBigButton(
                            label: "Navigate",
                            color: const Color.fromARGB(255, 0, 102, 254),
                            enabled: canNavigate,
                            onTap: onNavigate,
                          ),
                          const SizedBox(height: 25),

                          // Resume + Stop
                          Row(
                            children: [
                              Expanded(
                                child: buildSmallButton(
                                  label: "Resume",
                                  color: Colors.orange,
                                  enabled: canResume,
                                  onTap: onResume,   // Same as navigate
                                ),
                              ),
                              const SizedBox(width: 20),
                              Expanded(
                                child: buildSmallButton(
                                  label: "Stop",
                                  color: Colors.red,
                                  enabled: canStop,
                                  onTap: onStop,
                                ),
                              ),
                            ],
                          ),
                        ],
                      ),
                    ),
                  ),
                ],
              );
            },
          ),

          Positioned(
            top: 10,
            left: 10,
            child: IconButton(
              icon: const Icon(Icons.arrow_back, color: Colors.black87, size: 30),
              onPressed: () {
                if (emergencyActive) {
                  _apiService.setLed(2);
                } else {
                  _apiService.setLed(6);
                }
                Navigator.pop(context, _apiService.emergencyActive);
              },
            ),
          ),

          // ---------- EMERGENCY BUTTON ----------
          Positioned(
            top: 10,
            right: 10,
            child: ElevatedButton(
              style: ElevatedButton.styleFrom(
                backgroundColor:
                    emergencyActive ? const Color(0xFF690A0A) : Colors.red,
                foregroundColor: Colors.white,
                padding: const EdgeInsets.symmetric(horizontal: 32, vertical: 20),
                shape: RoundedRectangleBorder(
                    borderRadius: BorderRadius.circular(12)),
              ),
              onPressed: () {
                setState(() => emergencyActive = !emergencyActive);

                if (emergencyActive) {
                  _apiService.setLed(2);
                } else {
                  _apiService.setLed(4);
                }

                _apiService.emergencyStop(emergencyActive);
              },
              child: const Text(
                "EMERGENCY",
                style: TextStyle(fontWeight: FontWeight.bold, fontSize: 22),
              ),
            ),
          ),
        ],
      ),
    );
  }

  // ---------- POINT CARDS ----------
  Widget buildPointCard(String title, List<String> points, Color textColour) {
    return Container(
      margin: const EdgeInsets.symmetric(vertical: 12),
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: Colors.grey[50],
        borderRadius: BorderRadius.circular(16),
        border: Border.all(color: Colors.grey.shade300),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.05),
            offset: const Offset(0, 3),
            blurRadius: 8,
          ),
        ],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text(title,
              style:
                  TextStyle(
                    fontSize: 18, 
                    fontWeight: FontWeight.bold,
                    color: textColour,
                  )),
          const SizedBox(height: 12),
          Column(children: points
            .map((point) => buildPointTile(point, textColour))
            .toList(),
          ),
        ],
      ),
    );
  }

  Widget buildPointTile(String pointName, Color textColor) {
    final isSelected = selectedPoint == pointName;

    return GestureDetector(
      onTap: () {
        setState(() {
          selectedPoint = pointName;
          canNavigate = true;
          canResume = false;
          canStop = false;
        });
      },
      child: Container(
        margin: const EdgeInsets.symmetric(vertical: 7),
        padding: const EdgeInsets.symmetric(vertical: 15, horizontal: 16),
        decoration: BoxDecoration(
          color: isSelected ? Colors.blueAccent : Colors.white,
          borderRadius: BorderRadius.circular(14),
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
                fontSize: 16,
                fontWeight: FontWeight.w600,
              ),
            ),
            if (isSelected)
              const Icon(Icons.check_circle, color: Colors.white),
          ],
        ),
      ),
    );
  }

  // ---------- BUTTON WIDGETS ----------
  Widget buildBigButton({
    required String label,
    required Color color,
    required bool enabled,
    required VoidCallback onTap,
    double height = 90,
  }) {
    return SizedBox(
      width: double.infinity,
      height: height,
      child: ElevatedButton(
        onPressed: enabled ? onTap : null,
        style: ElevatedButton.styleFrom(
          backgroundColor: color,
          disabledBackgroundColor: color.withOpacity(0.5),
          padding: const EdgeInsets.symmetric(vertical: 30),
          shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(22)),
        ),
        child: Text(
          label,
          style: const TextStyle(
            fontSize: 26,
            fontWeight: FontWeight.bold,
            color: Colors.white,
          ),
        ),
      ),
    );
  }

  Widget buildSmallButton({
    required String label,
    required Color color,
    required bool enabled,
    required VoidCallback onTap,
    double height = 80,   // <--- optional height parameter
  }) {
    return SizedBox(
      height: height,
      child: ElevatedButton(
        onPressed: enabled ? onTap : null,
        style: ElevatedButton.styleFrom(
          backgroundColor: color,
          disabledBackgroundColor: color.withOpacity(0.5),
          shape: RoundedRectangleBorder(
            borderRadius: BorderRadius.circular(18),
          ),
        ),
        child: Text(
          label,
          style: const TextStyle(
            fontSize: 20,
            fontWeight: FontWeight.w600,
            color: Colors.white,
          ),
        ),
      ),
    );
  }
}
