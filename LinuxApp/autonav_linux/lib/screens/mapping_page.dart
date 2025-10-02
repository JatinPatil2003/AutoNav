import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:web_socket_channel/web_socket_channel.dart';
import 'package:http/http.dart' as http;
import './map_view.dart';
import '../constants/api_constants.dart';
import '../services/api_service.dart';

class MappingPage extends StatefulWidget {
  const MappingPage({super.key});

  @override
  State<MappingPage> createState() => _MappingPageState();
}

class _MappingPageState extends State<MappingPage> {
  late WebSocketChannel channel;
  Map<String, dynamic>? robotPosition;
  List<Map<String, dynamic>> savedPoints = [];
  Map<String, dynamic>? currentMapData;
  bool emergencyActive = false;

  final String wsbaseUrl = ApiConstants.wsFullUrl;
  final ApiService _apiService = ApiService();

  @override
  void initState() {
    super.initState();
    channel = WebSocketChannel.connect(
      Uri.parse('$wsbaseUrl/autonav'), // replace with your WS
    );

    channel.stream.listen((message) {
      final data = jsonDecode(message);

      if (!mounted) return;

      if (data['type'] == 'location') {
        final pos = data['data'];
        setState(() {
          robotPosition = {
            'x': pos['x'].toDouble(),
            'y': pos['y'].toDouble(),
            'theta': pos['theta'].toDouble(),
          };
        });
      } else if (data['type'] == 'map') {
        setState(() {
          currentMapData = data['data']; // Map<String, dynamic>
          // print(data['data']);
        });
      }
    });
  }

  Future<void> _sendApi(String endpoint, {String? name}) async {
    try {
      final res = await http.post(
        Uri.parse("http://localhost:5000$endpoint"),
        body: name != null ? {"name": name} : null,
      );
      debugPrint("API $endpoint response: ${res.statusCode}");
    } catch (e) {
      debugPrint("API error: $e");
    }
  }

  void _showActionDialog(String title, String endpoint, {bool requiresInput = false}) {
    final TextEditingController controller = TextEditingController();
    bool isValid = false;

    showDialog(
      context: context,
      builder: (ctx) {
        return StatefulBuilder(
          builder: (context, setState) => AlertDialog(
            title: Text(title),
            content: requiresInput
                ? TextField(
                    controller: controller,
                    autofocus: true,
                    decoration: InputDecoration(hintText: "$title name"),
                    onChanged: (value) {
                      setState(() {
                        isValid = value.trim().isNotEmpty;
                      });
                    },
                  )
                : const Text("Confirm action?"),
            actions: [
              TextButton(
                onPressed: () => Navigator.pop(ctx),
                child: const Text("Cancel"),
              ),
              ElevatedButton(
                onPressed: (!requiresInput || isValid)
                  ? () async {
                      String? name;

                      if (requiresInput) {
                        name = controller.text.trim();
                      } 
                      else {
                        if (title == "Localization") {
                          name = "localization";
                        } else if (title == "Charger") {
                          name = "charger";
                        } else if (title == "Standby") {
                          name = "standby";
                        }
                      }

                      if (title == "Save Map") {
                        await _apiService.saveMap(name.toString());
                        await _apiService.savemapPoints(name.toString());

                        _apiService.stopMapping();

                        Navigator.pop(ctx);

                        channel.sink.close();
                        
                        if (Navigator.of(context).canPop()){
                          Navigator.of(context).pop();
                        }
                      } else {
                        await _apiService.savePose(name.toString());
                        if (robotPosition != null) {
                          setState(() {
                            savedPoints.add({
                              'name': name,
                              'position': robotPosition!,
                            });
                          });
                        }
                        Navigator.pop(ctx);
                      }
                    }
                  : null,
                child: const Text("Save"),
              ),
            ],
          ),
        );
      },
    );
  }

  @override
  void dispose() {
    channel.sink.close();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.black,
      body: SafeArea(
        child: Stack(
          children: [
            // Map area
            Positioned.fill(
              child: MapView(
                mapData: currentMapData,
                robotPosition: robotPosition,
                savedPoints: savedPoints,
              ),
            ),

            // Top bar: back + emergency
            Positioned(
              top: 10,
              left: 10,
              child: IconButton(
                icon: const Icon(Icons.arrow_back, color: Colors.white, size: 30),
                onPressed: () => {
                  _apiService.stopMapping(),
                  Navigator.pop(context),
                },
              ),
            ),
            Positioned(
              top: 10,
              right: 10,
              child: ElevatedButton(
                style: ElevatedButton.styleFrom(
                  backgroundColor: emergencyActive ? const Color(0xFF690A0A) : Colors.red,
                  foregroundColor: Colors.white,
                  padding: const EdgeInsets.symmetric(horizontal: 32, vertical: 20),
                  shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
                ),
                onPressed: () {
                  setState(() => emergencyActive = !emergencyActive);
                  _sendApi("/emergency");
                },
                child: const Text(
                  "EMERGENCY",
                  style: TextStyle(fontWeight: FontWeight.bold, fontSize: 22),
                ),
              ),
            ),

            // Right side buttons box
            Positioned(
              right: 20,
              top: MediaQuery.of(context).size.height * 0.25,
              child: Container(
                padding: const EdgeInsets.all(12),
                decoration: BoxDecoration(
                  color: const Color.fromARGB(255, 120, 93, 165).withAlpha(100),
                  borderRadius: BorderRadius.circular(12),
                ),
                child: Column(
                  children: [
                    _buildActionButton(Icons.save, "Save Map", () {
                      _showActionDialog("Save Map", "/mapping/save_map", requiresInput: true);
                    }),
                    const SizedBox(height: 10),
                    _buildActionButton(Icons.add_location, "Save Point", () {
                      _showActionDialog("Save Point", "/save_point", requiresInput: true);
                    }),
                    const SizedBox(height: 10),
                    _buildActionButton(Icons.my_location, "Localization", () {
                      _showActionDialog("Localization", "/localization");
                    }),
                    const SizedBox(height: 10),
                    _buildActionButton(Icons.battery_charging_full, "Charger", () {
                      _showActionDialog("Charger", "/charger");
                    }),
                    const SizedBox(height: 10),
                    _buildActionButton(Icons.pause_circle, "Standby", () {
                      _showActionDialog("Standby", "/standby");
                    }),
                  ],
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildActionButton(IconData icon, String label, VoidCallback onTap) {
    return InkWell(
      onTap: onTap,
      borderRadius: BorderRadius.circular(8),
      child: Container(
        width: 90,  // fixed width
        height: 70, // fixed height
        padding: const EdgeInsets.all(8),
        decoration: BoxDecoration(
          color: Colors.deepPurple[200],
          borderRadius: BorderRadius.circular(8),
        ),
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Icon(icon, color: Colors.white, size: 28),
            const SizedBox(height: 6),
            Text(
              label,
              textAlign: TextAlign.center,
              maxLines: 2,
              overflow: TextOverflow.ellipsis,
              style: const TextStyle(
                color: Colors.white,
                fontSize: 12,
                fontWeight: FontWeight.w600,
              ),
            ),
          ],
        ),
      ),
    );
  }

}
