import 'dart:convert';
// import 'package:flutter/widgets.dart';
import 'package:flutter/cupertino.dart';
import 'package:http/http.dart' as http;
import '../constants/api_constants.dart';

class MapData {
  final String id;
  final String name;
  final String previewUrl;
  MapData({required this.id, required this.name, required this.previewUrl});
}

class MapPoints {
  final List<String> localization;
  final List<String> charging;
  final List<String> standby;
  final List<String> goals;
  MapPoints({required this.localization, required this.charging, required this.standby, required this.goals});
}

class ApiService {
  final String baseUrl = ApiConstants.apiFullUrl;

  /// Sends a request to start the robot
  Future<bool> startRobot() async {
    print('calling start');
    final response = await http.get(Uri.parse('$baseUrl/robot/start'));
    print(response);
    return response.statusCode == 200;
    // return true;
  }

  /// Sends a request to stop the robot
  Future<bool> stopRobot() async {
    print('calling stop');
    final response = await http.get(Uri.parse('$baseUrl/robot/stop'));
    return response.statusCode == 200;
    // return true;
  }

  /// Example: fetch robot status
  Future<String> getStatus() async {
    print('calling status');
    final response = await http.get(Uri.parse('$baseUrl/robot/status'));
    if (response.statusCode == 200) {
      final body = jsonDecode(response.body);
      return body['status'] ?? "unknown";
    }
    return "error";
  }

  Future<bool> startMapping() async {
    print('calling mapping start');
    final response = await http.get(Uri.parse('$baseUrl/mapping/start'));
    return response.statusCode == 200;
    // return true;
  }

  Future<bool> stopMapping() async {
    print('calling mapping stop');
    final response = await http.get(Uri.parse('$baseUrl/mapping/stop'));
    return response.statusCode == 200;
    // return true;
  }

  Future<bool> saveMap(String name) async {
    try {
      final response = await http.post(
        Uri.parse('$baseUrl/mapping/save_map'),
        headers: {"Content-Type": "application/json"},
        body: jsonEncode({"name": name}),
      );
      return response.statusCode == 200;
    } catch (_) {
      return false;
    }
  }

  Future<bool> savemapPoints(String name) async {
    try {
      final response = await http.post(
        Uri.parse('$baseUrl/mapping/save_map_points'),
        headers: {"Content-Type": "application/json"},
        body: jsonEncode({"name": name}),
      );
      return response.statusCode == 200;
    } catch (_) {
      return false;
    }
  }

  Future<bool> savePose(String name) async {
    try {
      final response = await http.post(
        Uri.parse('$baseUrl/mapping/save_pose'),
        headers: {"Content-Type": "application/json"},
        body: jsonEncode({"name": name}),
      );
      debugPrint(response.body);
      return response.statusCode == 200;
    } catch (_) {
      return false;
    }
  }

  Future<void> localization() async {
    await http.post(Uri.parse('$baseUrl/localization'));
  }

  Future<void> charger() async {
    await http.post(Uri.parse('$baseUrl/charger'));
  }

  Future<void> standby() async {
    await http.post(Uri.parse('$baseUrl/standby'));
  }

  Future<void> emergencyStop() async {
    await http.post(Uri.parse('$baseUrl/emergency'));
  }

  // Fetch all available maps
  Future<List<MapData>> fetchMaps() async {
    final response = await http.get(Uri.parse('$baseUrl/navigation/list/maps_linux'));

    if (response.statusCode == 200) {
      final List<dynamic> data = jsonDecode(response.body);
      return data
          .map((e) => MapData(
                name: e['name'],
                previewUrl: '$baseUrl${e['thumbnailUrl']}',
                id: e['id'],
              ))
          .toList();
    } else {
      throw Exception("Failed to load maps");
    }
  }

  // Fetch a single map by name
  Future<MapData> fetchMapByName(String mapName) async {
    final response = await http.post(
      Uri.parse('$baseUrl/navigation/get_map'),
      headers: {'Content-Type': 'application/json'},
      body: jsonEncode({'name': mapName}),
    );

    if (response.statusCode == 200) {
      final data = jsonDecode(response.body);
      return MapData(
        name: data['name'],
        previewUrl: '$baseUrl${data['thumbnailUrl']}',
        id: data['id'],
      );
    } else {
      throw Exception("Failed to fetch map: ${response.body}");
    }
  }

  // Select / load a specific map
  Future<bool> loadMap(String mapId) async {
    print('Loading map with ID: $mapId');
    final response = await http.post(
      Uri.parse('$baseUrl/navigation/use_map_linux'),
      headers: {"Content-Type": "application/json"},
      body: jsonEncode({"name": mapId}),
    );
    return response.statusCode == 200;
  }

  Future<MapPoints> fetchLocalizationPoints(String mapName) async {
    final response = await http.post(
      Uri.parse("$baseUrl/navigation/points"),
      body: jsonEncode({"map_name": mapName}),
      headers: {"Content-Type": "application/json"},
    );

    if (response.statusCode == 200) {
      final data = jsonDecode(response.body);
      return MapPoints(
        localization: List<String>.from(data['localization']),
        charging: List<String>.from(data['charging']),
        standby: List<String>.from(data['standby']),
        goals: List<String>.from(data['goals']),
      );
    } else {
      return MapPoints(
        localization: ["Entrance", "Kitchen", "Lobby", "Storage Room"],
        charging: ["Dock 1", "Dock 2"],
        standby: ["Standby 1", "Standby 2"],
        goals: ["Goal A", "Goal B", "Goal C", "Goal D"],
        // goals: [""],
      );
      // throw Exception("Failed to fetch points");
    }
  }
}
