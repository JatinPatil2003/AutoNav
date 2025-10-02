import 'dart:convert';
// import 'package:flutter/widgets.dart';
import 'package:flutter/cupertino.dart';
import 'package:http/http.dart' as http;
import '../constants/api_constants.dart';

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
}
