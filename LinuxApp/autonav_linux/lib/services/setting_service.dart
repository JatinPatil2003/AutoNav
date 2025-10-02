import 'package:http/http.dart' as http;
import 'dart:convert';
import '../constants/api_constants.dart';
import 'dart:io';

class SettingsService {
  final String baseUrl = ApiConstants.apiFullUrl;

  // Stub: update WiFi settings
  Future<bool> updateWifi(String ssid, String password) async {
    try {
      // Example POST request
      final response = await http.post(
        Uri.parse('$baseUrl/wifi/connect'),
        headers: {'Content-Type': 'application/json'},
        body: jsonEncode({'ssid': ssid, 'password': password}),
      );
      return response.statusCode == 200;
    } catch (e) {
      print("WiFi update error: $e");
      return false;
    }
  }

  Future<List<String>> fetchWifiNetworks() async {
    try {
      final response = await http.get(
        Uri.parse('$baseUrl/wifi/list'),
        headers: {'Content-Type': 'application/json'},
      );

      if (response.statusCode == 200) {
        final data = jsonDecode(response.body);
        if (data is Map && data.containsKey("networks")) {
          return List<String>.from(data["networks"]);
        } else {
          throw Exception("Invalid response format");
        }
      } else {
        throw Exception("Failed to fetch Wi-Fi networks: ${response.statusCode}");
      }
    } catch (e) {
      print("WiFi update error: $e");
      // Fallback list
      return ["HomeWiFi", "OfficeWiFi", "GuestWiFi", "CafeWiFi", "PublicWiFi"];
    }
  }

  Future<String> fetchConnectedNetwork() async {
    try {
      final response = await http.get(Uri.parse('$baseUrl/wifi/current'));

      if (response.statusCode == 200) {
        final data = jsonDecode(response.body);
        if (data is Map && data.containsKey("current")) {
          return data["current"];
        } else {
          throw Exception("Invalid response format");
        }
      } else {
        throw Exception("Failed to fetch Wi-Fi networks: ${response.statusCode}");
      }
    } catch (e) {
      print("WiFi update error: $e");
      // Fallback list
      return "OfficeWiFi";
    }
  }

  // Stub: toggle VPN
  Future<bool> toggleVpn(bool enabled) async {
    try {
      // Determine the script path relative to current Dart/Flutter directory
      String scriptName = "";
      if (enabled){
        scriptName = "start_vpn.sh";
      }
      else{
        scriptName = "stop_vpn.sh";
      }

      // If running Flutter desktop, current directory should be project root
      String basePath = Directory.current.path;

      // Full path to script
      String scriptPath = "$basePath/lib/scripts/$scriptName";
      print("Reboot script path: $scriptPath");

      // Make sure the script exists
      if (!File(scriptPath).existsSync()) {
        print("Reboot script not found at $scriptPath");
        return false;
      }

      // Run the script asynchronously
      ProcessResult result = await Process.run('bash', [scriptPath]);

      if (result.exitCode == 0) {
        print("Reboot script executed successfully");
        return true;
      } else {
        print("Reboot script failed: ${result.stderr}");
        return false;
      }
    } catch (e) {
      print("Reboot error: $e");
      return false;
    }
  }

  // Stub: reboot robot
  Future<bool> rebootRobot() async {
    try {
      // Determine the script path relative to current Dart/Flutter directory
      String scriptName = "restart.sh";

      // If running Flutter desktop, current directory should be project root
      String basePath = Directory.current.path;

      // Full path to script
      String scriptPath = "$basePath/lib/scripts/$scriptName";
      print("Reboot script path: $scriptPath");

      // Make sure the script exists
      if (!File(scriptPath).existsSync()) {
        print("Reboot script not found at $scriptPath");
        return false;
      }

      // Run the script asynchronously
      ProcessResult result = await Process.run('bash', [scriptPath]);

      if (result.exitCode == 0) {
        print("Reboot script executed successfully");
        return true;
      } else {
        print("Reboot script failed: ${result.stderr}");
        return false;
      }
    } catch (e) {
      print("Reboot error: $e");
      return false;
    }
  }

  // Stub: sync date & time
  Future<bool> syncDateTime() async {
    try {
      final response = await http.post(Uri.parse('$baseUrl/sync_datetime'));
      return response.statusCode == 200;
    } catch (e) {
      print("Sync datetime error: $e");
      return false;
    }
  }

  // Stub: sensor health check
  Future<Map<String, dynamic>> sensorHealthCheck() async {
    try {
      final response = await http.get(Uri.parse('$baseUrl/sensors'));
      if (response.statusCode == 200) {
        return jsonDecode(response.body);
      } else {
        return {'status': 'error'};
      }
    } catch (e) {
      print("Sensor health error: $e");
      return {'status': 'error'};
    }
  }

  // Stub: update username/password
  Future<bool> updateCredentials(String username, String password) async {
    try {
      final response = await http.post(
        Uri.parse('$baseUrl/credentials'),
        headers: {'Content-Type': 'application/json'},
        body: jsonEncode({'username': username, 'password': password}),
      );
      return response.statusCode == 200;
    } catch (e) {
      print("Update credentials error: $e");
      return false;
    }
  }

  // Stub: update firmware/software
  Future<bool> updateFirmware() async {
    try {
      final response = await http.post(Uri.parse('$baseUrl/update'));
      return response.statusCode == 200;
    } catch (e) {
      print("Firmware update error: $e");
      return false;
    }
  }

  Future<String> firmwareVersion() async {
    try {
      final response = await http.get(Uri.parse('$baseUrl/version'));
      if (response.statusCode == 200) {
        final data = jsonDecode(response.body);
        return data['version'] ?? "Unknown";
      } else {
        return "Error: ${response.statusCode}";
      }
    } catch (e) {
      print("Firmware version fetch error: $e");
      return "Error";
    }
  }

  // Stub: reset database
  Future<bool> resetDatabase() async {
    try {
      final response = await http.post(Uri.parse('$baseUrl/reset_db'));
      return response.statusCode == 200;
    } catch (e) {
      print("Database reset error: $e");
      return false;
    }
  }
  
  Future<bool> sensorHealthCheckSensor(String sensor) async {
  try {
    final response = await http.get(Uri.parse('$baseUrl/sensor/$sensor'));
    if (response.statusCode == 200) {
      final data = jsonDecode(response.body);
      return data['status'] == 'ok';
    } else {
      print("Sensor check failed with status: ${response.statusCode}");
      return false;
    }
  } catch (e) {
    print("Sensor check error: $e");
    return false;
  }
}

}
