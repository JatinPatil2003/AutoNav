import 'package:flutter/material.dart';
import '../services/setting_service.dart';

class SettingsPage extends StatefulWidget {
  const SettingsPage({super.key});

  @override
  State<SettingsPage> createState() => _SettingsPageState();
}

class _SettingsPageState extends State<SettingsPage> {
  final SettingsService _service = SettingsService();

  final TextEditingController _usernameController = TextEditingController();
  final TextEditingController _passwordController = TextEditingController();

  bool _vpnEnabled = true; // VPN ON by default
  Map<String, dynamic> _sensorStatus = {};

  // WiFi state
  List<String> _wifiNetworks = ["Loading..."];
  String? _selectedWifi;

  @override
  void initState() {
    super.initState();
    _loadWifiNetworks();
  }

  Future<void> _loadWifiNetworks() async {
    // Replace with actual API call later
    await Future.delayed(const Duration(seconds: 1));
    setState(() {
      _wifiNetworks = ["HomeWiFi", "OfficeWiFi", "GuestWiFi"];
      _selectedWifi = _wifiNetworks[0];
    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text("Settings")),
      body: SingleChildScrollView(
        padding: const EdgeInsets.all(16),
        child: Column(
          children: [
            // WiFi Networking
            Card(
              shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
              child: ExpansionTile(
                title: const Text("WiFi Networking", style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold)),
                children: [
                  Padding(
                    padding: const EdgeInsets.all(16),
                    child: Column(
                      children: [
                        DropdownButton<String>(
                          isExpanded: true,
                          value: _selectedWifi,
                          items: _wifiNetworks
                              .map((wifi) => DropdownMenuItem(value: wifi, child: Text(wifi)))
                              .toList(),
                          onChanged: (val) {
                            setState(() => _selectedWifi = val);
                          },
                        ),
                        const SizedBox(height: 10),
                        ElevatedButton(
                          onPressed: () async {
                            bool success = await _service.updateWifi(_selectedWifi ?? "", "");
                            ScaffoldMessenger.of(context).showSnackBar(
                                SnackBar(content: Text(success ? "WiFi Updated" : "Failed")));
                          },
                          child: const Text("Connect WiFi"),
                        ),
                      ],
                    ),
                  ),
                ],
              ),
            ),

            // VPN
            Card(
              shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
              child: ExpansionTile(
                title: const Text("VPN", style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold)),
                children: [
                  Padding(
                    padding: const EdgeInsets.all(16),
                    child: Row(
                      mainAxisAlignment: MainAxisAlignment.spaceBetween,
                      children: [
                        const Text("Enable VPN"),
                        Switch(
                          value: _vpnEnabled,
                          onChanged: (val) async {
                            bool success = await _service.toggleVpn(val);
                            if (success) setState(() => _vpnEnabled = val);
                          },
                        ),
                      ],
                    ),
                  ),
                ],
              ),
            ),

            // Reboot
            Card(
              shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
              child: ExpansionTile(
                title:
                    const Text("Reboot Robot", style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold)),
                children: [
                  Padding(
                    padding: const EdgeInsets.all(16),
                    child: ElevatedButton(
                      style: ElevatedButton.styleFrom(backgroundColor: Colors.red),
                      onPressed: () async {
                        bool success = await _service.rebootRobot();
                        ScaffoldMessenger.of(context)
                            .showSnackBar(SnackBar(content: Text(success ? "Robot Rebooted" : "Failed")));
                      },
                      child: const Text("Reboot"),
                    ),
                  ),
                ],
              ),
            ),

            // Sensor Health
            Card(
              shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
              child: ExpansionTile(
                title: const Text("Sensor Health Check",
                    style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold)),
                children: [
                  Padding(
                    padding: const EdgeInsets.all(16),
                    child: ElevatedButton(
                      onPressed: () async {
                        _sensorStatus = await _service.sensorHealthCheck();
                        showDialog(
                          context: context,
                          builder: (_) => AlertDialog(
                            title: const Text("Sensor Health"),
                            content: Text(_sensorStatus.toString()),
                          ),
                        );
                      },
                      child: const Text("Check Sensors"),
                    ),
                  ),
                ],
              ),
            ),

            // Date & Time
            Card(
              shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
              child: ExpansionTile(
                title:
                    const Text("Date & Time", style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold)),
                children: [
                  Padding(
                    padding: const EdgeInsets.all(16),
                    child: ElevatedButton(
                      onPressed: () async {
                        bool success = await _service.syncDateTime();
                        ScaffoldMessenger.of(context)
                            .showSnackBar(SnackBar(content: Text(success ? "Date & Time Synced" : "Failed")));
                      },
                      child: const Text("Sync Date & Time"),
                    ),
                  ),
                ],
              ),
            ),

            // Credentials
            Card(
              shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
              child: ExpansionTile(
                title:
                    const Text("Credentials", style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold)),
                children: [
                  Padding(
                    padding: const EdgeInsets.all(16),
                    child: Column(
                      children: [
                        TextField(controller: _usernameController, decoration: const InputDecoration(labelText: "Username")),
                        TextField(controller: _passwordController, decoration: const InputDecoration(labelText: "Password")),
                        const SizedBox(height: 10),
                        ElevatedButton(
                          onPressed: () async {
                            bool success = await _service.updateCredentials(
                                _usernameController.text, _passwordController.text);
                            ScaffoldMessenger.of(context)
                                .showSnackBar(SnackBar(content: Text(success ? "Credentials Updated" : "Failed")));
                          },
                          child: const Text("Update Credentials"),
                        ),
                      ],
                    ),
                  ),
                ],
              ),
            ),

            // Firmware Update
            Card(
              shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
              child: ExpansionTile(
                title: const Text("Firmware Update",
                    style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold)),
                children: [
                  Padding(
                    padding: const EdgeInsets.all(16),
                    child: ElevatedButton(
                      onPressed: () async {
                        bool success = await _service.updateFirmware();
                        ScaffoldMessenger.of(context)
                            .showSnackBar(SnackBar(content: Text(success ? "Update Started" : "Failed")));
                      },
                      child: const Text("Update Firmware"),
                    ),
                  ),
                ],
              ),
            ),

            // Reset Database
            Card(
              shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
              child: ExpansionTile(
                title:
                    const Text("Reset Database", style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold)),
                children: [
                  Padding(
                    padding: const EdgeInsets.all(16),
                    child: ElevatedButton(
                      style: ElevatedButton.styleFrom(backgroundColor: Colors.orange),
                      onPressed: () async {
                        bool success = await _service.resetDatabase();
                        ScaffoldMessenger.of(context)
                            .showSnackBar(SnackBar(content: Text(success ? "Database Reset" : "Failed")));
                      },
                      child: const Text("Reset Database"),
                    ),
                  ),
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }
}
