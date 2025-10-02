import 'package:flutter/material.dart';
import '../services/setting_service.dart';
import '../services/onfoucs_keyboard.dart';

class SettingsPage extends StatefulWidget {
  const SettingsPage({super.key});

  @override
  State<SettingsPage> createState() => _SettingsPageState();
}

class _SettingsPageState extends State<SettingsPage> {
  final SettingsService _settingService = SettingsService();

  final TextEditingController _usernameController = TextEditingController();
  final TextEditingController _passwordController = TextEditingController();

  bool _vpnEnabled = true; // VPN ON by default
  // Map<String, dynamic> _sensorStatus = {};

  // WiFi state
  List<String> _wifiNetworks = ["Loading..."];
  String firmwareVersion = "Loading...";
  String? _connectedNetwork;
  bool _isUpdating = false; 

  @override
  void initState() {
    super.initState();
    _loadWifiNetworks();
    _fetchVersion();
  }

  Future<void> _fetchVersion() async {
    try {
      final version = await _settingService.firmwareVersion();
      setState(() {
        firmwareVersion = version;
      });
    } catch (e) {
      setState(() {
        firmwareVersion = "Error";
      });
    }
  }

  Future<void> _updateFirmware() async {
    setState(() {
      _isUpdating = true;
    });

    bool success = await _settingService.updateFirmware();

    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(content: Text(success ? "Update Started" : "Failed")),
    );

    await _fetchVersion();

    setState(() {
      _isUpdating = false;
    });
  }

  Future<void> _loadWifiNetworks() async {
    try {
      // Fetch available networks & current connection from API
      List<String> wifiNetworks = await _settingService.fetchWifiNetworks();
      String connectedNetwork = await _settingService.fetchConnectedNetwork();

      setState(() {
        _wifiNetworks = wifiNetworks.isNotEmpty
            ? wifiNetworks
            : ["HomeWiFi", "OfficeWiFi", "GuestWiFi", "CafeWiFi", "PublicWiFi"]; // fallback
        _connectedNetwork = connectedNetwork.isNotEmpty
            ? connectedNetwork
            : "OfficeWiFi"; // fallback
      });
    } catch (e) {
      print("Error loading WiFi networks: $e");
      setState(() {
        // fallback if API call fails
        _wifiNetworks = ["HomeWiFi", "OfficeWiFi", "GuestWiFi", "CafeWiFi", "PublicWiFi"];
        _connectedNetwork = "OfficeWiFi";
      });
    }
  }

  void _showPasswordDialog(String network) {
    final TextEditingController _passwordController = TextEditingController();
    bool _isLoading = false;

    showDialog(
      context: context,
      barrierDismissible: false,
      builder: (ctx) => StatefulBuilder(
        builder: (ctx, setStateDialog) => AlertDialog(
          title: Text("Connect to $network"),
          content: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              AutoFocusKeyboard(
                controller: _passwordController,
                hintText: "Password",
                // autofocus: true,
                // decoration: const InputDecoration(labelText: "Password"),
                obscureText: true,
              ),
              const SizedBox(height: 20),
              if (_isLoading) const CircularProgressIndicator(),
            ],
          ),
          actions: [
            TextButton(
              onPressed: _isLoading ? null : () => Navigator.of(ctx).pop(),
              child: const Text("Cancel"),
            ),
            ElevatedButton(
              onPressed: _isLoading
                  ? null
                  : () async {
                      setStateDialog(() => _isLoading = true);
                      bool success =
                          await _settingService.updateWifi(network, _passwordController.text);
                      setStateDialog(() => _isLoading = false);

                      ScaffoldMessenger.of(context).showSnackBar(
                        SnackBar(content: Text(success ? "Connected Successfully" : "Connection Failed")),
                      );

                      await Future.delayed(const Duration(seconds: 1));
                      if (mounted) Navigator.of(ctx).pop();

                      if (success) {
                        setState(() => _connectedNetwork = network);
                      }
                    },
              child: const Text("Connect"),
            ),
          ],
        ),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text("Settings")),
      body: SingleChildScrollView(
        padding: const EdgeInsets.all(16),
        child: Column(
          children: [
            Card(
              shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
              child: ExpansionTile(
                title: const Text("WiFi Networking",
                    style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold)),
                children: [
                  Padding(
                    padding: const EdgeInsets.all(16),
                    child: SizedBox(
                      height: 180, // Scrollable list height
                      child: ListView.builder(
                        itemCount: _wifiNetworks.length,
                        itemBuilder: (ctx, index) {
                          final network = _wifiNetworks[index];
                          final isConnected = network == _connectedNetwork;
                          return ListTile(
                            title: Text(
                              network,
                              style: TextStyle(
                                color: isConnected ? Colors.green.shade900 : Colors.black,
                                fontWeight: isConnected ? FontWeight.bold : FontWeight.normal,
                              ),
                            ),
                            trailing: isConnected ? const Icon(Icons.check, color: Colors.green) : null,
                            onTap: isConnected ? null : () => _showPasswordDialog(network),
                          );
                        },
                      ),
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
                            bool success = await _settingService.toggleVpn(val);
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
                title: const Text("Reboot Robot",
                    style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold)),
                children: [
                  Padding(
                    padding: const EdgeInsets.all(8),
                    child: SizedBox(
                      width: 180,
                      child: ElevatedButton(
                        style: ElevatedButton.styleFrom(
                          backgroundColor: Colors.red[400],
                          shape: RoundedRectangleBorder(
                            borderRadius: BorderRadius.circular(8), // optional rounded corners
                          ),
                        ),
                        onPressed: () async {
                          bool success = await _settingService.rebootRobot();
                          ScaffoldMessenger.of(context).showSnackBar(
                            SnackBar(content: Text(success ? "Robot will Reboot in 5 seconds ..." : "Failed")),
                          );
                        },
                        child: const Text("Reboot", textAlign: TextAlign.center, style: TextStyle(fontSize: 15, fontWeight: FontWeight.bold, color: Colors.white)),
                      ),
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
                    padding: const EdgeInsets.all(8),
                    child: ElevatedButton(
                      style: ElevatedButton.styleFrom(
                        backgroundColor: Colors.red[400],
                        shape: RoundedRectangleBorder(
                          borderRadius: BorderRadius.circular(8),
                        ),
                      ),
                      onPressed: () async {
                        // Initial dialog state
                        Map<String, String> sensorStatus = {
                          "LiDAR": "checking",
                          "IMU": "checking",
                          "Motor": "checking",
                        };

                        showDialog(
                          context: context,
                          barrierDismissible: false,
                          builder: (ctx) {
                            bool isCheckingStarted = false;

                            return StatefulBuilder(builder: (ctx, setStateDialog) {
                              // Function to check sensors one by one
                              Future<void> checkSensors() async {
                                if (isCheckingStarted) return; // prevent multiple calls
                                isCheckingStarted = true;

                                for (var sensor in sensorStatus.keys) {
                                  if (!mounted) break; // stop if dialog is closed
                                  setStateDialog(() => sensorStatus[sensor] = "loading");

                                  // Simulate API call
                                  bool success = await _settingService.sensorHealthCheckSensor(sensor);

                                  if (!mounted) break;
                                  setStateDialog(() => sensorStatus[sensor] = success ? "ok" : "fail");

                                  // Wait 5 seconds before next sensor
                                  await Future.delayed(const Duration(milliseconds: 500));
                                }
                              }

                              // Start checking after first frame
                              WidgetsBinding.instance.addPostFrameCallback((_) {
                                if (!isCheckingStarted) checkSensors();
                              });

                              return AlertDialog(
                                title: const Text("Sensor Health Check"),
                                content: Column(
                                  mainAxisSize: MainAxisSize.min,
                                  children: sensorStatus.entries.map((entry) {
                                    Widget statusWidget;
                                    Color textColor;

                                    switch (entry.value) {
                                      case "checking":
                                        statusWidget = const Icon(Icons.hourglass_empty, color: Colors.yellow);
                                        textColor = Colors.yellow.shade800;
                                        break;
                                      case "loading":
                                        statusWidget = const SizedBox(
                                          width: 20,
                                          height: 20,
                                          child: CircularProgressIndicator(strokeWidth: 2),
                                        );
                                        textColor = Colors.yellow.shade800;
                                        break;
                                      case "ok":
                                        statusWidget = const Icon(Icons.check_circle, color: Colors.green);
                                        textColor = Colors.green;
                                        break;
                                      case "fail":
                                        statusWidget = const Icon(Icons.cancel, color: Colors.red);
                                        textColor = Colors.red;
                                        break;
                                      default:
                                        statusWidget = const Icon(Icons.help);
                                        textColor = Colors.black;
                                    }

                                    return Padding(
                                      padding: const EdgeInsets.symmetric(vertical: 8),
                                      child: Row(
                                        mainAxisAlignment: MainAxisAlignment.spaceBetween,
                                        children: [
                                          Text(entry.key,
                                              style: TextStyle(
                                                  fontWeight: FontWeight.bold, color: textColor)),
                                          statusWidget,
                                        ],
                                      ),
                                    );
                                  }).toList(),
                                ),
                                actions: [
                                  TextButton(
                                    onPressed: () {
                                      Navigator.of(ctx).pop();
                                    },
                                    child: const Text("Close"),
                                  ),
                                ],
                              );
                            });
                          },
                        );
                      },
                      child: const Text(
                        "Check Sensors",
                        textAlign: TextAlign.center,
                        style: TextStyle(fontSize: 15, fontWeight: FontWeight.bold, color: Colors.white),
                      ),
                    ),
                  ),
                ],
              ),
            ),

            // Date & Time
            Card(
              shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
              child: ExpansionTile(
                title: const Text(
                  "Date & Time",
                  style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                ),
                children: [
                  Padding(
                    padding: const EdgeInsets.all(8),
                    child: Center(
                      child: SizedBox(
                        width: 180, // 👈 consistent width
                        child: ElevatedButton(
                          style: ElevatedButton.styleFrom(
                            backgroundColor: Colors.blue, // 👈 you can change color if needed
                            shape: RoundedRectangleBorder(
                              borderRadius: BorderRadius.circular(8),
                            ),
                          ),
                          onPressed: () async {
                            bool success = await _settingService.syncDateTime();
                            ScaffoldMessenger.of(context).showSnackBar(
                              SnackBar(
                                content: Text(success ? "Date & Time Synced" : "Failed"),
                              ),
                            );
                          },
                          child: const Text(
                            "Sync Date & Time",
                            textAlign: TextAlign.center,
                            style: TextStyle(
                              fontSize: 15,
                              fontWeight: FontWeight.bold,
                              color: Colors.white,
                            ),
                          ),
                        ),
                      ),
                    ),
                  ),
                ],
              ),
            ),

            // Credentials
            // Card(
            //   shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
            //   child: ExpansionTile(
            //     title: const Text("Credentials",
            //         style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold)),
            //     children: [
            //       Padding(
            //         padding: const EdgeInsets.all(16),
            //         child: Column(
            //           children: [
            //             AutoFocusKeyboard(controller: _usernameController, hintText: "Username"),
            //             AutoFocusKeyboard(controller: _passwordController, hintText: "Password", obscureText: true),
            //             const SizedBox(height: 10),
            //             ElevatedButton(
            //               onPressed: () async {
            //                 bool success = await _settingService.updateCredentials(
            //                     _usernameController.text, _passwordController.text);
            //                 ScaffoldMessenger.of(context).showSnackBar(
            //                     SnackBar(content: Text(success ? "Credentials Updated" : "Failed")));
            //               },
            //               child: const Text("Update Credentials"),
            //             ),
            //           ],
            //         ),
            //       ),
            //     ],
            //   ),
            // ),

            // Firmware Update
            Card(
              shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
              child: ExpansionTile(
                title: const Text(
                  "Firmware Update",
                  style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                ),
                children: [
                  Padding(
                    padding: const EdgeInsets.all(8),
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.center, // 👈 center children
                      children: [
                        Text(
                          "Current Version: $firmwareVersion",
                          style: const TextStyle(fontSize: 16),
                        ),
                        const SizedBox(height: 12),
                        Center( // 👈 ensures button is centered
                          child: SizedBox(
                            width: 180, // optional: fixed width for nicer look
                            child: ElevatedButton(
                              style: ElevatedButton.styleFrom(
                                backgroundColor: _isUpdating ? Colors.deepPurple[700] : Colors.deepPurple[400],
                                shape: RoundedRectangleBorder(
                                  borderRadius: BorderRadius.circular(8),
                                ),
                              ),
                              onPressed: _isUpdating
                                  ? null
                                  : () async {
                                      setState(() => _isUpdating = true);

                                      ScaffoldMessenger.of(context).showSnackBar(
                                        SnackBar(
                                          content: Text(
                                            "Update Started"
                                          ),
                                        ),
                                      );

                                      bool success = await _settingService.updateFirmware();

                                      ScaffoldMessenger.of(context).showSnackBar(
                                        SnackBar(
                                          content: Text(
                                            success ? "Update Successful, Please REBOOT!" : "Update Failed",
                                          ),
                                        ),
                                      );

                                      await _fetchVersion(); 
                                      setState(() => _isUpdating = false);
                                    },
                              child: _isUpdating
                                  ? const SizedBox(
                                      width: 20,
                                      height: 20,
                                      child: CircularProgressIndicator(
                                        color: Colors.deepPurple,
                                        strokeWidth: 2,
                                      ),
                                    )
                                  : const Text(
                                      "Update Firmware",
                                      textAlign: TextAlign.center,
                                      style: TextStyle(
                                        fontSize: 15,
                                        fontWeight: FontWeight.bold,
                                        color: Colors.white,
                                      ),
                                    ),
                            ),
                          ),
                        ),
                      ],
                    ),
                  ),
                ],
              ),
            ),

            // Reset Database
            Card(
              shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
              child: ExpansionTile(
                title: const Text(
                  "Reset Database",
                  style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                ),
                children: [
                  Padding(
                    padding: const EdgeInsets.all(8),
                    child: Center(
                      child: SizedBox(
                        width: 180, // 👈 keep width consistent
                        child: ElevatedButton(
                          style: ElevatedButton.styleFrom(
                            backgroundColor: Colors.orange,
                            shape: RoundedRectangleBorder(
                              borderRadius: BorderRadius.circular(8),
                            ),
                          ),
                          onPressed: () async {
                            bool success = await _settingService.resetDatabase();
                            ScaffoldMessenger.of(context).showSnackBar(
                              SnackBar(
                                content: Text(success ? "Database Reset" : "Failed"),
                              ),
                            );
                          },
                          child: const Text(
                            "Reset Database",
                            textAlign: TextAlign.center,
                            style: TextStyle(
                              fontSize: 15,
                              fontWeight: FontWeight.bold,
                              color: Colors.white,
                            ),
                          ),
                        ),
                      ),
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
