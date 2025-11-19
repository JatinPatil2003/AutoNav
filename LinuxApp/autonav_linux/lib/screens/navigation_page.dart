import 'package:flutter/material.dart';
import '../services/api_service.dart';
import './animation.dart';

class NavigationPage extends StatefulWidget {
  const NavigationPage({super.key});

  @override
  State<NavigationPage> createState() => _NavigationPageState();
}

class _NavigationPageState extends State<NavigationPage> {
  bool isLoading = false;
  bool isEmergency = false;

  // Mode toggle: true = Position, false = Cruise
  bool isPositionMode = true;

  // Selected points
  String? selectedPoint;
  String? selectedGoalForCruise;
  String? selectedCruisePoint;

  // Lists of points
  final List<String> goalPoints = ['Goal 1', 'Goal 2', 'Goal 3'];
  final List<String> standbyPoints = ['Standby 1', 'Standby 2'];
  final List<String> chargingPoints = ['Charging 1', 'Charging 2'];

  final List<String> cruisePoints = [];

  // Slider value
  double holdTime = 2.0;

  // API Service
  final ApiService apiService = ApiService();

  void startNavigation() async {
  if (selectedPoint == null) return;
  setState(() => isLoading = true);

  try {
    // await apiService.startNavigation(selectedPoint!);
    if (mounted) {
      Navigator.push(
        context,
        MaterialPageRoute(
          builder: (context) => AnimationPage(targetPoint: selectedPoint!),
        ),
      );
    }
  } catch (e) {
    if (mounted) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text('Failed to start navigation'),
          backgroundColor: Colors.red.shade400,
        ),
      );
    }
  } finally {
    setState(() => isLoading = false);
  }
}


  void addGoalToCruise() {
    if (selectedGoalForCruise != null && !cruisePoints.contains(selectedGoalForCruise)) {
      setState(() {
        cruisePoints.add(selectedGoalForCruise!);
      });
    }
  }

  void removeFromCruise() {
    if (selectedCruisePoint != null) {
      setState(() {
        cruisePoints.remove(selectedCruisePoint);
      });
    }
  }

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      // onDoubleTap: () {
      //   Navigator.pop(context); // double tap to go back
      // },
      child: Scaffold(
        appBar: AppBar(
          title: const Text('Navigation'),
          leading: IconButton(
            icon: const Icon(Icons.arrow_back),
            onPressed: () => Navigator.pop(context),
          ),
          actions: [
            IconButton(
              icon: Icon(Icons.warning, color: isEmergency ? Colors.red : Colors.white),
              onPressed: () {
                setState(() => isEmergency = !isEmergency);
                // Trigger emergency API if needed
              },
            )
          ],
        ),
        body: isLoading
            ? const Center(child: CircularProgressIndicator())
            : Column(
                children: [
                  // Mode toggle
                  Padding(
                    padding: const EdgeInsets.all(8.0),
                    child: ToggleButtons(
                      isSelected: [isPositionMode, !isPositionMode],
                      onPressed: (index) {
                        setState(() => isPositionMode = index == 0);
                      },
                      children: const [
                        Padding(
                          padding: EdgeInsets.symmetric(horizontal: 16.0),
                          child: Text('Position'),
                        ),
                        Padding(
                          padding: EdgeInsets.symmetric(horizontal: 16.0),
                          child: Text('Cruise'),
                        ),
                      ],
                    ),
                  ),
                  Expanded(
                    child: isPositionMode ? buildPositionMode() : buildCruiseMode(),
                  ),
                ],
              ),
      ),
    );
  }

  // ----------------- POSITION MODE -----------------
  Widget buildPositionMode() {
    return ListView(
      padding: const EdgeInsets.all(8.0),
      children: [
        const Text('Goal Points:', style: TextStyle(fontWeight: FontWeight.bold)),
        ...goalPoints.map((point) => ListTile(
              title: Text(point),
              selected: selectedPoint == point,
              onTap: () => setState(() => selectedPoint = point),
            )),
        const SizedBox(height: 10),
        const Text('Standby Points:', style: TextStyle(fontWeight: FontWeight.bold)),
        ...standbyPoints.map((point) => ListTile(
              title: Text(point),
              selected: selectedPoint == point,
              onTap: () => setState(() => selectedPoint = point),
            )),
        const SizedBox(height: 10),
        const Text('Charging Points:', style: TextStyle(fontWeight: FontWeight.bold)),
        ...chargingPoints.map((point) => ListTile(
              title: Text(point),
              selected: selectedPoint == point,
              onTap: () => setState(() => selectedPoint = point),
            )),
        if (selectedPoint != null)
          Padding(
            padding: const EdgeInsets.symmetric(vertical: 20.0),
            child: ElevatedButton(
              onPressed: startNavigation,
              child: const Text('Start'),
            ),
          ),
      ],
    );
  }

  // ----------------- CRUISE MODE -----------------
  Widget buildCruiseMode() {
    return Row(
      children: [
        // Left column - Goals
        Expanded(
          child: Column(
            children: [
              const Text('Goals:', style: TextStyle(fontWeight: FontWeight.bold)),
              Expanded(
                child: ListView(
                  children: goalPoints
                      .map(
                        (goal) => ListTile(
                          title: Text(goal),
                          selected: selectedGoalForCruise == goal,
                          onTap: () => setState(() => selectedGoalForCruise = goal),
                        ),
                      )
                      .toList(),
                ),
              ),
            ],
          ),
        ),
        // Middle buttons
        Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            ElevatedButton(
              onPressed: addGoalToCruise,
              child: const Icon(Icons.arrow_forward),
            ),
            const SizedBox(height: 10),
            ElevatedButton(
              onPressed: removeFromCruise,
              child: const Icon(Icons.arrow_back),
            ),
          ],
        ),
        // Right column - Cruise points
        Expanded(
          child: Column(
            children: [
              const Text('Cruise Points:', style: TextStyle(fontWeight: FontWeight.bold)),
              Expanded(
                child: ListView(
                  children: cruisePoints
                      .map(
                        (point) => ListTile(
                          title: Text(point),
                          selected: selectedCruisePoint == point,
                          onTap: () => setState(() => selectedCruisePoint = point),
                        ),
                      )
                      .toList(),
                ),
              ),
              // Slider for hold time
              Padding(
                padding: const EdgeInsets.all(8.0),
                child: Column(
                  children: [
                    Text('Hold Time: ${holdTime.toStringAsFixed(1)}s'),
                    Slider(
                      min: 2,
                      max: 20,
                      divisions: 18,
                      value: holdTime,
                      label: '${holdTime.toStringAsFixed(1)}s',
                      onChanged: (value) => setState(() => holdTime = value),
                    ),
                  ],
                ),
              ),
            ],
          ),
        ),
      ],
    );
  }
}
