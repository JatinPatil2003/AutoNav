import 'package:flutter/material.dart';
import 'dart:math';

class MapView extends StatefulWidget {
  final Map<String, dynamic>? mapData;
  final Map<String, dynamic>? robotPosition;
  final List<Map<String, dynamic>> savedPoints;

  const MapView({
    Key? key,
    required this.mapData,
    required this.robotPosition,
    required this.savedPoints,
  }) : super(key: key);

  @override
  State<MapView> createState() => _MapViewState();
}

class _MapViewState extends State<MapView> {
  double scale = 1.0;
  Offset offset = Offset.zero;
  Offset lastFocalPoint = Offset.zero;
  bool userInteracted = false; // Track if user did manual zoom/pan

  void resetToOrigin(Size screenSize) {
    if (widget.mapData == null) return;

    final info = widget.mapData!['info'];
    final mapWidth = (info['width'] as num).toDouble();
    final mapHeight = (info['height'] as num).toDouble();
    final resolution = info['resolution'] as num;
    final origin = info['origin']['position'];

    // Fit scale to screen (same logic as default auto-scale)
    final scaleX = screenSize.width / mapWidth;
    final scaleY = screenSize.height / mapHeight;
    final newScale = min(scaleX, scaleY);

    // Convert world origin to map pixel coords
    final originX = -(origin['x'] as num) / resolution;
    final originY = mapHeight + (origin['y'] as num) / resolution;

    setState(() {
      scale = newScale;
      offset = Offset(
        screenSize.width / 2 - originX * newScale,
        screenSize.height / 2 - originY * newScale,
      );
      userInteracted = false; // reset state
    });
  }


  @override
  Widget build(BuildContext context) {
    final screenSize = MediaQuery.of(context).size;

    // Default auto-fit scale
    if (!userInteracted && widget.mapData != null) {
      final mapWidth = (widget.mapData!['info']['width'] as num).toDouble();
      final mapHeight = (widget.mapData!['info']['height'] as num).toDouble();

      final scaleX = screenSize.width / mapWidth;
      final scaleY = screenSize.height / mapHeight;
      scale = min(scaleX, scaleY);
    }

    return Scaffold(
      body: GestureDetector(
        onScaleStart: (details) {
          lastFocalPoint = details.focalPoint;
        },
        onScaleUpdate: (details) {
          setState(() {
            userInteracted = true; // user changed zoom/pan

            // Zoom
            scale *= details.scale;
            scale = scale.clamp(3.0, 8.0);

            // Pan
            offset += details.focalPoint - lastFocalPoint;
            lastFocalPoint = details.focalPoint;
          });
        },
        child: Container(
          width: screenSize.width,
          height: screenSize.height,
          color: Colors.grey[400], // extra canvas background
          child: Transform(
            alignment: Alignment.topLeft,
            transform: Matrix4.identity()
              ..translate(offset.dx, offset.dy)
              ..scale(scale, scale),
            child: CustomPaint(
              painter: _MapPainter(
                widget.mapData,
                widget.robotPosition,
                widget.savedPoints,
              ),
              child: SizedBox(
                width: widget.mapData != null
                    ? (widget.mapData!['info']['width'] as num).toDouble()
                    : screenSize.width,
                height: widget.mapData != null
                    ? (widget.mapData!['info']['height'] as num).toDouble()
                    : screenSize.height,
              ),
            ),
          ),
        ),
      ),
      floatingActionButton: FloatingActionButton(
        onPressed: () => resetToOrigin(MediaQuery.of(context).size),
        child: const Icon(Icons.my_location),
      ),
    );
  }
}

class _MapPainter extends CustomPainter {
  final Map<String, dynamic>? mapData;
  final Map<String, dynamic>? robotPosition;
  final List<Map<String, dynamic>> savedPoints;

  _MapPainter(this.mapData, this.robotPosition, this.savedPoints);

  @override
  void paint(Canvas canvas, Size size) {
    final paint = Paint();

    // Fill full canvas background
    canvas.drawRect(Rect.fromLTWH(0, 0, size.width, size.height),
        paint..color = Colors.grey[400]!);

    if (mapData == null) return;

    final info = mapData!['info'];
    final width = (info['width'] as num).toInt();
    final height = (info['height'] as num).toInt();
    final resolution = info['resolution'] as num;
    final origin = info['origin']['position'];
    final data = List<int>.from(mapData!['data']);

    // Draw map cells
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        final idx = y * width + x;
        final value = data[idx];

        if (value > 50) {
          // paint.color = Colors.grey[700]!;
          paint.color = Colors.grey[900]!;
        } else if (value == 0) {
          paint.color = Colors.white;
        } else {
          paint.color = Colors.grey[400]!;
        }

        canvas.drawRect(
          Rect.fromLTWH(x.toDouble(), (height - 1 - y).toDouble(), 1.1, 1.1),
          paint,
        );
      }
    }

    // Draw saved points
    for (final point in savedPoints) {
      final pos = point['position'];
      final pointX = ((pos['x'] as num) - (origin['x'] as num)) / resolution;
      final pointY = height - ((pos['y'] as num) - (origin['y'] as num)) / resolution;

      // Set color based on type
      switch (point['name']) {
        case 'localization':
          paint.color = Colors.yellow;
          break;
        case 'charger':
          paint.color = Colors.green;
          break;
        case 'standby':
          paint.color = Colors.grey;
          break;
        default:
          paint.color = Colors.blue;
      }

      const pointSize = 1.0;

      // Draw circle at position
      canvas.drawCircle(Offset(pointX.toDouble(), pointY.toDouble()), pointSize, paint);

      // Draw the name label
      final textPainter = TextPainter(
        text: TextSpan(
          text: point['name'],
          style: const TextStyle(color: Colors.purple, fontSize: 1.5),
        ),
        textDirection: TextDirection.ltr,
      );
      textPainter.layout();
      textPainter.paint(canvas, Offset(pointX.toDouble()+1, pointY.toDouble()-0.5));
    }

    // Draw robot
    if (robotPosition != null) {
      final robotX =
          ((robotPosition!['x'] as num) - (origin['x'] as num)) / resolution;
      final robotY = height -
          ((robotPosition!['y'] as num) - (origin['y'] as num)) / resolution;
      final theta = -(robotPosition!['theta'] as num); // flip rotation

      paint.color = Colors.red;
      const robotSize = 2.5;

      canvas.save();
      canvas.translate(robotX.toDouble(), robotY.toDouble());
      canvas.rotate(theta.toDouble());
      canvas.drawCircle(Offset.zero, robotSize, paint);

      paint.color = Colors.yellow;
      paint.strokeWidth = 1;
      canvas.drawLine(Offset.zero, Offset(4, 0), paint);
      canvas.restore();
    }
  }

  @override
  bool shouldRepaint(covariant _MapPainter oldDelegate) {
    return oldDelegate.mapData != mapData ||
        oldDelegate.robotPosition != robotPosition ||
        oldDelegate.savedPoints != savedPoints;
  }
}
