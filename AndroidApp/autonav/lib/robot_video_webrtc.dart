import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:flutter_webrtc/flutter_webrtc.dart';
import 'package:web_socket_channel/web_socket_channel.dart';
import 'config.dart';

class RobotVideoWidget extends StatefulWidget {
  const RobotVideoWidget({super.key});

  @override
  State<RobotVideoWidget> createState() => RobotVideoWidgetState();
}

class RobotVideoWidgetState extends State<RobotVideoWidget> {
  RTCPeerConnection? pc;
  RTCVideoRenderer renderer = RTCVideoRenderer();
  WebSocketChannel? ws;

  bool connected = false;

  @override
  void initState() {
    super.initState();
    renderer.initialize();
    startConnection();
  }

  Future<void> startConnection() async {
    if (connected) return;

    final config = {
      "iceServers": [
        {"urls": "stun:stun.l.google.com:19302"},
        {"urls": "stun:stun1.l.google.com:19302"},
        {
          "urls": "turn:openrelay.metered.ca:80",
          "username": "openrelayproject",
          "credential": "openrelayproject"
        }
      ]
    };

    pc = await createPeerConnection(config);

    // ---------- RECEIVE VIDEO ----------
    pc!.onTrack = (event) {
      if (event.track.kind == "video") {
        renderer.srcObject = event.streams[0];
        setState(() => connected = true);
      }
    };

    // ---------- SEND ICE TO SERVER ----------
    pc!.onIceCandidate = (candidate) {
      if (candidate == null) return;

      ws?.sink.add(jsonEncode({
        "candidate": candidate.candidate,
        "sdpMid": candidate.sdpMid,
        "sdpMLineIndex": candidate.sdpMLineIndex,
      }));
    };

    connectWebSocket();
  }

  void connectWebSocket() {
    ws = WebSocketChannel.connect(Uri.parse(webRTCUrl));

    ws!.stream.listen((message) async {
      final data = jsonDecode(message);

      // ----------- OFFER FROM SERVER -----------
      if (data["sdp"] != null) {
        await pc!.setRemoteDescription(
          RTCSessionDescription(data["sdp"], data["type"]),
        );

        final answer = await pc!.createAnswer();
        await pc!.setLocalDescription(answer);

        ws!.sink.add(jsonEncode({
          "sdp": answer.sdp,
          "type": answer.type,
        }));
      }

      // ----------- ICE FROM SERVER -----------
      if (data["candidate"] != null) {
        await pc!.addCandidate(
          RTCIceCandidate(
            data["candidate"],
            data["sdpMid"],
            data["sdpMLineIndex"],
          ),
        );
      }
    });
  }

  Future<void> stopConnection() async {
    await pc?.close();
    await ws?.sink.close();

    if (!mounted) return;
    setState(() => connected = false);
  }

  @override
  void dispose() {
    stopConnection();
    renderer.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return AspectRatio(
      aspectRatio: 4 / 3,
      child: Container(
        color: Colors.black,
        child: connected
            ? RTCVideoView(renderer)
            : const Center(
                child: Text(
                  "Waiting for video...",
                  style: TextStyle(color: Colors.white),
                ),
              ),
      ),
    );
  }
}
