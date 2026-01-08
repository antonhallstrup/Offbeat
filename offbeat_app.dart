import 'dart:async';
import 'dart:math';
import 'package:flutter/material.dart';
import 'package:google_maps_flutter/google_maps_flutter.dart';
import 'package:flutter_google_places_sdk/flutter_google_places_sdk.dart' hide LatLng;
import 'package:flutter_blue_plus/flutter_blue_plus.dart';
import 'package:geolocator/geolocator.dart';
import 'package:geomag/geomag.dart';
import 'package:wakelock_plus/wakelock_plus.dart';
import 'package:flutter_compass/flutter_compass.dart';

void main() {
  runApp(const MapApp());
}

class MapApp extends StatelessWidget {
  const MapApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(
      home: MapHome(),
      debugShowCheckedModeBanner: false,
    );
  }
}

class MapHome extends StatefulWidget {
  const MapHome({super.key});

  @override
  State<MapHome> createState() => _MapHomeState();
}

class _MapHomeState extends State<MapHome> {
  bool _usePhoneHeading = true;
  late GoogleMapController mapController;
  final TextEditingController _searchController = TextEditingController();
  LatLng _position = const LatLng(51.4416, 5.4697); // Eindhoven
  static const String apiKey = "AIzaSyCudhvWGb4zv_WWWMyd5cAg7cT8xYD4Ink";

  BluetoothDevice? _nanoDevice;
  BluetoothCharacteristic? _writeChar;

  bool _isConnected = false;
  bool _isCalibratingDirection = false;
  bool _isDirectionCalibrated = false;
  bool _isUpsideDown = false;

  double _phoneHeading = 0.0;
  StreamSubscription<CompassEvent>? _compassSub;

  static const String serviceUuid = "6E400001-B5A3-F393-E0A9-E50E24DCCA9F";
  static const String characteristicUuid = "6E400002-B5A3-F393-E0A9-E50E24DCCA9F";

  final _places = FlutterGooglePlacesSdk(_MapHomeState.apiKey);
  List<AutocompletePrediction> _predictions = [];

  Timer? _bearingTimer;

  @override
  void initState() {
    super.initState();

    WakelockPlus.enable();

    // Listen to phone compass heading
    _compassSub = FlutterCompass.events?.listen((event) {
      final h = event.heading;
      if (h != null) {
        _phoneHeading = h;
      }
    });

    // Request location permissions before starting Bluetooth scan
    Future.delayed(const Duration(milliseconds: 500), () async {
      LocationPermission permission = await Geolocator.checkPermission();
      if (permission == LocationPermission.denied) {
        permission = await Geolocator.requestPermission();
      }
    });

    _startScan();
  }

  Future<void> _startScan() async {
    debugPrint("Initializing Bluetooth...");

    if (FlutterBluePlus.isScanningNow) {
      debugPrint("Scan already running");
      return;
    }

    StreamSubscription<BluetoothAdapterState>? stateSub;
    stateSub = FlutterBluePlus.adapterState.listen((state) async {
      debugPrint("Bluetooth state: $state");
      if (state == BluetoothAdapterState.on) {
        await stateSub?.cancel();

        final targetService = Guid(serviceUuid);

        await FlutterBluePlus.startScan(
          withServices: [targetService],
          timeout: const Duration(seconds: 8),
        );

        late final StreamSubscription<List<ScanResult>> scanSub;
        scanSub = FlutterBluePlus.scanResults.listen((results) async {
          for (final r in results) {
            final adv = r.advertisementData;
            final name = adv.advName.isNotEmpty
                ? adv.advName
                : r.device.platformName;
            final uuids = adv.serviceUuids.map((u) => u.str128).toList();

            debugPrint("Found: $name | services: $uuids");

            final hasTarget = adv.serviceUuids.contains(targetService);
            if (hasTarget) {
              debugPrint("Match by service UUID → connecting to: $name");
              await FlutterBluePlus.stopScan();
              await scanSub.cancel();

              _nanoDevice = r.device;
              await _connectToDevice(r.device);
              return;
            }
          }
        });
      }
    });

    FlutterBluePlus.turnOn();
  }

  Future<void> _connectToDevice(BluetoothDevice device) async {
    await device.connect();
    setState(() => _isConnected = true);

    device.connectionState.listen((state) {
      if (state == BluetoothConnectionState.disconnected) {
        _bearingTimer?.cancel();
        _bearingTimer = null;
        setState(() {
          _isConnected = false;
          _isCalibratingDirection = false;
          _isDirectionCalibrated = false;
          _isUpsideDown = false;
        });
        debugPrint("Disconnected from device");
      }
    });

    debugPrint("Connected to ${device.platformName}");

    List<BluetoothService> services = await device.discoverServices();
    for (var s in services) {
      if (s.uuid.toString().toUpperCase() == serviceUuid) {
        for (var c in s.characteristics) {
          if (c.uuid.toString().toUpperCase() == characteristicUuid) {
            _writeChar = c;
            debugPrint("Write characteristic found!");

            // Send initial heading source state to Nano so it matches the toggle
            try {
              final initialCmd = _usePhoneHeading ? "HEAD_PHONE" : "HEAD_NANO";
              await _writeChar!.write(initialCmd.codeUnits, withoutResponse: true);
              debugPrint("Sent initial heading source command: $initialCmd");
            } catch (e) {
              debugPrint("Error sending initial heading source: $e");
            }

            // Listen for messages from the Nano (e.g. REQ / REQ_ON / REQ_OFF)
            try {
              if (c.properties.notify || c.properties.indicate) {
                await c.setNotifyValue(true);
                c.lastValueStream.listen((value) {
                  final msg = String.fromCharCodes(value);
                  _handleNanoMessage(msg);
                });
              }
            } catch (e) {
              debugPrint("Error enabling notifications: $e");
            }
          }
        }
      }
    }
  }  // <-- this closes _connectToDevice

  Future<void> _sendCurrentBearing() async {
    if (_writeChar == null) {
      debugPrint("No BLE characteristic, skipping send");
      return;
    }

    // Skip if no real destination is set (still at Eindhoven placeholder)
    if (_position == const LatLng(51.4416, 5.4697)) {
      debugPrint("No destination set, skipping bearing send");
      return;
    }

    try {
      final pos = await Geolocator.getCurrentPosition(
        desiredAccuracy: LocationAccuracy.high,
      );
      final current = LatLng(pos.latitude, pos.longitude);

      final bearing = _calculateBearing(current, _position);
      final declination = _getDeclination(current);
      final distance = _calculateDistance(current, _position);

      await _sendBearing(bearing, declination, distance);
    } catch (e) {
      debugPrint("sendCurrentBearing error: $e");
    }
  }


  double _calculateBearing(LatLng from, LatLng to) {
    final double lat1 = from.latitude * (pi / 180);
    final double lon1 = from.longitude * (pi / 180);
    final double lat2 = to.latitude * (pi / 180);
    final double lon2 = to.longitude * (pi / 180);

    final double dLon = lon2 - lon1;
    final double y = sin(dLon) * cos(lat2);
    final double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
    final double bearing = atan2(y, x) * (180 / pi);
    return (bearing + 360) % 360;
  }

  double _calculateDistance(LatLng from, LatLng to) {
    const R = 6371000; // Earth's radius in meters
    final lat1 = from.latitude * pi / 180;
    final lon1 = from.longitude * pi / 180;
    final lat2 = to.latitude * pi / 180;
    final lon2 = to.longitude * pi / 180;

    final dLat = lat2 - lat1;
    final dLon = lon2 - lon1;

    final a = sin(dLat / 2) * sin(dLat / 2) +
        cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);
    final c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return R * c; // distance in meters
  }

  double _getDeclination(LatLng position) {
    final gm = GeoMag();
    final r = gm.calculate(position.latitude, position.longitude, 0, DateTime.now());
    return r.dec;
  }

  Future<void> _sendBearing(double bearing, double declination, double distance) async {
    if (_writeChar == null) return;

    // If phone heading mode is active, send the live compass heading.
    // Otherwise send 0.0 so the Nano knows to rely on its own heading.
    final headingToSend = _usePhoneHeading ? _phoneHeading : 0.0;

    final msg =
        "${bearing.toStringAsFixed(2)},${declination.toStringAsFixed(2)},${distance.toStringAsFixed(1)},${headingToSend.toStringAsFixed(2)}";
    await _writeChar!.write(msg.codeUnits, withoutResponse: true);
    debugPrint(
        "Sent bearing: ${bearing.toStringAsFixed(2)}, declination: ${declination.toStringAsFixed(2)}, "
        "distance: ${distance.toStringAsFixed(1)} m, heading: ${headingToSend.toStringAsFixed(2)}° (usePhoneHeading=$_usePhoneHeading)");
  }

  Future<void> _sendCalibrationCommand(bool start) async {
    if (_writeChar == null) {
      debugPrint("No BLE characteristic, cannot send calibration command");
      return;
    }
    final cmd = start ? "CAL_START" : "CAL_END";
    await _writeChar!.write(cmd.codeUnits, withoutResponse: true);
    debugPrint("Sent calibration command: $cmd");
  }

  Future<void> _sendUpsideDownCommand(bool upsideDown) async {
    if (_writeChar == null) {
      debugPrint("No BLE characteristic, cannot send upside-down command");
      return;
    }
    final cmd = upsideDown ? "UPSIDE_DOWN" : "UPSIDE_NORMAL";
    await _writeChar!.write(cmd.codeUnits, withoutResponse: true);
    debugPrint("Sent upside-down command: $cmd");
  }

  Future<void> _sendResetCommand() async {
    if (_writeChar == null) {
      debugPrint("No BLE characteristic, cannot send RESET command");
      return;
    }
    const cmd = "RESET";
    await _writeChar!.write(cmd.codeUnits, withoutResponse: true);
    debugPrint("Sent command: $cmd");
  }

  void _openSettingsSheet() {
    showModalBottomSheet(
      context: context,
      builder: (ctx) {
        return Padding(
          padding: const EdgeInsets.all(16.0),
          child: Column(
            mainAxisSize: MainAxisSize.min,
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              const Text(
                'Settings',
                style: TextStyle(fontSize: 20, fontWeight: FontWeight.bold),
              ),
              const SizedBox(height: 16),
              Row(
                mainAxisAlignment: MainAxisAlignment.spaceBetween,
                children: [
                  const Expanded(
                    child: Text(
                      'Flip orientation',
                      style: TextStyle(fontSize: 16),
                    ),
                  ),
                  Switch(
                    value: _isUpsideDown,
                    onChanged: (value) async {
                      Navigator.of(ctx).pop();
                      setState(() {
                        _isUpsideDown = value;
                      });
                      await _sendUpsideDownCommand(value);
                    },
                  ),
                ],
              ),
              const SizedBox(height: 16),
              Row(
                mainAxisAlignment: MainAxisAlignment.spaceBetween,
                children: [
                  const Expanded(
                    child: Text(
                      'Use phone heading',
                      style: TextStyle(fontSize: 16),
                    ),
                  ),
                  Switch(
                    value: _usePhoneHeading,
                    onChanged: (value) async {
                      Navigator.of(ctx).pop();
                      setState(() {
                        _usePhoneHeading = value;
                      });
                      final cmd = value ? "HEAD_PHONE" : "HEAD_NANO";
                      await _writeChar?.write(cmd.codeUnits, withoutResponse: true);
                      debugPrint("Sent heading source command: $cmd");
                    },
                  ),
                ],
              ),
              const SizedBox(height: 16),
              SizedBox(
                width: double.infinity,
                child: ElevatedButton.icon(
                  icon: const Icon(Icons.restart_alt),
                  label: const Text('Restart device'),
                  onPressed: () async {
                    await _sendResetCommand();
                    Navigator.of(ctx).pop();
                  },
                ),
              ),
            ],
          ),
        );
      },
    );
  }

  Future<void> _handleNanoMessage(String msg) async {
    msg = msg.trim();
    debugPrint("From Nano: $msg");

    if (msg == "REQ") {
      // Single immediate update requested by Nano
      await _sendCurrentBearing();
    } else if (msg == "REQ_ON") {
      // Start continuous updates while Nano is requesting them
      _startBearingTimer();
    } else if (msg == "REQ_OFF") {
      // Stop continuous updates
      _bearingTimer?.cancel();
      _bearingTimer = null;
      debugPrint("Stopped continuous bearing updates");
    } else {
      debugPrint("Unknown message from Nano: $msg");
    }
  }

  void _startBearingTimer() {
    _bearingTimer?.cancel();
    _bearingTimer = Timer.periodic(const Duration(seconds: 1), (timer) async {
      await _sendCurrentBearing();
    });
    debugPrint("Started continuous bearing updates");
  }

  void _clearDestination() {
    setState(() {
      // TODO: Replace placeholder coordinates with null destination handling
      _position = const LatLng(51.4416, 5.4697);
      _bearingTimer?.cancel();
    });
    debugPrint("Destination cleared");
  }

  @override
  void dispose() {
    _bearingTimer?.cancel();
    _compassSub?.cancel();
    WakelockPlus.disable();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text("Offbeat"),
        actions: [
          IconButton(
            icon: Icon(
              Icons.bluetooth,
              color: _isConnected ? Colors.blue : Colors.grey,
            ),
            onPressed: _startScan,
          ),
          IconButton(
            icon: Icon(
              Icons.explore,
              color: _isConnected
                  ? (_isCalibratingDirection || _isDirectionCalibrated ? Colors.green : Colors.grey)
                  : Colors.grey,
            ),
            tooltip: "Direction calibration",
            onPressed: _isConnected
                ? () async {
                    setState(() => _isCalibratingDirection = true);
                    await _sendCalibrationCommand(true);
                    if (!mounted) return;
                    await Navigator.of(context).push(
                      MaterialPageRoute(
                        builder: (_) => DirectionCalibrationScreen(
                          onClose: () async {
                            await _sendCalibrationCommand(false);
                          },
                        ),
                      ),
                    );
                    if (!mounted) return;
                    setState(() {
                      _isCalibratingDirection = false;
                      _isDirectionCalibrated = true;
                    });
                  }
                : null,
          ),
          IconButton(
            icon: const Icon(Icons.settings),
            tooltip: 'Settings',
            onPressed: _isConnected ? _openSettingsSheet : null,
          ),
          IconButton(
            icon: const Icon(Icons.clear),
            tooltip: "Clear destination",
            onPressed: _clearDestination,
          ),
        ],
      ),
      body: Stack(
        children: [
          GoogleMap(
            cloudMapId: '207a62b9ca9cc9f5f6f79b96',
            onMapCreated: (controller) => mapController = controller,
            initialCameraPosition: CameraPosition(target: _position, zoom: 12),
            myLocationEnabled: true,
            myLocationButtonEnabled: true,
            markers: _position != const LatLng(51.4416, 5.4697)
                ? {
                    Marker(
                      markerId: const MarkerId("selected"),
                      position: _position,
                    ),
                  }
                : {},
            onTap: (LatLng tappedPoint) async {
              FocusManager.instance.primaryFocus?.unfocus();
              setState(() => _position = tappedPoint);

              mapController.animateCamera(
                  CameraUpdate.newLatLngZoom(tappedPoint, 14));

              await _sendCurrentBearing();
            },
          ),
          Positioned(
            top: 16,
            left: 16,
            right: 16,
            child: Column(
              children: [
                Material(
                  elevation: 4,
                  borderRadius: BorderRadius.circular(8),
                  child: TextField(
                    controller: _searchController,
                    decoration: InputDecoration(
                      hintText: "Where do you want to go?",
                      border: const OutlineInputBorder(borderSide: BorderSide.none),
                      filled: true,
                      fillColor: Colors.white,
                      contentPadding: const EdgeInsets.symmetric(horizontal: 16, vertical: 12),
                      suffixIcon: _searchController.text.isNotEmpty
                          ? IconButton(
                              icon: const Icon(Icons.close),
                              onPressed: () {
                                FocusManager.instance.primaryFocus?.unfocus();
                                setState(() {
                                  _searchController.clear();
                                  _predictions = [];
                                });
                              },
                            )
                          : null,
                    ),
                    onChanged: (value) async {
                      if (value.isEmpty) {
                        setState(() => _predictions = []);
                        return;
                      }
                      final result =
                          await _places.findAutocompletePredictions(value);
                      setState(() => _predictions = List.from(result.predictions));
                    },
                  ),
                ),
                if (_predictions.isNotEmpty)
                  Container(
                    margin: const EdgeInsets.only(top: 4),
                    constraints: const BoxConstraints(maxHeight: 200),
                    decoration: BoxDecoration(
                      color: Colors.white,
                      borderRadius: BorderRadius.circular(8),
                      boxShadow: [
                        BoxShadow(
                          color: Colors.black26,
                          blurRadius: 6,
                          offset: Offset(0, 2),
                        ),
                      ],
                    ),
                    child: ListView.builder(
                      shrinkWrap: true,
                      itemCount: _predictions.length,
                      itemBuilder: (context, index) {
                        final p = _predictions[index];
                        return ListTile(
                          title: Text(p.fullText ?? ''),
                          onTap: () async {
                            FocusManager.instance.primaryFocus?.unfocus();
                            await Future.delayed(const Duration(milliseconds: 100));
                            setState(() => _predictions = []);

                            final details = await _places.fetchPlace(
                              p.placeId!,
                              fields: [PlaceField.Location],
                            );

                            final location = details.place?.latLng;
                            if (location != null) {
                              final destination =
                                  LatLng(location.lat, location.lng);

                              setState(() => _position = destination);
                              mapController.animateCamera(
                                  CameraUpdate.newLatLngZoom(destination, 14));

                              await _sendCurrentBearing();
                            }
                          },
                        );
                      },
                    ),
                  ),
              ],
            ),
          ),
        ],
      ),
    );
  }
}

class DirectionCalibrationScreen extends StatelessWidget {
  final Future<void> Function() onClose;
  const DirectionCalibrationScreen({super.key, required this.onClose});

  @override
  Widget build(BuildContext context) {
    return WillPopScope(
      onWillPop: () async {
        // Just allow back navigation without sending calibration-complete
        return true;
      },
      child: Scaffold(
        appBar: AppBar(
          title: const Text('Direction calibration'),
          leading: const SizedBox.shrink(),
          actions: [
            IconButton(
              icon: const Icon(Icons.check),
              onPressed: () async {
                await onClose();
                if (context.mounted) {
                  Navigator.of(context).pop();
                }
              },
            ),
          ],
        ),
        body: Center(
          child: Column(
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              const Padding(
                padding: EdgeInsets.symmetric(horizontal: 24.0),
                child: Text(
                  'Turn slowly to face north with the device attached.',
                  style: TextStyle(fontSize: 18),
                  textAlign: TextAlign.center,
                ),
              ),
              const SizedBox(height: 32),
              SizedBox(
                width: 220,
                height: 220,
                child: StreamBuilder<CompassEvent>(
                  stream: FlutterCompass.events,
                  builder: (context, snapshot) {
                    final heading = snapshot.data?.heading;
                    if (heading == null) {
                      return const Center(child: Text('No compass data'));
                    }
                    return Transform.rotate(
                      angle: (-heading) * (pi / 180),
                      child: const Icon(
                        Icons.navigation,
                        size: 200,
                      ),
                    );
                  },
                ),
              ),
              const SizedBox(height: 16),
              const Text(
                'Arrow points to north',
                style: TextStyle(fontSize: 16),
              ),
            ],
          ),
        ),
      ),
    );
  }
}