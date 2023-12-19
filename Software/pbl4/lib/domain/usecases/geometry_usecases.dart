import 'dart:convert';
import 'dart:developer';
import 'dart:typed_data';
import 'dart:ui';

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';

import 'package:http/http.dart' as http;
import 'dart:math' as math;
import 'package:flutter_map/src/layer/marker_layer.dart' show Marker;

import 'package:flutter_map/src/layer/polyline_layer.dart' show Polyline;
import 'package:latlong2/latlong.dart';
import 'package:pbl4/constant/app_constant.dart';
import 'package:pbl4/domain/repository/geometry_repository.dart';
import 'package:pbl4/presentations/bloc/geometry/geometry_state.dart';

import 'package:pbl4/presentations/widgets/distance.dart';

class GeometryUseCase extends GeometryRepository {
  @override
  List<Marker> buildMarkers(List<LatLng> touches) {
    List<Marker> markers = [];
    for (var i = 0; i < touches.length; i++) {
      final pos = touches[i];
      final markerKey = ValueKey<String>("Markers+$i");
      markers.add(Marker(
        point: pos,
        height: 12,
        width: 12,
        builder: (_) => GestureDetector(
          child: RepaintBoundary(
            key: markerKey,
            child: Container(
              width: 12,
              height: 12,
              decoration: const BoxDecoration(
                  color: Colors.red, shape: BoxShape.circle),
              child: Center(
                child: Text(
                  i.toString(),
                  style: const TextStyle(
                      color: Colors.white,
                      fontWeight: FontWeight.bold,
                      fontSize: 7),
                ),
              ),
            ),
          ),
        ),
      ));
    }

    return markers;
  }

  ///[buildPolyline]  between two points take from _touches list
  @override
  List<Polyline> buildPolyline(List<LatLng> touches) {
    final List<Polyline> polyglines = [];

    for (var i = 0; i < touches.length; i++) {
      polyglines
          .add(Polyline(color: Colors.black, points: touches, strokeWidth: 2));
    }
    return polyglines;
  }

  @override
  Future<String> calculateDistanceTwoPoint(
      LatLng firstLocation, LatLng secondLocation) async {
    const r = 6371; //Radius of Earth - km
    var p = math.pi / 180;
    var a = 0.5 -
        math.cos((secondLocation.latitude - firstLocation.latitude) * p) / 2 +
        math.cos(firstLocation.latitude * p) *
            math.cos(secondLocation.latitude * p) *
            (1 -
                math.cos(
                    (secondLocation.longitude - firstLocation.longitude) * p)) /
            2;
    var distance = 2 * r * math.asin(math.sqrt(a));
    if (distance < 1) {
      return "${(double.parse(distance.toStringAsFixed(3)) * 1000).toString().split(".")[0]} m";
    } else {
      return "${double.parse(distance.toStringAsFixed(2))} km";
    }
  }

  @override

  ///[getCenterLatLng] to get center location between two coordinate
  Future<LatLng> getCenterLatLng(List<LatLng> touches) async {
    double pi = math.pi / 180;
    double xpi = 180 / math.pi;
    double x = 0, y = 0, z = 0;

    if (touches.length == 1) {
      return touches[0];
    }

    for (int i = 0; i < touches.length; i++) {
      double latitude = touches[i].latitude * pi;
      double longitude = touches[i].longitude * pi;
      double c1 = math.cos(latitude);
      x = x + c1 * math.cos(longitude);
      y = y + c1 * math.sin(longitude);
      z = z + math.sin(latitude);
    }

    int total = touches.length;
    x = x / total;
    y = y / total;
    z = z / total;

    double centralLongitude = math.atan2(y, x);
    double centralSquareRoot = math.sqrt(x * x + y * y);
    double centralLatitude = math.atan2(z, centralSquareRoot);

    return LatLng(centralLatitude * xpi, centralLongitude * xpi);
  }

  @override

  ///[getUint8List] Converting Widget to PNG to improve performance
  Future<Uint8List> getUint8List(
      GlobalKey<State<StatefulWidget>> widgetKey) async {
    RenderRepaintBoundary boundary =
        widgetKey.currentContext!.findRenderObject() as RenderRepaintBoundary;
    var image = await boundary.toImage(pixelRatio: 2.0);
    ByteData? byteData = await (image.toByteData(format: ImageByteFormat.png));
    return byteData!.buffer.asUint8List();
  }

  @override
  Future<Marker> createDistanceMarker(
      LatLng startLocation, LatLng location) async {
    LatLng center = await getCenterLatLng([startLocation, location]);
    String dist = await calculateDistanceTwoPoint(startLocation, location);
    final distanceKey = ValueKey<String>("Distance+$center");
    Marker distance = Marker(
      point: center,
      height: 16,
      width: 50,
      rotate: true,
      builder: (context) => DistanceMap(
        distance: dist,
        key: distanceKey,
      ),
    );
    return distance;
  }

  @override
  List<LatLng> clearAll(List<LatLng> touches) {
    touches.clear();
    return touches;
  }

  @override
  Future<Map<String, dynamic>> getDataFromDatabase() async {
    try {
      var response = await http.get(Uri.parse(AppConstant.getDataGPS));
      if (response.statusCode == 200) {
        print(json.decode(response.body));
        return json.decode(response.body);
      } else {
        throw Exception('Failed to load data');
      }
    } catch (e) {
      throw Exception('Failed');
    }
  }

  @override
  Future<RequestStatus> pushDataToServer() async {
    final Map<String, String> headers = {
      'Content-Type': "application/json",
    };
    const url = 'http://192.168.1.6:3000/clynctrash/listpoint/fuckyoudat';
    final List<Map<String, dynamic>> data = [
      {"latitude": 40.7128, "longitude": -74.0060},
      {"latitude": 34.0522, "longitude": -118.2437},
      {"latitude": 51.5074, "longitude": -0.1278},
      {"latitude": 40.7128, "longitude": -74.0060},
      {"latitude": 34.0522, "longitude": -118.2437},
      {"latitude": 51.5074, "longitude": -0.1278},
      {"latitude": 40.7128, "longitude": -74.0060},
      {"latitude": 34.0522, "longitude": -118.2437},
      {"latitude": 51.5074, "longitude": -0.1278},
      {"latitude": 40.7128, "longitude": -74.0060},
      {"latitude": 34.0522, "longitude": -118.2437},
      {"latitude": 51.5074, "longitude": -0.1278},
      {"latitude": 40.7128, "longitude": -74.0060},
      {"latitude": 34.0522, "longitude": -118.2437},
      {"latitude": 51.5074, "longitude": -0.1278},
      {"latitude": 40.7128, "longitude": -74.0060},
      {"latitude": 34.0522, "longitude": -118.2437},
      {"latitude": 51.5074, "longitude": -0.1278},
      {"latitude": 40.7128, "longitude": -74.0060},
      {"latitude": 34.0522, "longitude": -118.2437},
      {"latitude": 51.5074, "longitude": -0.1278}
    ];

    final Map<String, List<dynamic>> data2 = {"points": data};

    try {
      final response = await http.post(Uri.parse(url),
          headers: headers, body: json.encode(data2));

      if (response.statusCode == 200) {
        log('Data sent successfully');
        return RequestStatus.success;
      } else {
        log('Failed to send data. Status code: ${response}');
        return RequestStatus.error;
      }
    } catch (e) {
      return RequestStatus.error;
    }
  }

  @override
  List<dynamic> convertLatLong(List<LatLng> list) {
    return list.map((latlng) => [latlng.latitude, latlng.longitude]).toList();
  }
}
