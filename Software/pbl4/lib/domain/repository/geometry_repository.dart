import 'dart:typed_data';

import 'package:flutter/material.dart';
import 'package:flutter_map/flutter_map.dart';

import 'package:latlong2/latlong.dart';
import 'package:pbl4/presentations/bloc/geometry/geometry_state.dart';

abstract class GeometryRepository {
  List<LatLng> clearAll(List<LatLng> touches);
  List<Marker> buildMarkers(List<LatLng> touches);
  List<Polyline> buildPolyline(List<LatLng> touches);
  Future<Marker> createDistanceMarker(LatLng startLocation, LatLng location);
  Future<LatLng> getCenterLatLng(List<LatLng> touches);
  Future<Uint8List> getUint8List(GlobalKey widgetKey);
  Future<String> calculateDistanceTwoPoint(
      LatLng firstLocation, LatLng secondLocation);
  //Get Data GPS From database
  Future<Map<String, dynamic>> getDataFromDatabase();
  Future<RequestStatus> pushDataToServer();
  List<dynamic> convertLatLong(List<LatLng> list);
}
