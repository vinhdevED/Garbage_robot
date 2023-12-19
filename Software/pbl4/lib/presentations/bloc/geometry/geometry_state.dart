import 'package:equatable/equatable.dart';
import 'package:flutter_map/plugin_api.dart';
import 'package:latlong2/latlong.dart';

enum GeometryStatus { viewing, editor }

enum RequestStatus { success, error, loading }

extension GeometryStatusX on GeometryStatus {
  bool get isViewing => this == GeometryStatus.viewing;
  bool get isEditor => this == GeometryStatus.editor;
}

extension RequestStatusX on RequestStatus {
  bool get isSuccess => this == RequestStatus.success;
  bool get isError => this == RequestStatus.error;
  bool get isLoading => this == RequestStatus.loading;
}

class GeometryState extends Equatable {
  final List<LatLng> positions;
  final List<Marker> markers;
  final List<Marker> distances;
  final List<Polyline> polylines;
  final String distance;
  final GeometryStatus status;
  final RequestStatus? request;

  const GeometryState(
      {this.request,
      this.status = GeometryStatus.viewing,
      this.positions = const <LatLng>[],
      this.markers = const <Marker>[],
      this.polylines = const <Polyline>[],
      this.distances = const <Marker>[],
      this.distance = ""});

  GeometryState copyWith(
      {List<LatLng>? positions,
      List<Marker>? markers,
      String? distance,
      List<Marker>? distances,
      List<Polyline>? polylines,
      GeometryStatus? status,
      RequestStatus? request}) {
    return GeometryState(
        positions: positions ?? this.positions,
        markers: markers ?? this.markers,
        distance: distance ?? this.distance,
        distances: distances ?? this.distances,
        polylines: polylines ?? this.polylines,
        status: status ?? this.status,
        request: request ?? this.request);
  }

  @override
  List<Object?> get props =>
      [positions, markers, distance, polylines, distances, status, request];
}
