import 'package:equatable/equatable.dart';

import 'package:latlong2/latlong.dart';

abstract class GeometryEvent extends Equatable {
  const GeometryEvent();

  @override
  List<Object?> get props => [];
}

class GeometryTapAddPosition extends GeometryEvent {
  const GeometryTapAddPosition(this.positions);
  final List<LatLng> positions;
  @override
  List<Object?> get props => [positions];
}

class GeometryMoveObject extends GeometryEvent {
  const GeometryMoveObject(this.position);
  final LatLng position;
  @override
  List<Object?> get props => [position];
}

//For calculating the distance between two location
class GeometryShowDistance extends GeometryEvent {
  const GeometryShowDistance(this.firstLocation, this.secondLocation);
  final LatLng firstLocation;
  final LatLng secondLocation;

  @override
  List<Object?> get props => [firstLocation, secondLocation];
}

class GeometryChangeMode extends GeometryEvent {
  const GeometryChangeMode();
}

//For clear all data in edit mode
class GeometryClearMarkers extends GeometryEvent {
  const GeometryClearMarkers(this.positions);
  final List<LatLng> positions;
  @override
  List<Object?> get props => [positions];
}

class GeometryTransmit extends GeometryEvent {
  const GeometryTransmit();

  @override
  List<Object?> get props => [];
}
