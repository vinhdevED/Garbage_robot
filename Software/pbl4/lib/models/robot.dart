import 'package:flutter/material.dart';
import 'package:latlong2/latlong.dart';

class Dimension {
  final double height;
  final double width;
  final double length;
  Dimension({this.height = 0, this.length = 0, this.width = 0});

  static Dimension defaultValues() {
    return Dimension(height: 26, width: 50, length: 50);
  }
}

class Coordinate {
  LatLng coor;
  Coordinate(this.coor);
}

class Robot {
  Dimension dimension;
  Coordinate startPos;
  Coordinate? endPos;
  double percentage;
  double? speed;
  Widget object;
  Robot(
      {Dimension? dimension,
      Widget? object,
      double? speed,
      required this.percentage,
      required this.startPos,
      this.endPos})
      : dimension = dimension ?? Dimension.defaultValues(),
        object = object ??
            Container(
                decoration: BoxDecoration(
                    color: Colors.white,
                    shape: BoxShape.circle,
                    border: Border.all(color: Colors.white, width: 1.5)),
                child: const Icon(
                  Icons.circle,
                  size: 14,
                  color: Colors.green,
                )),
        speed = speed ?? 0.0;

  Robot copyWith(
      {Dimension? dimension,
      Coordinate? startPos,
      Coordinate? endPos,
      double? percentage,
      double? speed,
      Widget? object}) {
    return Robot(
        startPos: startPos ?? this.startPos,
        endPos: endPos ?? this.endPos,
        percentage: percentage ?? this.percentage,
        speed: speed ?? this.speed);
  }
}
