import 'dart:async';
import 'dart:math' as math;

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:flutter_map/plugin_api.dart';
import 'package:google_fonts/google_fonts.dart';

import 'package:latlong2/latlong.dart';
import 'package:pbl4/constant/app_constant.dart';

import 'package:pbl4/models/robot.dart';
import 'package:pbl4/presentations/bloc/geometry/geometry_bloc.dart';
import 'package:pbl4/presentations/bloc/geometry/geometry_event.dart';
import 'package:pbl4/presentations/bloc/geometry/geometry_state.dart';
import 'package:pbl4/presentations/widgets/detail.dart';
import 'package:pbl4/presentations/widgets/text_button.dart';
import 'package:pbl4/presentations/widgets/tool_editor.dart';
import 'package:socket_io_client/socket_io_client.dart' as IO;

class DashboardPage extends StatefulWidget {
  const DashboardPage({super.key});

  @override
  State<DashboardPage> createState() => _DashboardPageState();
}

class _DashboardPageState extends State<DashboardPage>
    with SingleTickerProviderStateMixin {
  final List<LatLng> _touches = [];
  MapController mapController = MapController();
  var infoWindowVisible = false;
  //* Initial delay
  bool loading = true;
  bool circleLoaded = false;
  double radius = 1500; //* circle radius
  AnimationController? animationController;

  late Robot robot;
  final latLngList = [
    LatLng(16.062690, 108.205865), // Starting position
    LatLng(16.062690, 108.205865), // Add more coordinates here
    LatLng(16.062994, 108.206117),
    LatLng(16.062902, 108.206401),
    LatLng(16.062909, 108.206645),
    LatLng(16.062459, 108.206620),
    LatLng(16.061660, 108.206234),
  ];
  late IO.Socket socket;

  @override
  void initState() {
    robot = Robot(
      startPos: Coordinate(const LatLng(16.060273379105404, 108.2059118772624)),
      percentage: 0.0,
    );

    animationController = AnimationController(
        vsync: this, duration: const Duration(milliseconds: 800))
      ..repeat();
    _delay();
    connectAndListen();
    super.initState();
  }

  void connectAndListen() {
    // Replace with your server's IP and port
    socket = IO.io('http://10.10.30.140:3000', <String, dynamic>{
      'transports': ['websocket'],
      'autoConnect': false,
    });

    socket.connect();

    // Listen for GPS updates
    socket.on('gps-update', (data) {
      print('New GPS Data: $data');
      // Handle the received GPS data
      setState(() {
        setUpRobot(data['latitude'], data['longitude'],
            data['percentage_battery'], data['speed']);
      });
    });
  }

  void setUpRobot(
      double latitude, double longitude, double percentage, double speed) {
    robot = Robot(
        startPos: Coordinate(LatLng(latitude.toDouble(), longitude.toDouble())),
        percentage: percentage.toDouble(),
        speed: speed.toDouble());
  }

  _delay() {
    Future.delayed(
      const Duration(milliseconds: 500),
      () {
        setState(() => loading = false);
      },
    );
  }

  @override
  void dispose() {
    animationController!.dispose();
    mapController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    // Make status bar of Phone Transparent
    SystemChrome.setSystemUIOverlayStyle(const SystemUiOverlayStyle(
        statusBarColor: Colors.transparent,
        statusBarIconBrightness: Brightness.dark));

    return BlocBuilder<GeometryBloc, GeometryState>(builder: (context, state) {
      return Scaffold(
        appBar: buildAppBar(),
        extendBodyBehindAppBar: true,
        resizeToAvoidBottomInset: false,
        body: !loading
            ? _buildZoneMap()
            : const Center(
                child: CircularProgressIndicator(
                  strokeWidth: 2.0,
                ),
              ),
        floatingActionButtonLocation: FloatingActionButtonLocation.startFloat,
        // floatingActionButton: FloatingActionButton(
        //   onPressed: () {
        //     setState(() {
        //       connectAndListen();
        //     });
        //     log('${convertLatLong(_touches)}');
        //     log('${convertLatLong(createGridPointinMap(_touches, 0.000005))}');
        //   },
        //   backgroundColor: Colors.black,
        //   child: const Icon(
        //     Icons.location_searching_rounded,
        //     color: Colors.white,
        //   ),
        // ),
      );
    });
  }

  PreferredSizeWidget? buildAppBar() {
    return AppBar(
        centerTitle: true,
        forceMaterialTransparency: true,
        title:
            BlocBuilder<GeometryBloc, GeometryState>(builder: (context, state) {
          return AnimatedSwitcher(
            duration: const Duration(milliseconds: 500),
            transitionBuilder: (Widget child, Animation<double> animation) {
              return FadeTransition(opacity: animation, child: child);
            },
            child: switch (state.request) {
              RequestStatus.loading => TextButton.icon(
                  onPressed: null,
                  icon: const CircularProgressIndicator(
                    color: Colors.black,
                  ),
                  style: ButtonStyle(
                    backgroundColor: MaterialStateProperty.all(Colors.white),
                  ),
                  label: Text(
                    'Loading',
                    style: GoogleFonts.montserrat(
                      fontWeight: FontWeight.w800,
                      color: Colors.black,
                    ),
                  ),
                ),
              RequestStatus.success => const TextButtonFormat(
                  color: Colors.green,
                  icon: Icons.arrow_upward_rounded,
                  text: "Success"),
              RequestStatus.error => const TextButtonFormat(
                  color: Colors.red,
                  icon: Icons.error_outline_rounded,
                  text: "Error"),
              _ => const TextButtonFormat(
                  color: Colors.blue,
                  icon: Icons.arrow_upward_rounded,
                  text: "Moving"),
            },
          );
        }));
  }

  Widget _buildZoneMap() {
    return Stack(
      children: [
        BlocBuilder<GeometryBloc, GeometryState>(builder: (context, state) {
          return Positioned.fill(
            child: SizedBox(
              height: MediaQuery.sizeOf(context).height * 0.8,
              width: double.infinity,
              child: Builder(builder: (context) {
                return AnimatedBuilder(
                    animation: animationController!,
                    builder: (context, child) {
                      return FlutterMap(
                        mapController: mapController,
                        options: MapOptions(
                          center: robot.startPos.coor,
                          maxZoom: 18.44,
                          minZoom: 5,
                          zoom: 13,
                          keepAlive: true,
                          pinchMoveThreshold: 10,
                          pinchZoomThreshold: 30,
                          onTap: state.status == GeometryStatus.editor
                              ? (tapPosition, point) {
                                  setState(() {
                                    _touches.add(point);
                                  });
                                  context
                                      .read<GeometryBloc>()
                                      .add(GeometryTapAddPosition(_touches));
                                }
                              : null,
                          onMapReady: () {
                            mapController.mapEventStream.listen((event) {});
                          },
                        ),
                        children: [
                          TileLayer(
                            urlTemplate:
                                "https://api.mapbox.com/styles/v1/treehot68/{mapStyleID}/tiles/256/{z}/{x}/{y}@2x?access_token={accessToken}",
                            additionalOptions: const {
                              'mapStyleID': AppConstant.mapBoxStyleId,
                              'accessToken': AppConstant.mapBoxAccessToken
                            },
                          ),
                          PolygonLayer(
                            polygons: [
                              Polygon(
                                points: _touches,
                                color: Colors.red.withOpacity(0.3),
                                isFilled: true,
                                borderStrokeWidth: 2, // Border width
                                borderColor: Colors.red, // Border color
                              )
                            ],
                          ),
                          PolylineLayer(
                            polylines: state.polylines,
                          ),
                          // PolylineLayer(
                          //   polylines: [
                          //     Polyline(
                          //       points: latLngList,
                          //       gradientColors: [
                          //         Color(0xff3f5efb),
                          //         Color(0xfffc466b)
                          //       ],
                          //       strokeWidth: 4.0,
                          //     )
                          //   ],
                          // ),
                          MarkerLayer(
                            markers: state.markers,
                          ),
                          MarkerLayer(
                            markers: buildMarkerGridPoints(
                                createGridPointinMap(_touches, 0.00003)),
                          ),

                          /*ROBOT POSITION*/
                          CircleLayer(
                            circles: [
                              CircleMarker(
                                  point: robot.startPos.coor,
                                  radius: (radius /
                                          2 *
                                          animationController!.value) /
                                      2,
                                  color: Colors.green.withOpacity(
                                      (animationController!.value - 1.0).abs()),
                                  borderStrokeWidth: 3.0,
                                  borderColor: Colors.green,
                                  useRadiusInMeter: true),
                            ],
                          ),
                          MarkerLayer(markers: [
                            Marker(
                                width: 17.0,
                                height: 17.0,
                                point: robot.startPos.coor,
                                builder: (context) => GestureDetector(
                                    onTap: () {
                                      Scaffold.of(context).showBottomSheet(
                                          (context) => DetailRobot(
                                                robot: robot,
                                                status: socket.connected,
                                              ));
                                    },
                                    child: robot.object))
                          ])

                          /*--------------*/
                        ],
                      );
                    });
              }),
            ),
          );
        }),
        Align(
          alignment: Alignment.topRight,
          child: ToolEditorGeometry(
              touches: _touches,
              gridList:
                  convertLatLong(createGridPointinMap(_touches, 0.00003))),
        ),
      ],
    );
  }

  Set<dynamic> convertLatLong(List<LatLng> list) {
    return list.map((latlng) => {latlng.latitude, latlng.longitude}).toSet();
  }

  //Testing function create grid point in map [WARNING - TESTING]
  List<LatLng> createGridPointinMap(List<LatLng> points, double spacing) {
    List<LatLng> gridPoints = [];
    if (points.length >= 3) {
      // 1. Find bounding box
      double minX = points[0].longitude;
      double maxX = points[0].longitude;
      double minY = points[0].latitude;
      double maxY = points[0].latitude;

      for (LatLng point in points) {
        minX = math.min(minX, point.longitude);
        maxX = math.max(maxX, point.longitude);
        minY = math.min(minY, point.latitude);
        maxY = math.max(maxY, point.latitude);
      }

      // Generate grid points within the bounding box

      for (double x = minX; x <= maxX; x += spacing) {
        for (double y = minY; y <= maxY; y += spacing) {
          LatLng point = LatLng(y, x);
          if (isPointInPolygon(point, points)) {
            gridPoints.add(point);
          }
        }
      }
    }

    return gridPoints;
  }

  bool isPointInPolygon(LatLng point, List<LatLng> polygon) {
    bool isInside = false;
    for (int i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
      if ((polygon[i].longitude > point.longitude) !=
              (polygon[j].longitude > point.longitude) &&
          (point.latitude <
              (polygon[j].latitude - polygon[i].latitude) *
                      (point.longitude - polygon[i].longitude) /
                      (polygon[j].longitude - polygon[i].longitude) +
                  polygon[i].latitude)) {
        isInside = !isInside;
      }
    }
    return isInside;
  }

  List<Marker> buildMarkerGridPoints(List<LatLng> gridPoints) {
    List<Marker> markers = [];
    for (var i = 0; i < gridPoints.length; i++) {
      final pos = gridPoints[i];
      final markerKey = ValueKey<String>("Markers+$i");
      markers.add(Marker(
        point: pos,
        height: 3,
        width: 3,
        builder: (_) => RepaintBoundary(
          key: markerKey,
          child: Container(
            width: 3,
            height: 3,
            decoration: const BoxDecoration(
                color: Colors.white, shape: BoxShape.circle),
          ),
        ),
      ));
    }

    return markers;
  }
}



  // void setupRobotAnimation() {
  //   int currentPosition = 0;
  //   Timer.periodic(const Duration(seconds: 1), (timer) {
  //     if (currentPosition < latLngList.length) {
  //       final newPosition = latLngList[currentPosition];
  //       setState(() {
  //         robot.startPos.coor =
  //             LatLng(newPosition.latitude, newPosition.longitude);
  //       });
  //       currentPosition++;
  //       mapController.move(newPosition, 20);
  //       setState(() {});
  //     } else {
  //       timer.cancel(); // Stop the animation when the path is complete
  //     }
  //   });
  // }