import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:google_fonts/google_fonts.dart';

import 'package:latlong2/latlong.dart';
import 'package:pbl4/presentations/bloc/geometry/geometry_bloc.dart';
import 'package:pbl4/presentations/bloc/geometry/geometry_event.dart';
import 'package:pbl4/presentations/bloc/geometry/geometry_state.dart';
import 'package:pbl4/presentations/widgets/text_button.dart';

class ToolEditorGeometry extends StatefulWidget {
  final List<LatLng> touches;
  final Set<dynamic> gridList;
  const ToolEditorGeometry(
      {required this.touches, required this.gridList, super.key});

  @override
  State<ToolEditorGeometry> createState() => _ToolEditorGeometryState();
}

class _ToolEditorGeometryState extends State<ToolEditorGeometry> {
  @override
  Widget build(BuildContext context) {
    return SafeArea(
      minimum: const EdgeInsets.only(right: 10, left: 10, bottom: 10),
      child: SizedBox(
          height: double.infinity,
          width: double.infinity,
          child: bodyFunction()),
    );
  }

  Widget bodyFunction() {
    return BlocBuilder<GeometryBloc, GeometryState>(builder: (context, state) {
      return Column(
        key: const ValueKey<String>("NoMode"),
        mainAxisAlignment: MainAxisAlignment.spaceAround,
        crossAxisAlignment: CrossAxisAlignment.end,
        children: [
          formatIcon(Icons.near_me_rounded, "Navigation", () {}),
          AnimatedCrossFade(
            crossFadeState: state.status == GeometryStatus.viewing
                ? CrossFadeState.showFirst
                : CrossFadeState.showSecond,
            firstCurve: Curves.easeIn,
            secondCurve: Curves.easeOut,
            reverseDuration: const Duration(milliseconds: 100),
            duration: const Duration(milliseconds: 200),
            firstChild: formatIcon(
                Icons.mode_standby,
                "Mode Standby",
                () => context
                    .read<GeometryBloc>()
                    .add(const GeometryChangeMode())),
            secondChild: Container(
              height: 170,
              padding: const EdgeInsets.symmetric(vertical: 5, horizontal: 10),
              decoration: BoxDecoration(
                  borderRadius: BorderRadius.circular(20),
                  color: Colors.black45),
              child: Column(
                mainAxisAlignment: MainAxisAlignment.spaceAround,
                crossAxisAlignment: CrossAxisAlignment.end,
                children: [
                  formatIcon(
                      Icons.move_up_outlined,
                      "Exit",
                      () => context
                          .read<GeometryBloc>()
                          .add(const GeometryChangeMode()),
                      color: Colors.red),
                  formatIcon(Icons.layers_clear_rounded, "Clear", () {
                    setState(() {});
                    context
                        .read<GeometryBloc>()
                        .add(GeometryClearMarkers(widget.touches));
                  }),
                  formatIcon(Icons.arrow_circle_right_outlined, "Transmit", () {
                    context.read<GeometryBloc>().add(const GeometryTransmit());
                  }, color: Colors.green.shade500)
                ],
              ),
            ),
          ),
          formatIcon(Icons.feed_rounded, "JSON points", () {
            setState(() {
              SnackBar snackbar = SnackBar(
                  content: Text(
                '${widget.gridList}',
                overflow: TextOverflow.clip,
              ));

              Clipboard.setData(ClipboardData(text: '${widget.gridList}')).then(
                  (_) => ScaffoldMessenger.of(context).showSnackBar(snackbar));
            });
          }),
          const Spacer(
            flex: 1,
          ),
          Opacity(
              opacity: state.status == GeometryStatus.viewing ? 0.0 : 1.0,
              child: TextButtonFormat(
                  color: Colors.green.shade500,
                  icon: Icons.explore_rounded,
                  text: 'Advanced Mode - Can choose point make polygon.')),
        ],
      );
    });
  }

  Widget formatIcon(IconData icon, String text, Function()? function,
      {Color color = Colors.black}) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 10),
      child: TextButton.icon(
          onPressed: function,
          icon: Icon(
            icon,
            color: Colors.white,
          ),
          style: ButtonStyle(
            backgroundColor: MaterialStateProperty.all(color),
          ),
          label: Text(
            text,
            style: GoogleFonts.montserrat(
                fontWeight: FontWeight.w800, color: Colors.white),
          )),
    );
  }
}
