import 'dart:async';

import 'package:flutter/cupertino.dart';
import 'package:flutter/material.dart';
import 'package:google_fonts/google_fonts.dart';
import 'package:pbl4/models/robot.dart';

class DetailRobot extends StatefulWidget {
  final Robot robot;
  final bool status;
  const DetailRobot({required this.robot, required this.status, super.key});

  @override
  State<DetailRobot> createState() => _DetailRobotState();
}

class _DetailRobotState extends State<DetailRobot> {
  int seconds = 0;
  int minutes = 0;
  late Timer timer;

  @override
  void initState() {
    _startTimer();
    super.initState();
  }

  void _startTimer() {
    timer = Timer.periodic(const Duration(seconds: 1), (timer) {
      setState(() {
        if (seconds < 59) {
          seconds++;
        } else {
          seconds = 0;
          if (minutes < 59) {
            minutes++;
          } else {
            minutes = 0;
          }
        }
      });
    });
  }

  @override
  void dispose() {
    timer.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Container(
      height: 480,
      decoration: const BoxDecoration(
          color: Colors.white,
          borderRadius: BorderRadius.only(
              topRight: Radius.circular(20), topLeft: Radius.circular(20))),
      child: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Text(
              'Clync Trash Model X',
              style: GoogleFonts.montserrat(
                  fontSize: 25, fontWeight: FontWeight.w800),
            ),
            Row(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                TextButton.icon(
                  onPressed: null,
                  icon: const Icon(
                    Icons.sensors,
                    size: 24,
                  ),
                  label: Text(
                    'Connected',
                    style: GoogleFonts.montserrat(
                        fontSize: 16,
                        fontWeight: FontWeight.bold,
                        color: widget.status ? Colors.green : Colors.red),
                  ),
                ),
                TextButton.icon(
                  onPressed: null,
                  icon: const Icon(
                    Icons.battery_full_rounded,
                    size: 24,
                  ),
                  label: Text(
                    '${widget.robot.percentage} %',
                    style: GoogleFonts.montserrat(
                        fontSize: 16,
                        fontWeight: FontWeight.bold,
                        color: Colors.green),
                  ),
                ),
                TextButton.icon(
                  onPressed: null,
                  icon: const Icon(
                    Icons.speed_rounded,
                    size: 24,
                  ),
                  label: Text(
                    '${widget.robot.speed} m/s',
                    style: GoogleFonts.montserrat(
                        fontSize: 16,
                        fontWeight: FontWeight.bold,
                        color: Colors.black),
                  ),
                ),
              ],
            ),
            ListView(
              shrinkWrap: true,
              children: [
                ListTile(
                  leading: const Icon(
                    Icons.my_location_rounded,
                    color: Colors.red,
                  ),
                  minLeadingWidth: 5,
                  title: Text(
                    'Current Position',
                    style: GoogleFonts.montserrat(
                        fontSize: 16, fontWeight: FontWeight.bold),
                  ),
                  subtitle: Text(
                    '${widget.robot.startPos.coor.latitude} - ${widget.robot.startPos.coor.longitude}',
                    style: GoogleFonts.montserrat(
                        fontSize: 13, fontWeight: FontWeight.w500),
                  ),
                ),
                const Divider(
                  indent: 20,
                  endIndent: 20,
                  color: Colors.black,
                  thickness: 1.5,
                ),
                ListTile(
                  leading: Icon(
                    Icons.location_on_rounded,
                    size: 24,
                    color: Colors.green.shade800,
                  ),
                  minLeadingWidth: 5,
                  title: Text(
                    'Start Position',
                    style: GoogleFonts.montserrat(
                        fontSize: 16, fontWeight: FontWeight.bold),
                  ),
                  subtitle: Text(
                    '${widget.robot.startPos.coor.latitude} - ${widget.robot.startPos.coor.longitude}',
                    style: GoogleFonts.montserrat(
                        fontSize: 13, fontWeight: FontWeight.w500),
                  ),
                ),
                ListTile(
                  leading: const Icon(
                    Icons.push_pin_rounded,
                    size: 24,
                  ),
                  minLeadingWidth: 5,
                  title: Text(
                    'End Position',
                    style: GoogleFonts.montserrat(
                        fontSize: 16, fontWeight: FontWeight.bold),
                  ),
                  subtitle: Text(
                    '0 - 0',
                    style: GoogleFonts.montserrat(
                        fontSize: 13, fontWeight: FontWeight.w500),
                  ),
                ),
                ListTile(
                  leading: const Icon(
                    Icons.timer_rounded,
                    size: 24,
                  ),
                  minLeadingWidth: 5,
                  title: Text(
                    'Active Time ${minutes.toString().padLeft(2, '0')}:${seconds.toString().padLeft(2, '0')}',
                    style: GoogleFonts.montserrat(
                        fontSize: 16, fontWeight: FontWeight.bold),
                  ),
                  subtitle: Text(
                    'Time Start Leave Home To Finish Path Route',
                    style: GoogleFonts.montserrat(
                        fontSize: 13, fontWeight: FontWeight.w500),
                  ),
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }
}
