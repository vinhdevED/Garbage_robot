import 'package:flutter/material.dart';
import 'package:google_fonts/google_fonts.dart';

class TextButtonFormat extends StatelessWidget {
  final String text;
  final IconData icon;
  final Color color;
  const TextButtonFormat(
      {required this.color, required this.icon, required this.text, super.key});

  @override
  Widget build(BuildContext context) {
    return TextButton.icon(
      onPressed: null,
      icon: Icon(
        icon,
        color: color,
      ),
      style: ButtonStyle(
        backgroundColor: MaterialStateProperty.all(Colors.white),
      ),
      label: Text(
        text,
        style: GoogleFonts.roboto(
          fontWeight: FontWeight.w800,
          color: color,
        ),
      ),
    );
  }
}
