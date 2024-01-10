import 'package:flutter/material.dart';
import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:pbl4/domain/usecases/geometry_usecases.dart';
import 'package:pbl4/presentations/bloc/geometry/geometry_bloc.dart';
import 'package:pbl4/presentations/page/dashboard_page.dart';

void main() {
  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  // This widget is the root of your application.
  @override
  Widget build(BuildContext context) {
    return MultiBlocProvider(
      providers: [
        BlocProvider(
          create: (context) => GeometryBloc(useCase: GeometryUseCase()),
        )
      ],
      child: MaterialApp(
          debugShowCheckedModeBanner: false,
          title: 'Map Zone for Cyclone',
          theme: ThemeData(
            colorScheme: ColorScheme.fromSeed(seedColor: Colors.deepPurple),
            useMaterial3: true,
          ),
          home: const DashboardPage()),
    );
  } 
}
