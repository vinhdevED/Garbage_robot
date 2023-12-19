import 'dart:async';

import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:flutter_map/plugin_api.dart';
import 'package:pbl4/domain/usecases/geometry_usecases.dart';

import 'package:pbl4/presentations/bloc/geometry/geometry_event.dart';
import 'package:pbl4/presentations/bloc/geometry/geometry_state.dart';

class GeometryBloc extends Bloc<GeometryEvent, GeometryState> {
  GeometryBloc({required GeometryUseCase useCase})
      : _useCase = useCase,
        super(const GeometryState()) {
    on<GeometryTapAddPosition>(_onTapAddPosition);
    //  on<GeometryShowDistance>(_onShowDistance);
    on<GeometryClearMarkers>(_onClearMarkers);
    on<GeometryChangeMode>(_onChangeMode);
    on<GeometryTransmit>(_onTransmit);
  }

  final GeometryUseCase _useCase;

  Future<void> _onTapAddPosition(
      GeometryTapAddPosition event, Emitter<GeometryState> emit) async {
    List<Marker> resultDistance = [];
    if (event.positions.length >= 2) {
      int i = 0;
      while (i < event.positions.length - 1) {
        await _useCase
            .createDistanceMarker(event.positions[i], event.positions[i + 1])
            .then((value) => resultDistance.add(value));
        i++;
      }
      if (state.status == GeometryStatus.viewing) {}
    }

    emit(state.copyWith(
        markers: _useCase.buildMarkers(event.positions),
        polylines: _useCase.buildPolyline(event.positions),
        distances: resultDistance));
  }

  void _onClearMarkers(
      GeometryClearMarkers event, Emitter<GeometryState> emit) {
    emit(state.copyWith(positions: _useCase.clearAll(event.positions)));
  }

  Future<void> _onChangeMode(
      GeometryChangeMode event, Emitter<GeometryState> emit) async {
    if (state.status == GeometryStatus.viewing) {
      emit(state.copyWith(status: GeometryStatus.editor));
    } else {
      emit(state.copyWith(status: GeometryStatus.viewing));
    }
  }

  Future<void> _onTransmit(
      GeometryTransmit event, Emitter<GeometryState> emit) async {
    emit(state.copyWith(request: RequestStatus.loading));

    await _useCase.pushDataToServer().then((value) {
      if (value == RequestStatus.success) {
        emit(state.copyWith(request: RequestStatus.success));
      } else {
        //Handle error
        emit(state.copyWith(request: RequestStatus.error));
      }
    });
  }
}
