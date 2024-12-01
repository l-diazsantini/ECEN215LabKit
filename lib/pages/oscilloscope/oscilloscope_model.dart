import '/backend/schema/structs/index.dart';
import '/flutter_flow/flutter_flow_charts.dart';
import '/flutter_flow/flutter_flow_icon_button.dart';
import '/flutter_flow/flutter_flow_theme.dart';
import '/flutter_flow/flutter_flow_util.dart';
import '/flutter_flow/flutter_flow_widgets.dart';
import '/flutter_flow/instant_timer.dart';
import '/custom_code/actions/index.dart' as actions;
import 'oscilloscope_widget.dart' show OscilloscopeWidget;
import 'package:flutter/material.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter_spinkit/flutter_spinkit.dart';
import 'package:google_fonts/google_fonts.dart';
import 'package:provider/provider.dart';

class OscilloscopeModel extends FlutterFlowModel<OscilloscopeWidget> {
  ///  Local state fields for this page.

  String? dataa;

  List<double> textT = [];
  void addToTextT(double item) => textT.add(item);
  void removeFromTextT(double item) => textT.remove(item);
  void removeAtIndexFromTextT(int index) => textT.removeAt(index);
  void insertAtIndexInTextT(int index, double item) =>
      textT.insert(index, item);
  void updateTextTAtIndex(int index, Function(double) updateFn) =>
      textT[index] = updateFn(textT[index]);

  double? amp;

  double? time;

  bool zoom = false;

  ///  State fields for stateful widgets in this page.

  InstantTimer? instantTimer;
  // Stores action output result for [Custom Action - receiveData] action in Button widget.
  String? dataaa;
  // Stores action output result for [Custom Action - receiveAndPlotData] action in Button widget.
  List<double>? listt;
  // Stores action output result for [Custom Action - getMaxValueAndPosition] action in Button widget.
  List<dynamic>? amplitude;
  // Stores action output result for [Custom Action - receiveData] action in Button widget.
  String? dat;

  @override
  void initState(BuildContext context) {}

  @override
  void dispose() {
    instantTimer?.cancel();
  }
}
