import '/backend/schema/structs/index.dart';
import '/flutter_flow/flutter_flow_icon_button.dart';
import '/flutter_flow/flutter_flow_theme.dart';
import '/flutter_flow/flutter_flow_util.dart';
import '/flutter_flow/flutter_flow_widgets.dart';
import '/flutter_flow/instant_timer.dart';
import '/custom_code/actions/index.dart' as actions;
import 'power_supply_widget.dart' show PowerSupplyWidget;
import 'package:flutter/material.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter_spinkit/flutter_spinkit.dart';
import 'package:google_fonts/google_fonts.dart';
import 'package:provider/provider.dart';

class PowerSupplyModel extends FlutterFlowModel<PowerSupplyWidget> {
  ///  Local state fields for this page.

  int? currentRssi;

  String? receivedValue;

  bool colorChange = false;

  bool colorChang = false;

  int tap = 1;

  int tapp = 1;

  ///  State fields for stateful widgets in this page.

  InstantTimer? rssiUpdateTimer;
  // Stores action output result for [Custom Action - getRssi] action in PowerSupply widget.
  int? updatedRssi;
  // Stores action output result for [Custom Action - moduloChecker] action in Container widget.
  bool? hehe;
  // Stores action output result for [Custom Action - moduloChecker] action in Container widget.
  bool? hehehe;

  @override
  void initState(BuildContext context) {}

  @override
  void dispose() {
    rssiUpdateTimer?.cancel();
  }
}
