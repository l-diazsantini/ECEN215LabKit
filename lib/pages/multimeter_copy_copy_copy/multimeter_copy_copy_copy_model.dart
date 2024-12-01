import '/backend/schema/structs/index.dart';
import '/flutter_flow/flutter_flow_icon_button.dart';
import '/flutter_flow/flutter_flow_theme.dart';
import '/flutter_flow/flutter_flow_util.dart';
import '/flutter_flow/flutter_flow_widgets.dart';
import '/flutter_flow/instant_timer.dart';
import '/widgets/display_received_data/display_received_data_widget.dart';
import '/custom_code/actions/index.dart' as actions;
import 'multimeter_copy_copy_copy_widget.dart'
    show MultimeterCopyCopyCopyWidget;
import 'package:flutter/material.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter_spinkit/flutter_spinkit.dart';
import 'package:google_fonts/google_fonts.dart';
import 'package:provider/provider.dart';

class MultimeterCopyCopyCopyModel
    extends FlutterFlowModel<MultimeterCopyCopyCopyWidget> {
  ///  Local state fields for this page.

  int? currentRssi;

  String? receivedValue;

  ///  State fields for stateful widgets in this page.

  InstantTimer? rssiUpdateTimer;
  // Stores action output result for [Custom Action - getRssi] action in MultimeterCopyCopyCopy widget.
  int? updatedRssi;
  // Model for DisplayReceivedData component.
  late DisplayReceivedDataModel displayReceivedDataModel;

  @override
  void initState(BuildContext context) {
    displayReceivedDataModel =
        createModel(context, () => DisplayReceivedDataModel());
  }

  @override
  void dispose() {
    rssiUpdateTimer?.cancel();
    displayReceivedDataModel.dispose();
  }
}
