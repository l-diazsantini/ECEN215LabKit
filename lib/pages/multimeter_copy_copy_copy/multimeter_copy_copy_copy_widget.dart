import '/backend/schema/structs/index.dart';
import '/flutter_flow/flutter_flow_icon_button.dart';
import '/flutter_flow/flutter_flow_theme.dart';
import '/flutter_flow/flutter_flow_util.dart';
import '/flutter_flow/flutter_flow_widgets.dart';
import '/flutter_flow/instant_timer.dart';
import '/widgets/display_received_data/display_received_data_widget.dart';
import '/custom_code/actions/index.dart' as actions;
import 'package:flutter/material.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter_spinkit/flutter_spinkit.dart';
import 'package:google_fonts/google_fonts.dart';
import 'package:provider/provider.dart';
import 'multimeter_copy_copy_copy_model.dart';
export 'multimeter_copy_copy_copy_model.dart';

class MultimeterCopyCopyCopyWidget extends StatefulWidget {
  const MultimeterCopyCopyCopyWidget({
    super.key,
    required this.deviceName,
    required this.deviceId,
    required this.deviceRssi,
    required this.hasWriteCharacteristic,
  });

  final String? deviceName;
  final String? deviceId;
  final int? deviceRssi;
  final bool? hasWriteCharacteristic;

  @override
  State<MultimeterCopyCopyCopyWidget> createState() =>
      _MultimeterCopyCopyCopyWidgetState();
}

class _MultimeterCopyCopyCopyWidgetState
    extends State<MultimeterCopyCopyCopyWidget> {
  late MultimeterCopyCopyCopyModel _model;

  final scaffoldKey = GlobalKey<ScaffoldState>();

  @override
  void initState() {
    super.initState();
    _model = createModel(context, () => MultimeterCopyCopyCopyModel());

    // On page load action.
    SchedulerBinding.instance.addPostFrameCallback((_) async {
      _model.currentRssi = widget!.deviceRssi;
      safeSetState(() {});
      _model.rssiUpdateTimer = InstantTimer.periodic(
        duration: Duration(milliseconds: 2000),
        callback: (timer) async {
          _model.updatedRssi = await actions.getRssi(
            BTDeviceStruct(
              name: widget!.deviceName,
              id: widget!.deviceId,
              rssi: widget!.deviceRssi,
            ),
          );
          _model.currentRssi = _model.updatedRssi;
          safeSetState(() {});
        },
        startImmediately: true,
      );
    });
  }

  @override
  void dispose() {
    _model.dispose();

    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      onTap: () => FocusScope.of(context).unfocus(),
      child: Scaffold(
        key: scaffoldKey,
        backgroundColor: FlutterFlowTheme.of(context).primaryBackground,
        appBar: AppBar(
          backgroundColor: FlutterFlowTheme.of(context).primaryBackground,
          automaticallyImplyLeading: false,
          leading: FlutterFlowIconButton(
            borderColor: Colors.transparent,
            borderRadius: 30.0,
            borderWidth: 1.0,
            buttonSize: 60.0,
            icon: Icon(
              Icons.arrow_back_rounded,
              color: FlutterFlowTheme.of(context).primaryText,
              size: 30.0,
            ),
            onPressed: () async {
              context.pop();
              await actions.sendData(
                BTDeviceStruct(
                  name: widget!.deviceName,
                  id: widget!.deviceId,
                  rssi: widget!.deviceRssi,
                ),
                'Stop',
              );
            },
          ),
          title: Text(
            'View your measured resistance below.',
            style: FlutterFlowTheme.of(context).titleMedium.override(
                  fontFamily: 'Montserrat',
                  color: FlutterFlowTheme.of(context).primaryText,
                  fontSize: 15.0,
                  letterSpacing: 0.0,
                  fontWeight: FontWeight.w500,
                ),
          ),
          actions: [],
          centerTitle: true,
          elevation: 2.0,
        ),
        body: SafeArea(
          top: true,
          child: Container(
            width: double.infinity,
            height: double.infinity,
            decoration: BoxDecoration(
              color: FlutterFlowTheme.of(context).primaryBackground,
            ),
            child: Stack(
              children: [
                Align(
                  alignment: AlignmentDirectional(0.0, 0.0),
                  child: Column(
                    mainAxisSize: MainAxisSize.max,
                    children: [
                      Expanded(
                        child: Padding(
                          padding: EdgeInsetsDirectional.fromSTEB(
                              0.0, 250.0, 0.0, 0.0),
                          child: wrapWithModel(
                            model: _model.displayReceivedDataModel,
                            updateCallback: () => safeSetState(() {}),
                            child: DisplayReceivedDataWidget(
                              device: BTDeviceStruct(
                                name: widget!.deviceName,
                                id: widget!.deviceId,
                                rssi: widget!.deviceRssi,
                              ),
                            ),
                          ),
                        ),
                      ),
                    ],
                  ),
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }
}
