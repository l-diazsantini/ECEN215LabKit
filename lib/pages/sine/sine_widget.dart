import '/backend/schema/structs/index.dart';
import '/flutter_flow/flutter_flow_icon_button.dart';
import '/flutter_flow/flutter_flow_theme.dart';
import '/flutter_flow/flutter_flow_util.dart';
import '/flutter_flow/flutter_flow_widgets.dart';
import '/flutter_flow/instant_timer.dart';
import '/custom_code/actions/index.dart' as actions;
import 'package:flutter/material.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter_spinkit/flutter_spinkit.dart';
import 'package:google_fonts/google_fonts.dart';
import 'package:provider/provider.dart';
import 'sine_model.dart';
export 'sine_model.dart';

class SineWidget extends StatefulWidget {
  const SineWidget({
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
  State<SineWidget> createState() => _SineWidgetState();
}

class _SineWidgetState extends State<SineWidget> {
  late SineModel _model;

  final scaffoldKey = GlobalKey<ScaffoldState>();

  @override
  void initState() {
    super.initState();
    _model = createModel(context, () => SineModel());

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
            },
          ),
          title: Text(
            'Sine Wave',
            style: FlutterFlowTheme.of(context).titleMedium.override(
                  fontFamily: 'Montserrat',
                  color: FlutterFlowTheme.of(context).primaryText,
                  fontSize: 30.0,
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
          child: Column(
            mainAxisSize: MainAxisSize.min,
            mainAxisAlignment: MainAxisAlignment.start,
            crossAxisAlignment: CrossAxisAlignment.center,
            children: [
              Padding(
                padding: EdgeInsetsDirectional.fromSTEB(24.0, 0.0, 24.0, 0.0),
                child: Text(
                  'What frequency and amplitude would you like the wave to have?',
                  textAlign: TextAlign.start,
                  style: FlutterFlowTheme.of(context).headlineLarge.override(
                        fontFamily: 'Montserrat',
                        letterSpacing: 0.0,
                      ),
                ),
              ),
              Padding(
                padding: EdgeInsetsDirectional.fromSTEB(24.0, 16.0, 24.0, 16.0),
                child: InkWell(
                  splashColor: Colors.transparent,
                  focusColor: Colors.transparent,
                  hoverColor: Colors.transparent,
                  highlightColor: Colors.transparent,
                  onTap: () async {
                    await actions.sendData(
                      BTDeviceStruct(
                        name: widget!.deviceName,
                        id: widget!.deviceId,
                        rssi: _model.currentRssi,
                      ),
                      'Lab 5 sine',
                    );
                  },
                  child: Material(
                    color: Colors.transparent,
                    elevation: 2.0,
                    shape: RoundedRectangleBorder(
                      borderRadius: BorderRadius.circular(12.0),
                    ),
                    child: Container(
                      width: double.infinity,
                      height: 75.0,
                      decoration: BoxDecoration(
                        color: FlutterFlowTheme.of(context).accent2,
                        borderRadius: BorderRadius.circular(12.0),
                      ),
                      child: Align(
                        alignment: AlignmentDirectional(0.0, 0.0),
                        child: Text(
                          'Frequency = 30Hz\nAmplitude = 2V\nPhase Shift = 50°',
                          style:
                              FlutterFlowTheme.of(context).titleLarge.override(
                                    fontFamily: 'Montserrat',
                                    fontSize: 17.0,
                                    letterSpacing: 0.0,
                                  ),
                        ),
                      ),
                    ),
                  ),
                ),
              ),
              Padding(
                padding: EdgeInsetsDirectional.fromSTEB(24.0, 16.0, 24.0, 16.0),
                child: InkWell(
                  splashColor: Colors.transparent,
                  focusColor: Colors.transparent,
                  hoverColor: Colors.transparent,
                  highlightColor: Colors.transparent,
                  onTap: () async {
                    await actions.sendData(
                      BTDeviceStruct(
                        name: widget!.deviceName,
                        id: widget!.deviceId,
                        rssi: _model.currentRssi,
                      ),
                      'Lab 6 sine',
                    );
                  },
                  child: Material(
                    color: Colors.transparent,
                    elevation: 2.0,
                    shape: RoundedRectangleBorder(
                      borderRadius: BorderRadius.circular(12.0),
                    ),
                    child: Container(
                      width: double.infinity,
                      height: 75.0,
                      decoration: BoxDecoration(
                        color: FlutterFlowTheme.of(context).accent2,
                        borderRadius: BorderRadius.circular(12.0),
                      ),
                      child: Align(
                        alignment: AlignmentDirectional(0.0, 0.0),
                        child: Text(
                          'Frequency = 1Hz\nAmplitude = 2V\nPhase Shift = 0°',
                          style:
                              FlutterFlowTheme.of(context).titleLarge.override(
                                    fontFamily: 'Montserrat',
                                    fontSize: 17.0,
                                    letterSpacing: 0.0,
                                  ),
                        ),
                      ),
                    ),
                  ),
                ),
              ),
              Padding(
                padding: EdgeInsetsDirectional.fromSTEB(24.0, 16.0, 24.0, 16.0),
                child: InkWell(
                  splashColor: Colors.transparent,
                  focusColor: Colors.transparent,
                  hoverColor: Colors.transparent,
                  highlightColor: Colors.transparent,
                  onTap: () async {
                    await actions.sendData(
                      BTDeviceStruct(
                        name: widget!.deviceName,
                        id: widget!.deviceId,
                        rssi: _model.currentRssi,
                      ),
                      'Lab 7 sine',
                    );
                  },
                  child: Material(
                    color: Colors.transparent,
                    elevation: 2.0,
                    shape: RoundedRectangleBorder(
                      borderRadius: BorderRadius.circular(12.0),
                    ),
                    child: Container(
                      width: double.infinity,
                      height: 75.0,
                      decoration: BoxDecoration(
                        color: FlutterFlowTheme.of(context).accent2,
                        borderRadius: BorderRadius.circular(12.0),
                      ),
                      child: Align(
                        alignment: AlignmentDirectional(0.0, 0.0),
                        child: Text(
                          'Frequency = 30Hz\nAmplitude = 0.5V\nPhase Shift = 0°',
                          style:
                              FlutterFlowTheme.of(context).titleLarge.override(
                                    fontFamily: 'Montserrat',
                                    fontSize: 17.0,
                                    letterSpacing: 0.0,
                                  ),
                        ),
                      ),
                    ),
                  ),
                ),
              ),
              Padding(
                padding: EdgeInsetsDirectional.fromSTEB(24.0, 22.0, 24.0, 12.0),
                child: Text(
                  'Stop the wave generation:',
                  style: FlutterFlowTheme.of(context).titleLarge.override(
                        fontFamily: 'Montserrat',
                        letterSpacing: 0.0,
                      ),
                ),
              ),
              Expanded(
                child: Padding(
                  padding: EdgeInsetsDirectional.fromSTEB(24.0, 0.0, 24.0, 0.0),
                  child: FFButtonWidget(
                    onPressed: () async {
                      await actions.sendData(
                        BTDeviceStruct(
                          name: widget!.deviceName,
                          id: widget!.deviceId,
                          rssi: widget!.deviceRssi,
                        ),
                        'Off',
                      );
                    },
                    text: 'Stop',
                    options: FFButtonOptions(
                      width: 300.0,
                      height: 200.0,
                      padding:
                          EdgeInsetsDirectional.fromSTEB(16.0, 0.0, 16.0, 0.0),
                      iconPadding:
                          EdgeInsetsDirectional.fromSTEB(0.0, 0.0, 0.0, 0.0),
                      color: FlutterFlowTheme.of(context).error,
                      textStyle:
                          FlutterFlowTheme.of(context).titleLarge.override(
                                fontFamily: 'Montserrat',
                                color: FlutterFlowTheme.of(context).primary,
                                letterSpacing: 0.0,
                              ),
                      elevation: 0.0,
                      borderRadius: BorderRadius.circular(8.0),
                    ),
                  ),
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }
}
