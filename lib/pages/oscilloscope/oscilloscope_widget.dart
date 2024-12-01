import '/backend/schema/structs/index.dart';
import '/flutter_flow/flutter_flow_charts.dart';
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
import 'oscilloscope_model.dart';
export 'oscilloscope_model.dart';

class OscilloscopeWidget extends StatefulWidget {
  const OscilloscopeWidget({
    super.key,
    this.devicee,
    required this.deviceName,
    required this.deviceId,
    required this.deviceRssi,
    required this.hasWriteChracteristic,
  });

  final BTDeviceStruct? devicee;
  final String? deviceName;
  final String? deviceId;
  final int? deviceRssi;
  final bool? hasWriteChracteristic;

  @override
  State<OscilloscopeWidget> createState() => _OscilloscopeWidgetState();
}

class _OscilloscopeWidgetState extends State<OscilloscopeWidget> {
  late OscilloscopeModel _model;

  final scaffoldKey = GlobalKey<ScaffoldState>();

  @override
  void initState() {
    super.initState();
    _model = createModel(context, () => OscilloscopeModel());

    // On page load action.
    SchedulerBinding.instance.addPostFrameCallback((_) async {
      FFAppState().xaxis = [];
      FFAppState().yaxis = [];
      safeSetState(() {});
    });
  }

  @override
  void dispose() {
    _model.dispose();

    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    context.watch<FFAppState>();

    return GestureDetector(
      onTap: () => FocusScope.of(context).unfocus(),
      child: Scaffold(
        key: scaffoldKey,
        backgroundColor: FlutterFlowTheme.of(context).primaryBackground,
        appBar: AppBar(
          backgroundColor: FlutterFlowTheme.of(context).primary,
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
                'TimerOff',
              );
            },
          ),
          title: Text(
            'Oscilloscope',
            style: FlutterFlowTheme.of(context).titleMedium.override(
                  fontFamily: 'Montserrat',
                  color: FlutterFlowTheme.of(context).primaryText,
                  fontSize: 28.0,
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
            mainAxisSize: MainAxisSize.max,
            children: [
              Padding(
                padding: EdgeInsetsDirectional.fromSTEB(0.0, 5.0, 0.0, 0.0),
                child: Text(
                  'Press \"Start\" to begin. Press \"Stop\" to view measurements below. Double tap chart to zoom.',
                  textAlign: TextAlign.center,
                  style: FlutterFlowTheme.of(context).bodyMedium.override(
                        fontFamily: 'Montserrat',
                        letterSpacing: 0.0,
                      ),
                ),
              ),
              Align(
                alignment: AlignmentDirectional(0.0, -1.0),
                child: Padding(
                  padding:
                      EdgeInsetsDirectional.fromSTEB(24.0, 5.0, 24.0, 24.0),
                  child: Container(
                    width: 288.0,
                    height: 100.0,
                    decoration: BoxDecoration(
                      color: FlutterFlowTheme.of(context).secondaryBackground,
                      borderRadius: BorderRadius.circular(24.0),
                    ),
                    child: Row(
                      mainAxisSize: MainAxisSize.max,
                      children: [
                        Expanded(
                          child: Align(
                            alignment: AlignmentDirectional(0.0, 0.0),
                            child: Padding(
                              padding: EdgeInsets.all(24.0),
                              child: FFButtonWidget(
                                onPressed: () async {
                                  await actions.sendData(
                                    BTDeviceStruct(
                                      name: widget!.deviceName,
                                      id: widget!.deviceId,
                                      rssi: widget!.deviceRssi,
                                    ),
                                    'Oscilloscope',
                                  );
                                  _model.instantTimer = InstantTimer.periodic(
                                    duration: Duration(milliseconds: 250),
                                    callback: (timer) async {
                                      _model.dataaa = await actions.receiveData(
                                        widget!.devicee!,
                                      );
                                      _model.dataa = _model.dataaa;
                                      safeSetState(() {});
                                      _model.listt =
                                          await actions.receiveAndPlotData(
                                        _model.dataaa!,
                                      );
                                      FFAppState()
                                          .addToXaxis(_model.listt!.first);
                                      FFAppState()
                                          .addToYaxis(_model.listt!.last);
                                      safeSetState(() {});
                                    },
                                    startImmediately: true,
                                  );

                                  safeSetState(() {});
                                },
                                text: 'Start',
                                options: FFButtonOptions(
                                  height: 40.0,
                                  padding: EdgeInsetsDirectional.fromSTEB(
                                      16.0, 0.0, 16.0, 0.0),
                                  iconPadding: EdgeInsetsDirectional.fromSTEB(
                                      0.0, 0.0, 0.0, 0.0),
                                  color: FlutterFlowTheme.of(context).success,
                                  textStyle: FlutterFlowTheme.of(context)
                                      .titleLarge
                                      .override(
                                        fontFamily: 'Montserrat',
                                        color: FlutterFlowTheme.of(context)
                                            .primary,
                                        letterSpacing: 0.0,
                                      ),
                                  elevation: 0.0,
                                  borderRadius: BorderRadius.circular(8.0),
                                ),
                              ),
                            ),
                          ),
                        ),
                        Expanded(
                          child: Align(
                            alignment: AlignmentDirectional(0.0, 0.0),
                            child: Padding(
                              padding: EdgeInsets.all(24.0),
                              child: FFButtonWidget(
                                onPressed: () async {
                                  _model.instantTimer?.cancel();
                                  _model.amplitude =
                                      await actions.getMaxValueAndPosition(
                                    FFAppState().yaxis.toList(),
                                  );
                                  _model.amp = valueOrDefault<double>(
                                    _model.amplitude?.first,
                                    0.0,
                                  );
                                  safeSetState(() {});
                                  _model.time = valueOrDefault<double>(
                                    FFAppState().xaxis[_model.amplitude!.last],
                                    0.0,
                                  );
                                  safeSetState(() {});
                                  await actions.sendData(
                                    BTDeviceStruct(
                                      name: widget!.deviceName,
                                      id: widget!.deviceId,
                                      rssi: widget!.deviceRssi,
                                    ),
                                    'OscData',
                                  );
                                  _model.dat = await actions.receiveData(
                                    BTDeviceStruct(
                                      name: widget!.deviceName,
                                      id: widget!.deviceId,
                                      rssi: widget!.deviceRssi,
                                    ),
                                  );
                                  _model.dataa = _model.dat;
                                  safeSetState(() {});

                                  safeSetState(() {});
                                },
                                text: 'Stop',
                                options: FFButtonOptions(
                                  height: 40.0,
                                  padding: EdgeInsetsDirectional.fromSTEB(
                                      16.0, 0.0, 16.0, 0.0),
                                  iconPadding: EdgeInsetsDirectional.fromSTEB(
                                      0.0, 0.0, 0.0, 0.0),
                                  color: FlutterFlowTheme.of(context).error,
                                  textStyle: FlutterFlowTheme.of(context)
                                      .titleLarge
                                      .override(
                                        fontFamily: 'Montserrat',
                                        color: FlutterFlowTheme.of(context)
                                            .primary,
                                        letterSpacing: 0.0,
                                      ),
                                  elevation: 0.0,
                                  borderRadius: BorderRadius.circular(8.0),
                                ),
                              ),
                            ),
                          ),
                        ),
                      ],
                    ),
                  ),
                ),
              ),
              Expanded(
                child: Align(
                  alignment: AlignmentDirectional(0.0, 0.0),
                  child: Container(
                    width: 100.0,
                    height: 100.0,
                    constraints: BoxConstraints(
                      minWidth: double.infinity,
                      minHeight: double.infinity,
                      maxWidth: double.infinity,
                      maxHeight: double.infinity,
                    ),
                    decoration: BoxDecoration(
                      color: FlutterFlowTheme.of(context).primary,
                    ),
                    child: SingleChildScrollView(
                      scrollDirection: Axis.horizontal,
                      child: Row(
                        mainAxisSize: MainAxisSize.max,
                        children: [
                          Align(
                            alignment: AlignmentDirectional(0.0, 0.0),
                            child: InkWell(
                              splashColor: Colors.transparent,
                              focusColor: Colors.transparent,
                              hoverColor: Colors.transparent,
                              highlightColor: Colors.transparent,
                              onDoubleTap: () async {
                                _model.zoom = !_model.zoom;
                                safeSetState(() {});
                              },
                              child: Container(
                                width: valueOrDefault<double>(
                                  _model.zoom == true ? 70000.0 : 7000.0,
                                  7000.0,
                                ),
                                height: 397.0,
                                child: FlutterFlowLineChart(
                                  data: [
                                    FFLineChartData(
                                      xData: FFAppState().xaxis,
                                      yData: FFAppState().yaxis,
                                      settings: LineChartBarData(
                                        color: FlutterFlowTheme.of(context)
                                            .success,
                                        barWidth: 2.0,
                                        isCurved: true,
                                      ),
                                    )
                                  ],
                                  chartStylingInfo: ChartStylingInfo(
                                    backgroundColor:
                                        FlutterFlowTheme.of(context)
                                            .primaryText,
                                    showGrid: true,
                                    borderColor: FlutterFlowTheme.of(context)
                                        .secondaryText,
                                    borderWidth: 3.0,
                                  ),
                                  axisBounds: AxisBounds(
                                    minX: -0.0,
                                    minY: -5.0,
                                    maxX: 100.0,
                                    maxY: 5.0,
                                  ),
                                  xAxisLabelInfo: AxisLabelInfo(
                                    showLabels: true,
                                    labelTextStyle: TextStyle(),
                                    labelInterval: 10.0,
                                    reservedSize: 32.0,
                                  ),
                                  yAxisLabelInfo: AxisLabelInfo(
                                    showLabels: true,
                                    labelInterval: 5.0,
                                    reservedSize: 40.0,
                                  ),
                                ),
                              ),
                            ),
                          ),
                        ],
                      ),
                    ),
                  ),
                ),
              ),
              Align(
                alignment: AlignmentDirectional(0.0, 0.0),
                child: Padding(
                  padding: EdgeInsets.all(12.0),
                  child: Container(
                    width: double.infinity,
                    height: 100.0,
                    decoration: BoxDecoration(
                      color: FlutterFlowTheme.of(context).secondaryBackground,
                      borderRadius: BorderRadius.circular(0.0),
                      shape: BoxShape.rectangle,
                    ),
                    child: Stack(
                      children: [
                        Stack(
                          children: [
                            Align(
                              alignment: AlignmentDirectional(0.0, -1.0),
                              child: Padding(
                                padding: EdgeInsetsDirectional.fromSTEB(
                                    0.0, 5.0, 0.0, 0.0),
                                child: Text(
                                  'Measurements:',
                                  textAlign: TextAlign.center,
                                  style: FlutterFlowTheme.of(context)
                                      .titleMedium
                                      .override(
                                        fontFamily: 'Montserrat',
                                        color: FlutterFlowTheme.of(context)
                                            .primary,
                                        letterSpacing: 0.0,
                                      ),
                                ),
                              ),
                            ),
                            Align(
                              alignment: AlignmentDirectional(-1.0, 0.0),
                              child: Padding(
                                padding: EdgeInsetsDirectional.fromSTEB(
                                    5.0, 0.0, 0.0, 0.0),
                                child: Text(
                                  'Time:',
                                  style: FlutterFlowTheme.of(context)
                                      .headlineMedium
                                      .override(
                                        fontFamily: 'Montserrat',
                                        color: FlutterFlowTheme.of(context)
                                            .primary,
                                        fontSize: 22.0,
                                        letterSpacing: 0.0,
                                      ),
                                ),
                              ),
                            ),
                            Align(
                              alignment: AlignmentDirectional(-1.0, 1.0),
                              child: Padding(
                                padding: EdgeInsetsDirectional.fromSTEB(
                                    5.0, 0.0, 0.0, 5.0),
                                child: Text(
                                  'Amplitude:',
                                  style: FlutterFlowTheme.of(context)
                                      .headlineMedium
                                      .override(
                                        fontFamily: 'Montserrat',
                                        color: FlutterFlowTheme.of(context)
                                            .primary,
                                        fontSize: 22.0,
                                        letterSpacing: 0.0,
                                      ),
                                ),
                              ),
                            ),
                          ],
                        ),
                        Align(
                          alignment: AlignmentDirectional(-1.0, 1.0),
                          child: Padding(
                            padding: EdgeInsetsDirectional.fromSTEB(
                                135.0, 0.0, 0.0, 5.0),
                            child: Text(
                              valueOrDefault<String>(
                                _model.amp?.toString(),
                                '-',
                              ),
                              style: FlutterFlowTheme.of(context)
                                  .headlineMedium
                                  .override(
                                    fontFamily: 'Montserrat',
                                    color: FlutterFlowTheme.of(context).primary,
                                    fontSize: 22.0,
                                    letterSpacing: 0.0,
                                  ),
                            ),
                          ),
                        ),
                        Align(
                          alignment: AlignmentDirectional(-1.0, 0.0),
                          child: Padding(
                            padding: EdgeInsetsDirectional.fromSTEB(
                                70.0, 0.0, 0.0, 0.0),
                            child: Text(
                              valueOrDefault<String>(
                                _model.time?.toString(),
                                '-',
                              ),
                              textAlign: TextAlign.center,
                              style: FlutterFlowTheme.of(context)
                                  .headlineMedium
                                  .override(
                                    fontFamily: 'Montserrat',
                                    color: FlutterFlowTheme.of(context).primary,
                                    fontSize: 22.0,
                                    letterSpacing: 0.0,
                                  ),
                            ),
                          ),
                        ),
                        Align(
                          alignment: AlignmentDirectional(0.75, 0.72),
                          child: Padding(
                            padding: EdgeInsetsDirectional.fromSTEB(
                                0.0, 0.0, 5.0, 5.0),
                            child: Text(
                              valueOrDefault<String>(
                                _model.dataa,
                                '-',
                              ),
                              style: FlutterFlowTheme.of(context)
                                  .headlineMedium
                                  .override(
                                    fontFamily: 'Montserrat',
                                    color: FlutterFlowTheme.of(context).primary,
                                    fontSize: 22.0,
                                    letterSpacing: 0.0,
                                  ),
                            ),
                          ),
                        ),
                        Align(
                          alignment: AlignmentDirectional(0.99, -0.1),
                          child: Padding(
                            padding: EdgeInsetsDirectional.fromSTEB(
                                0.0, 0.0, 5.0, 0.0),
                            child: Text(
                              'Current Point:',
                              style: FlutterFlowTheme.of(context)
                                  .headlineMedium
                                  .override(
                                    fontFamily: 'Montserrat',
                                    color: FlutterFlowTheme.of(context).primary,
                                    fontSize: 22.0,
                                    letterSpacing: 0.0,
                                  ),
                            ),
                          ),
                        ),
                      ],
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
