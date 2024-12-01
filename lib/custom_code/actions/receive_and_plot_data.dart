// Automatic FlutterFlow imports
import '/backend/schema/structs/index.dart';
import '/flutter_flow/flutter_flow_theme.dart';
import '/flutter_flow/flutter_flow_util.dart';
import 'index.dart'; // Imports other custom actions
import 'package:flutter/material.dart';
// Begin custom action code
// DO NOT REMOVE OR MODIFY THE CODE ABOVE!

Future<List<double>?> receiveAndPlotData(String data) async {
  List<double> result = []; // Initialize an empty list to return

  try {
    debugPrint("Input data: '$data'"); // Print the input data for debugging

    // Expecting "x,y" format and parse it
    final xyValues = data.split(',');
    debugPrint("Split values: $xyValues"); // Debug the split result

    if (xyValues.length == 2) {
      final x = double.tryParse(xyValues[0].trim());
      final y = double.tryParse(xyValues[1].trim());

      debugPrint("Parsed values: x = $x, y = $y"); // Debug parsed values

      // Add valid x and y values to the result list
      if (x != null && y != null) {
        result.addAll([x, y]);
        debugPrint("Successfully added values to the result: $result");
      } else {
        debugPrint("Failed to parse x or y as double");
      }
    } else {
      debugPrint("Data format incorrect: Expected 'x,y', received: '$data'");
    }
  } catch (e) {
    debugPrint("Error in receiveAndPlotData: $e");
  }

  // Return the result list (could be empty if parsing failed)
  return result;
}
