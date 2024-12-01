// Automatic FlutterFlow imports
import '/backend/schema/structs/index.dart';
import '/flutter_flow/flutter_flow_theme.dart';
import '/flutter_flow/flutter_flow_util.dart';
import 'index.dart'; // Imports other custom actions
import 'package:flutter/material.dart';
// Begin custom action code
// DO NOT REMOVE OR MODIFY THE CODE ABOVE!

import 'dart:core'; // Import to handle List and String types

List<double> getXValues(List<String> dataList) {
  List<double> xValues = [];

  for (String data in dataList) {
    List<String> values = data.split(','); // Split the "x,y" string into [x, y]

    if (values.length == 2) {
      double x = double.tryParse(values[0]) ?? 0.0; // Parse x value
      xValues.add(x); // Add the x value to the xValues list
    }
  }

  return xValues; // Return the list of x values
}

// Set your action name, define your arguments and return parameter,
// and then add the boilerplate code using the green button on the right!
