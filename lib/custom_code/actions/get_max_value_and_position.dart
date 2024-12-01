// Automatic FlutterFlow imports
import '/backend/schema/structs/index.dart';
import '/flutter_flow/flutter_flow_theme.dart';
import '/flutter_flow/flutter_flow_util.dart';
import 'index.dart'; // Imports other custom actions
import 'package:flutter/material.dart';
// Begin custom action code
// DO NOT REMOVE OR MODIFY THE CODE ABOVE!

// Custom action to find the maximum value and its position in a list of doubles.
List<dynamic> getMaxValueAndPosition(List<double> inputList) {
  if (inputList.isEmpty) {
    return [null, -1]; // Return null and -1 if the list is empty
  }

  double maxValue = inputList[0];
  int maxPosition = 0;

  for (int i = 1; i < inputList.length; i++) {
    if (inputList[i] > maxValue) {
      maxValue = inputList[i];
      maxPosition = i;
    }
  }

  return [maxValue, maxPosition];
}

// Set your action name, define your arguments and return parameter,
// and then add the boilerplate code using the green button on the right!
