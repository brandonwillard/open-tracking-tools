package org.opentrackingtools.impl.graph.paths;

public class EdgePredictiveResults {

  final PathStateBelief beliefPrediction;
  final PathStateBelief locationPrediction;
  final double edgePredMarginalLogLik;
  final double edgePredTransLogLik;
  final double measurementPredLogLik;

  public EdgePredictiveResults(
    PathStateBelief beliefPrediction,
    PathStateBelief locationPrediction,
    double edgePredMarginalLogLik,
    double edgePredTransLogLik, double measurementPredLik) {
    this.beliefPrediction = locationPrediction;
    this.locationPrediction = locationPrediction;
    this.edgePredMarginalLogLik = edgePredMarginalLogLik;
    this.edgePredTransLogLik = edgePredTransLogLik;
    this.measurementPredLogLik = measurementPredLik;
  }

  public PathStateBelief getBeliefPrediction() {
    return beliefPrediction;
  }

  public double getEdgePredMarginalLogLik() {
    return edgePredMarginalLogLik;
  }

  public double getEdgePredTransLogLik() {
    return edgePredTransLogLik;
  }

  public PathStateBelief getLocationPrediction() {
    return locationPrediction;
  }

  public double getMeasurementPredLogLik() {
    return measurementPredLogLik;
  }

  public double getTotalLogLik() {
    return edgePredMarginalLogLik + edgePredTransLogLik
        + measurementPredLogLik;
  }

  @Override
  public String toString() {
    final StringBuilder builder = new StringBuilder();
    builder.append("EdgePredictiveResults")
        .append("[locationPrediction=")
        .append(locationPrediction)
        .append(", edgePredMarginalLogLik=")
        .append(edgePredMarginalLogLik)
        .append(", edgePredTransLogLik=")
        .append(edgePredTransLogLik)
        .append(", measurementPredLik=")
        .append(measurementPredLogLik).append("]");
    return builder.toString();
  }

}