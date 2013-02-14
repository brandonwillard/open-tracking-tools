package org.opentrackingtools.graph.paths.edges.impl;

import gov.sandia.cognition.util.AbstractCloneableSerializable;
import gov.sandia.cognition.util.CloneableSerializable;

import org.opentrackingtools.graph.paths.states.PathStateBelief;

public class EdgePredictiveResults extends AbstractCloneableSerializable {

  protected PathStateBelief beliefPrediction;
  protected PathStateBelief locationPrediction;
  protected double edgePredMarginalLogLik;
  protected double edgePredTransLogLik;
  protected double measurementPredLogLik;

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

  @Override
  public EdgePredictiveResults clone() {
    EdgePredictiveResults clone = (EdgePredictiveResults) super.clone();
    clone.beliefPrediction = beliefPrediction.clone();
    clone.edgePredMarginalLogLik = edgePredMarginalLogLik;
    clone.edgePredTransLogLik = edgePredTransLogLik;
    clone.locationPrediction = locationPrediction.clone();
    clone.measurementPredLogLik = measurementPredLogLik;
    
    return clone;
  }

}