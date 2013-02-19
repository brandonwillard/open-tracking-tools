package org.opentrackingtools.graph.paths.edges.impl;

import gov.sandia.cognition.util.AbstractCloneableSerializable;
import gov.sandia.cognition.util.CloneableSerializable;

import org.apache.commons.lang3.builder.ToStringBuilder;
import org.opentrackingtools.statistics.distributions.PathStateDistribution;

public class EdgePredictiveResults extends AbstractCloneableSerializable {

  protected PathStateDistribution beliefPrediction;
  protected PathStateDistribution locationPrediction;
  protected double edgePredMarginalLogLik;
  protected double edgePredTransLogLik;
  protected double measurementPredLogLik;
  private Double total;

  public EdgePredictiveResults(
    PathStateDistribution beliefPrediction,
    PathStateDistribution locationPrediction,
    double edgePredMarginalLogLik,
    double edgePredTransLogLik, double measurementPredLik) {
    this.beliefPrediction = locationPrediction;
    this.locationPrediction = locationPrediction;
    this.edgePredMarginalLogLik = edgePredMarginalLogLik;
    this.edgePredTransLogLik = edgePredTransLogLik;
    this.measurementPredLogLik = measurementPredLik;
  }

  public PathStateDistribution getBeliefPrediction() {
    return beliefPrediction;
  }

  public double getEdgePredMarginalLogLik() {
    return edgePredMarginalLogLik;
  }

  public double getEdgePredTransLogLik() {
    return edgePredTransLogLik;
  }

  public PathStateDistribution getLocationPrediction() {
    return locationPrediction;
  }

  public double getMeasurementPredLogLik() {
    return measurementPredLogLik;
  }

  public double getTotalLogLik() {
    if (total == null) {
    total = edgePredMarginalLogLik 
        + edgePredTransLogLik
        + measurementPredLogLik;
    }
    return total;
  }

  @Override
  public String toString() {
    ToStringBuilder builder = new ToStringBuilder(this);
    builder.append("locationPrediction", locationPrediction);
    builder.append("total", total);
    builder.append("edgePredMarginalLogLik", edgePredMarginalLogLik);
    builder.append("edgePredTransLogLik", edgePredTransLogLik);
    builder.append("measurementPredLogLik", measurementPredLogLik);
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