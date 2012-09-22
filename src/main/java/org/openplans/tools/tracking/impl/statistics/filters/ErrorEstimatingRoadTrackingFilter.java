package org.openplans.tools.tracking.impl.statistics.filters;

import java.util.Random;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.signals.LinearDynamicalSystem;
import gov.sandia.cognition.statistics.distribution.InverseWishartDistribution;

public class ErrorEstimatingRoadTrackingFilter extends
    AbstractRoadTrackingFilter {

  private InverseWishartDistribution obsVariancePrior;
  private InverseWishartDistribution onRoadStateVariancePrior;
  private InverseWishartDistribution offRoadStateVariancePrior;
  private Vector currentStateSample;
  
  public ErrorEstimatingRoadTrackingFilter(
    Vector obsVarPrior, final int obsVarDof,
    Vector offRoadStateVarPrior, final int offRoadVarDof, 
    Vector onRoadStateVariance, final int onRoadVarDof,
    Random rng) {
    
    this.obsVariancePrior = new InverseWishartDistribution(
        MatrixFactory.getDefault().createDiagonal(obsVariance)
        , obsVarDof);
    this.offRoadStateVariancePrior = new InverseWishartDistribution(
        MatrixFactory.getDefault().createDiagonal(
            offRoadStateVariance), offRoadVarDof);
    this.onRoadStateVariancePrior = new InverseWishartDistribution(
        MatrixFactory.getDefault()
            .createDiagonal(onRoadStateVariance), onRoadVarDof);
    
    if (rng != null) {
      this.obsVariance = obsVariancePrior.sample(rng);
      this.offRoadStateVariance = offRoadStateVariancePrior.sample(rng);
      this.onRoadStateVariance = onRoadStateVariancePrior.sample(rng);
    } else {
      this.obsVariance = obsVariancePrior.getMean();
      this.offRoadStateVariance = offRoadStateVariancePrior.getMean();
      this.onRoadStateVariance = onRoadStateVariancePrior.getMean();
    }

    /*
     * Create the road-coordinates filter
     */
    final LinearDynamicalSystem roadModel =
        new LinearDynamicalSystem(0, 2);
    final Matrix roadG =
        createStateTransitionMatrix(currentTimeDiff, true);
    roadModel.setA(roadG);
    roadModel.setB(MatrixFactory.getDefault().createIdentity(2, 2));
    roadModel.setC(Or);
    this.roadModel = roadModel;

    this.Qr =
        MatrixFactory.getDefault()
            .createDiagonal(onRoadStateVariance);
    this.roadFilter =
        new AdjKalmanFilter(roadModel, createStateCovarianceMatrix(
            1d, Qr, true), this.obsVariance);

    /*
     * Create the ground-coordinates filter
     */
    final LinearDynamicalSystem groundModel =
        new LinearDynamicalSystem(0, 4);

    final Matrix groundGct =
        createStateTransitionMatrix(currentTimeDiff, false);

    groundModel.setA(groundGct);
    groundModel.setB(MatrixFactory.getDefault().createIdentity(4, 4));
    groundModel.setC(Og);

    this.groundModel = groundModel;

    this.Qg =
        MatrixFactory.getDefault().createDiagonal(
            offRoadStateVariance);
    this.groundFilter =
        new AdjKalmanFilter(groundModel, createStateCovarianceMatrix(
            1d, Qg, false), this.obsVariance);
  }

  public InverseWishartDistribution getObsVariancePrior() {
    return obsVariancePrior;
  }

  public InverseWishartDistribution getOnRoadStateVariancePrior() {
    return onRoadStateVariancePrior;
  }

  public InverseWishartDistribution getOffRoadStateVariancePrior() {
    return offRoadStateVariancePrior;
  }

  public Vector getCurrentStateSample() {
    return currentStateSample;
  }

  public void setCurrentStateSample(Vector currentStateSample) {
    this.currentStateSample = currentStateSample;
  }

  public void setObsVariancePrior(
    InverseWishartDistribution obsVariancePrior) {
    this.obsVariancePrior = obsVariancePrior;
  }

  public void setOnRoadStateVariancePrior(
    InverseWishartDistribution onRoadStateVariancePrior) {
    this.onRoadStateVariancePrior = onRoadStateVariancePrior;
  }

  public void setOffRoadStateVariancePrior(
    InverseWishartDistribution offRoadStateVariancePrior) {
    this.offRoadStateVariancePrior = offRoadStateVariancePrior;
  }

}
