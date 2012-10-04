package org.openplans.tools.tracking.impl.statistics.filters;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.signals.LinearDynamicalSystem;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import java.util.Random;

import org.openplans.tools.tracking.impl.Observation;
import org.openplans.tools.tracking.impl.VehicleState;
import org.openplans.tools.tracking.impl.graph.paths.InferredPath;

public class StandardRoadTrackingFilter extends
    AbstractRoadTrackingFilter {

  private static final long serialVersionUID = -7872007182966059657L;

  /**
   * Standard 2D tracking model with the following state equation: {@latex[ D_
   * x_t = G x_ t-1} + A \epsilon_t} Also, when angle != null, a constraint
   * matrix is created for the state covariance, with perpendicular variance
   * a0Variance. aVariance doubles as both the x and y variances for
   * free-motion.
   * 
   * @param gVariance
   * @param aVariance
   * @param a0Variance
   * @param angle
   */
  public StandardRoadTrackingFilter(Vector obsVariance,
    Vector offRoadStateVariance, Vector onRoadStateVariance, int initialObsFreq) {

    this.obsVariance =
        MatrixFactory.getDefault().createDiagonal(obsVariance);
    this.offRoadStateVariance =
        MatrixFactory.getDefault().createDiagonal(
            offRoadStateVariance);
    this.onRoadStateVariance =
        MatrixFactory.getDefault()
            .createDiagonal(onRoadStateVariance);
    
    this.currentTimeDiff = initialObsFreq;

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
    final Matrix roadStateCov = createStateCovarianceMatrix(
            this.currentTimeDiff, Qr, true);
    this.roadFilter =
        new AdjKalmanFilter(roadModel, roadStateCov, this.obsVariance);

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
    final Matrix groundStateCov = createStateCovarianceMatrix(
            this.currentTimeDiff, Qg, false);
    this.groundFilter =
        new AdjKalmanFilter(groundModel, groundStateCov, this.obsVariance);

  }

  @Override
  public void updateSufficientStatistics(Observation obs,
    VehicleState state, MultivariateGaussian sampledBelief,
    InferredPath sampledPath, Random rng) {
    /*
     * Nothing to update, since this model believes that the parameters in
     * this filter are constant.
     */

  }

}
