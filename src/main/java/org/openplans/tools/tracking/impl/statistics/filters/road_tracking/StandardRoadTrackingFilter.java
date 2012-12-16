package org.openplans.tools.tracking.impl.statistics.filters.road_tracking;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.signals.LinearDynamicalSystem;

import java.util.Random;

import org.openplans.tools.tracking.impl.Observation;
import org.openplans.tools.tracking.impl.VehicleState;
import org.openplans.tools.tracking.impl.graph.paths.PathStateBelief;
import org.openplans.tools.tracking.impl.statistics.filters.AdjKalmanFilter;

public class StandardRoadTrackingFilter extends
    AbstractRoadTrackingFilter<StandardRoadTrackingFilter> {

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
  public StandardRoadTrackingFilter(Vector obsCov,
    Vector unscaledOffRoadStateCov, Vector unscaledOnRoadStateCov,
    int initialObsFreq) {

    this.obsCovar = MatrixFactory.getDefault()
        .createDiagonal(obsCov);

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

    this.setQr(MatrixFactory.getDefault().createDiagonal(
        unscaledOnRoadStateCov));
    final Matrix roadStateCov =
        createStateCovarianceMatrix(this.currentTimeDiff,
            this.getQr(), true);
    this.roadFilter =
        new AdjKalmanFilter(roadModel, roadStateCov,
            this.obsCovar);
    this.setOnRoadStateTransCovar(roadStateCov);

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

    this.setQg(MatrixFactory.getDefault().createDiagonal(
        unscaledOffRoadStateCov));
    final Matrix groundStateCov =
        createStateCovarianceMatrix(this.currentTimeDiff,
            this.getQg(), false);
    this.groundFilter =
        new AdjKalmanFilter(groundModel, groundStateCov,
            this.obsCovar);
    this.setOffRoadStateTransCovar(groundStateCov);

  }

  @Override
  public int compareTo(StandardRoadTrackingFilter o) {
    /*
     * In this case, all road tracking filters should be equal
     */
    return 0;
  }

  @Override
  public void update(VehicleState state, Observation obs, PathStateBelief posteriorState,
    PathStateBelief priorPredictiveState, Random rng) {
    /*
     * Not updating.
     */

  }

}
