package inference;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.mtj.DenseMatrixFactoryMTJ;
import gov.sandia.cognition.math.matrix.mtj.decomposition.EigenDecompositionRightMTJ;
import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import inference.InferenceService.INFO_LEVEL;
import inference.ResultSet.InferenceResultSet;

import java.util.Date;
import java.util.List;

import models.InferenceInstance;

import org.codehaus.jackson.annotate.JsonIgnore;
import org.codehaus.jackson.map.annotate.JsonSerialize;
import org.openplans.tools.tracking.impl.LogDefaultDataDistribution;
import org.openplans.tools.tracking.impl.Observation;
import org.openplans.tools.tracking.impl.VehicleState;
import org.openplans.tools.tracking.impl.graph.paths.InferredPath;
import org.openplans.tools.tracking.impl.graph.paths.PathEdge;
import org.openplans.tools.tracking.impl.statistics.StandardRoadTrackingFilter;
import org.openplans.tools.tracking.impl.util.GeoUtils;

import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;

import controllers.Api;

public class InferenceResultRecord {

  private final String time;

  private final Coordinate observedCoords;

  private final ResultSet actualResults;

  private final InferenceResultSet infResults;

  private final DataDistribution<VehicleState> postDistribution;

  private final DataDistribution<VehicleState> resampleDistribution;

  public InferenceResultRecord(long time, Coordinate obsCoords,
    ResultSet actualResults, ResultSet infResults,
    DataDistribution<VehicleState> postDist,
    DataDistribution<VehicleState> priorDist) {
    this.actualResults = actualResults;
    final int count;
    if (postDist != null)
      count =
          ((LogDefaultDataDistribution<VehicleState>) postDist)
              .getCount(infResults.getState());
    else
      count = 1;

    this.infResults = new InferenceResultSet(infResults, count);
    this.observedCoords = obsCoords;
    this.time = Api.sdf.format(new Date(time));
    this.postDistribution = postDist;
    this.resampleDistribution = priorDist;
  }

  @JsonSerialize
  public ResultSet getActualResults() {
    return actualResults;
  }

  @JsonSerialize
  public ResultSet getInfResults() {
    return infResults;
  }

  @JsonSerialize
  public Coordinate getObservedCoords() {
    return observedCoords;
  }

  @JsonIgnore
  public DataDistribution<VehicleState> getPostDistribution() {
    return postDistribution;
  }

  @JsonIgnore
  public DataDistribution<VehicleState> getResampleDistribution() {
    return resampleDistribution;
  }

  @JsonSerialize
  public String getTime() {
    return time;
  }

  public static InferenceResultRecord createInferenceResultRecord(
    Observation observation, InferenceInstance inferenceInstance) {
    return createInferenceResultRecord(observation,
        inferenceInstance,
        null,
        inferenceInstance.getBestState(),
        inferenceInstance.getPostBelief().clone(),
        //        inferenceInstance.getInfoLevel().compareTo(INFO_LEVEL.DEBUG) >= 0 ? inferenceInstance
        //            .getPostBelief() : null,
        inferenceInstance.getInfoLevel().compareTo(INFO_LEVEL.DEBUG) >= 0
            ? inferenceInstance.getResampleBelief() : null);
  }

  public static InferenceResultRecord createInferenceResultRecord(
    Observation observation, InferenceInstance instance,
    VehicleState actualState, VehicleState inferredState,
    DataDistribution<VehicleState> postDist,
    DataDistribution<VehicleState> priorDist) {

    Preconditions.checkNotNull(observation);

    ResultSet actualResults = null;
    if (actualState != null) {
      actualResults =
          processVehicleStateResults(actualState, instance);
    }

    ResultSet infResults = null;
    if (inferredState != null) {
      infResults =
          processVehicleStateResults(inferredState, instance);
    }

    /*
     * XXX distributions are cloned, if given.
     */
    return new InferenceResultRecord(observation.getTimestamp()
        .getTime(), observation.getObsCoordsLatLon(), actualResults,
        infResults, postDist != null ? postDist.clone() : null,
        priorDist != null ? priorDist.clone() : null);

  }

  private static ResultSet processVehicleStateResults(
    VehicleState state, InferenceInstance instance) {

    /*
     * The last edge of the path should correspond to the current edge, and the
     * belief should be adjusted to the start of that edge.
     */

    /* 
     * TODO XXX why is it necessary to clone this state?
     * when it's a parent state, it can yield
     * a mean of dim 2 (on-road) and empty edge.
     * something is giving away a pointer to the belief
     * (or edge?  seems unlikely).
     */
    final VehicleState cloneState = state.clone();
    final PathEdge currentEdge =
        PathEdge.getEdge(cloneState.getInferredEdge(), 0d);
    final MultivariateGaussian gbelief =
        cloneState.getBelief().clone();
    final Matrix O =
        StandardRoadTrackingFilter.getGroundObservationMatrix();
    final Vector mean;
    final Vector minorAxis;
    final Vector majorAxis;

    StandardRoadTrackingFilter.convertToGroundBelief(gbelief,
        currentEdge);

    mean = O.times(gbelief.getMean().clone());

    if (currentEdge.isEmptyEdge()) {
      /*-
       * TODO only implemented for off-road
       * FIXME results look fishy
       */
      final EigenDecompositionRightMTJ decomp =
          EigenDecompositionRightMTJ
              .create(DenseMatrixFactoryMTJ.INSTANCE
                  .copyMatrix(gbelief.getCovariance()));

      final Matrix Shalf =
          MatrixFactory.getDefault().createIdentity(2, 2);
      double eigenValue1 = decomp.getEigenValue(0).getRealPart();
      double eigenValue2 = decomp.getEigenValue(1).getRealPart();
      eigenValue1 = Math.abs(eigenValue1) > 1e-10 ? eigenValue1 : 0d;
      eigenValue2 = Math.abs(eigenValue2) > 1e-10 ? eigenValue2 : 0d;
      Shalf.setElement(0, 0, Math.sqrt(eigenValue1));
      Shalf.setElement(1, 1, Math.sqrt(eigenValue2));
      majorAxis =
          mean.plus(O
              .times(decomp.getEigenVectorsRealPart().getColumn(0))
              .times(Shalf).scale(1.98));
      minorAxis =
          mean.plus(O
              .times(decomp.getEigenVectorsRealPart().getColumn(1))
              .times(Shalf).scale(1.98));
    } else {
      majorAxis = mean;
      minorAxis = mean;
    }

    final Coordinate meanCoords = GeoUtils.makeCoordinate(mean);
    final Coordinate majorAxisCoords =
        GeoUtils.makeCoordinate(majorAxis);
    final Coordinate minorAxisCoords =
        GeoUtils.makeCoordinate(minorAxis);

    final List<OsmSegmentWithVelocity> pathSegmentIds =
        Lists.newArrayList();
    Double pathDirection = null;
    final InferredPath path = cloneState.getPath();
    if (path.getTotalPathDistance() != null)
      pathDirection = path.getTotalPathDistance() > 0d ? 1d : -1d;
    for (final PathEdge edge : path.getEdges()) {
      if (edge.isEmptyEdge())
        continue;
      final double edgeMean =
          edge.getInferredEdge().getVelocityPrecisionDist()
              .getLocation();
      final int edgeId =
          edge.getInferredEdge().getEdgeId() != null ? edge
              .getInferredEdge().getEdgeId() : -1;
      final Geometry geom =
          edge.isEmptyEdge() ? null : edge.getInferredEdge()
              .getGeometry();
      final String name =
          edge.isEmptyEdge() ? "empty" : edge.getInferredEdge()
              .getEdge().getName();

      final OsmSegmentWithVelocity osmSegment =
          new OsmSegmentWithVelocity(edgeId, geom, name, edgeMean);

      pathSegmentIds.add(osmSegment);
    }

    return new ResultSet(cloneState, instance.getFilter(),
        meanCoords, majorAxisCoords, minorAxisCoords, pathSegmentIds,
        pathDirection);
  }

}
