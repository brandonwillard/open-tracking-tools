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
import inference.ResultSet.OffRoadPath;

import java.util.Collections;
import java.util.Date;
import java.util.List;
import java.util.Map;

import models.InferenceInstance;

import org.codehaus.jackson.annotate.JsonIgnore;
import org.codehaus.jackson.map.annotate.JsonSerialize;
import org.openplans.tools.tracking.impl.Observation;
import org.openplans.tools.tracking.impl.VehicleState;
import org.openplans.tools.tracking.impl.graph.InferredEdge;
import org.openplans.tools.tracking.impl.graph.paths.InferredPath;
import org.openplans.tools.tracking.impl.graph.paths.PathEdge;
import org.openplans.tools.tracking.impl.statistics.DefaultCountedDataDistribution;
import org.openplans.tools.tracking.impl.statistics.filters.road_tracking.AbstractRoadTrackingFilter;
import org.openplans.tools.tracking.impl.util.GeoUtils;
import org.openplans.tools.tracking.impl.util.ProjectedCoordinate;
import org.opentripplanner.routing.graph.Edge;

import api.OsmSegment;

import com.google.common.base.Preconditions;
import com.google.common.collect.Iterables;
import com.google.common.collect.Lists;
import com.google.common.collect.Maps;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;

import controllers.Api;

public class InferenceResultRecord {

  private final String time;

  private final ProjectedCoordinate observedPoint;

  private final InferenceResultSet actualResults;

  private final InferenceResultSet infResults;

  private final DataDistribution<VehicleState> postDistribution;

  private final DataDistribution<VehicleState> resampleDistribution;

  public InferenceResultRecord(InferenceInstance instance, 
    long time, ProjectedCoordinate obsCoords,
    ResultSet actualResults, ResultSet infResults,
    DataDistribution<VehicleState> postDist,
    DataDistribution<VehicleState> priorDist) {
    this.actualResults = actualResults != null 
        ? new InferenceResultSet(actualResults, 0, Collections.<OffRoadPath>emptyList()) : null;
    final int count;
    if (postDist != null)
      count =
          ((DefaultCountedDataDistribution<VehicleState>) postDist)
              .getCount(infResults.getState());
    else
      count = 1;

    this.infResults = new InferenceResultSet(infResults, count, 
        instance.getStateToOffRoadPaths().get(infResults.state));
    this.observedPoint = obsCoords;
    this.time = Long.toString(time);//Api.sdf.format(new Date(time));
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
  public ProjectedCoordinate getObservedPoint() {
    return observedPoint;
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
        inferenceInstance.getInfoLevel().compareTo(INFO_LEVEL.DEBUG) >= 0
            ? inferenceInstance.getResampleBelief() : null);
  }

  public static InferenceResultRecord createInferenceResultRecord(
    Observation observation, InferenceInstance instance,
    VehicleState actualState, VehicleState inferredState,
    DataDistribution<VehicleState> postDist,
    DataDistribution<VehicleState> priorDist) {
    return createInferenceResultRecord(observation, instance, actualState, 
        inferredState, postDist, priorDist, true);
  }
  
  public static InferenceResultRecord createInferenceResultRecord(
    Observation observation, InferenceInstance instance,
    VehicleState actualState, VehicleState inferredState,
    DataDistribution<VehicleState> postDist,
    DataDistribution<VehicleState> priorDist, boolean updateOffRoad) {

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
      
      if (updateOffRoad)
        updateOffRoadPaths(postDist, instance);
    }

    /*
     * XXX distributions are cloned, if given.
     */
    return new InferenceResultRecord(instance, observation.getTimestamp()
        .getTime(), observation.getObsPoint(), actualResults,
        infResults, postDist != null ? postDist.clone() : null,
        priorDist != null ? priorDist.clone() : null);

  }

  private static void updateOffRoadPaths(
    DataDistribution<VehicleState> postDist, InferenceInstance instance) {
    
    /*
     * This map will be our new map; containing only the parent
     * particles of the next posterior set.
     */
    Map<VehicleState, List<OffRoadPath>> newMap = Maps.newHashMap();
    
    synchronized (instance) {
      
      // FIXME this shouldn't be true!
      if (postDist == null) {
        return;
      }
      
      for (VehicleState state : postDist.getDomain()) {
        
        /*
         * Update current off-road path, if applicable.
         * We always keep a list, though.
         */
        List<OffRoadPath> previousOffRoadPaths = instance.getStateToOffRoadPaths().get(state.getParentState()); 
        if (previousOffRoadPaths == null) {
          previousOffRoadPaths = Lists.newArrayList();
          newMap.put(state, previousOffRoadPaths);
        } else {
          /*
           * Make a copy for this particle.
           */
          previousOffRoadPaths = Lists.newArrayList(previousOffRoadPaths);
        }
        
        VehicleState parentState = state.getParentState();
        if (state.getBelief().getPath().isEmptyPath()) {
          if (parentState != null && parentState.getBelief().getPath().isEmptyPath()
              // FIXME below shouldn't be true!
              && !previousOffRoadPaths.isEmpty()) {
            /*
             * If this is the case, then we're continuing on an off-road
             * path, so just add the new mean location.
             */
            OffRoadPath previousPath = new OffRoadPath(Iterables.getLast(previousOffRoadPaths)); 
            previousPath.getPointsBetween().add(GeoUtils.makeCoordinate(
                state.getMeanLocation()));
            /*
             * Replace the last entry
             */
            previousOffRoadPaths.remove(previousOffRoadPaths.size() - 1);
            previousOffRoadPaths.add(previousPath);
            
          } else {
            /*
             * Newly started off-road path
             */
            List<Coordinate> coordList = Lists.newArrayList();
            coordList.add(GeoUtils.makeCoordinate(
                state.getMeanLocation()));
            OffRoadPath newPath = new OffRoadPath();
            newPath.setStartObs(state.getObservation());
            
            InferredEdge parentStateEdge = parentState != null ? parentState.getBelief().getEdge().getEdge() : null;
            newPath.setStartEdge(parentStateEdge != null ? new OsmSegment(parentStateEdge) : null);
            newPath.setPointsBetween(coordList);
            previousOffRoadPaths.add(newPath);
            
          }
          
        } else if (parentState != null && parentState.getBelief().getPath().isEmptyPath()){
          /*
           * We just finished our off-road path.
           * There should be a previousPath...
           */
          OffRoadPath previousPath = new OffRoadPath(Iterables.getLast(previousOffRoadPaths)); 
          previousPath.getPointsBetween().add(GeoUtils.makeCoordinate(
              state.getMeanLocation()));
          previousPath.setEndEdge(new OsmSegment(state.getBelief().getEdge().getEdge().getEdgeId(), 
                  state.getBelief().getEdge().getEdge().getGeometry(), 
                  state.getBelief().getEdge().getEdge().getEdge().getName()));
          previousPath.setEndObs(state.getObservation());
          /*
           * Replace the last entry
           */
          previousOffRoadPaths.remove(previousOffRoadPaths.size() - 1);
          previousOffRoadPaths.add(previousPath);
          
        }
        
        /*
         * Finally, update the new map.
         */
        newMap.put(state, previousOffRoadPaths);
      }
      
      instance.setStateToOffRoadPaths(newMap);
    
    }
    
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
    final Boolean isBackward = cloneState.getBelief().getPath().getIsBackward();
    final PathEdge currentEdge =
        PathEdge.getEdge(cloneState.getBelief().getEdge().getInferredEdge(), 0d, isBackward);
    final MultivariateGaussian gbelief =
        cloneState.getBelief().getStateBelief().clone();
    final Matrix O =
        AbstractRoadTrackingFilter.getGroundObservationMatrix();
    final Vector mean;
    final Vector minorAxis;
    final Vector majorAxis;

    if (!gbelief.getCovariance().isZero()) {
      AbstractRoadTrackingFilter.convertToGroundBelief(gbelief,
          currentEdge, true);
    } else {
      gbelief.setMean(AbstractRoadTrackingFilter.convertToGroundState(gbelief.getMean(),
          currentEdge, true));
    }
      
    mean = O.times(gbelief.getMean().clone());

    if (currentEdge.isEmptyEdge()
        && !gbelief.getCovariance().isZero()) {
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
    final InferredPath path = cloneState.getBelief().getPath();
    if (path.getTotalPathDistance() != null)
      pathDirection = path.getTotalPathDistance() > 0d ? 1d : -1d;
    for (final PathEdge edge : path.getEdges()) {
      if (edge.isEmptyEdge())
        continue;
      final double edgeMean =
          edge.getInferredEdge().getVelocityPrecisionDist()
              .getLocation();

      final OsmSegmentWithVelocity osmSegment =
          new OsmSegmentWithVelocity(edge.getInferredEdge(), edgeMean);

      pathSegmentIds.add(osmSegment);
    }

    return new ResultSet(cloneState, instance.getFilter(),
        meanCoords, majorAxisCoords, minorAxisCoords, pathSegmentIds,
        pathDirection);
  }

}
