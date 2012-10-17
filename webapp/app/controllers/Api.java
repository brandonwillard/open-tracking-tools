package controllers;

import gov.sandia.cognition.collection.ScalarMap.Entry;
import gov.sandia.cognition.learning.data.DefaultTargetEstimatePair;
import gov.sandia.cognition.learning.data.TargetEstimatePair;
import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.bayesian.BayesianCredibleInterval;
import gov.sandia.cognition.statistics.distribution.BernoulliDistribution;
import gov.sandia.cognition.statistics.distribution.BetaDistribution;
import gov.sandia.cognition.statistics.distribution.UnivariateGaussian;
import inference.InferenceResultRecord;
import inference.InferenceService;
import inference.OsmSegmentWithVelocity;

import java.awt.geom.Point2D;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Map;

import models.InferenceInstance;

import org.codehaus.jackson.JsonGenerationException;
import org.codehaus.jackson.map.JsonMappingException;
import org.codehaus.jackson.map.ObjectMapper;
import org.opengis.referencing.operation.MathTransform;
import org.openplans.tools.tracking.impl.Observation;
import org.openplans.tools.tracking.impl.VehicleState;
import org.openplans.tools.tracking.impl.VehicleStatePerformanceResult;
import org.openplans.tools.tracking.impl.VehicleTrackingPerformanceEvaluator;
import org.openplans.tools.tracking.impl.graph.InferredEdge;
import org.openplans.tools.tracking.impl.util.GeoUtils;
import org.openplans.tools.tracking.impl.util.OtpGraph;
import org.openplans.tools.tracking.impl.util.ProjectedCoordinate;
import org.opentripplanner.common.geometry.GeometryUtils;
import org.opentripplanner.routing.graph.Edge;

import play.Logger;
import play.Play;
import play.mvc.Controller;
import api.OsmSegment;

import com.google.common.collect.Iterables;
import com.google.common.collect.Lists;
import com.google.common.collect.Maps;
import com.vividsolutions.jts.geom.Coordinate;

public class Api extends Controller {

  public static final SimpleDateFormat sdf = new SimpleDateFormat(
      "yyyy-MM-dd hh:mm:ss");

  public static OtpGraph graph = new OtpGraph(
      Play.configuration.getProperty("application.otpGraphPath"), null);

  public static ObjectMapper jsonMapper = new ObjectMapper();

//  public static void convertToLatLon(String x, String y, String lat, String lon)
//      throws JsonGenerationException, JsonMappingException,
//      IOException {
//
//    final Coordinate rawCoords =
//        new Coordinate(Double.parseDouble(x), Double.parseDouble(y));
//    
//    final Coordinate refLatLon =
//        new Coordinate(Double.parseDouble(lat), Double.parseDouble(lon));
//    
//    final MathTransform transform = GeoUtils.getTransform(refLatLon);
//    final String epsgCode = "EPSG:" + GeoUtils.getEPSGCodefromUTS(refLatLon);
//    Coordinate coords = GeoUtils.convertToLatLon(transform, rawCoords);
//    Map<String, Object> jsonResults = Maps.newHashMap();
//    jsonResults.put("x", coords.x);
//    jsonResults.put("y", coords.y);
//    jsonResults.put("epsgCode", epsgCode);
//
//    renderJSON(jsonMapper.writeValueAsString(jsonResults));
//  }
  
  public static void convertToLatLon(String x, String y)
      throws JsonGenerationException, JsonMappingException,
      IOException {

    final Coordinate rawCoords =
        new Coordinate(Double.parseDouble(x), Double.parseDouble(y));
    
    final Coordinate refLatLon = GeoUtils.reverseCoordinates(graph.getTurnGraph().getExtent().centre());
    final MathTransform transform = GeoUtils.getTransform(refLatLon);
    final String epsgCode = "EPSG:" + GeoUtils.getEPSGCodefromUTS(refLatLon);
    Coordinate coords = GeoUtils.convertToLatLon(transform, rawCoords);
    Map<String, Object> jsonResults = Maps.newHashMap();
    jsonResults.put("x", coords.x);
    jsonResults.put("y", coords.y);
    jsonResults.put("epsgCode", epsgCode);

    renderJSON(jsonMapper.writeValueAsString(jsonResults));
  }

  public static void
      bestCumulativePath(String vehicleId)
          throws JsonGenerationException, JsonMappingException,
          IOException {
    final InferenceInstance instance =
        InferenceService.getInferenceInstance(vehicleId);
    if (instance == null)
      renderJSON(jsonMapper.writeValueAsString(null));

    List<Map<String, Object>> results = Lists.newArrayList();
    VehicleState state = instance.getBestState();
    List<java.util.Map.Entry<Long, InferredEdge>> edges = instance.getStateCumulativePath(state);
    
    for (java.util.Map.Entry<Long, InferredEdge> edgeEntry : edges) {
      Map<String, Object> jsonResult = Maps.newHashMap();
      jsonResult.put("time", edgeEntry.getKey());
      jsonResult.put("edge", new OsmSegment(edgeEntry.getValue()));
      results.add(jsonResult);
    }

    renderJSON(jsonMapper.writeValueAsString(results));
  }
  
  public static void
      evaluatedPaths(String vehicleId, int recordNumber)
          throws JsonGenerationException, JsonMappingException,
          IOException {
    final InferenceInstance instance =
        InferenceService.getInferenceInstance(vehicleId);
    if (instance == null)
      renderJSON(jsonMapper.writeValueAsString(null));

    final Collection<InferenceResultRecord> results =
        instance.getResultRecords();
    if (results.isEmpty())
      renderJSON(jsonMapper.writeValueAsString(null));

    final InferenceResultRecord tmpResult =
        Iterables.get(results, recordNumber, null);

    if (tmpResult == null)
      error(vehicleId + " result record " + recordNumber
          + " is out-of-bounds");

    renderJSON(jsonMapper.writeValueAsString(tmpResult
        .getInfResults().createEvaluatedPaths()));
  }
  
  public static void
      getOffRoadPaths(String vehicleId, int recordNumber)
          throws JsonGenerationException, JsonMappingException,
          IOException {
    final InferenceInstance instance =
        InferenceService.getInferenceInstance(vehicleId);
    if (instance == null)
      renderJSON(jsonMapper.writeValueAsString(null));

    final Collection<InferenceResultRecord> results =
        instance.getResultRecords();
    if (results.isEmpty())
      renderJSON(jsonMapper.writeValueAsString(null));

    final InferenceResultRecord tmpResult =
        Iterables.get(results, recordNumber, null);

    if (tmpResult == null)
      error(vehicleId + " result record " + recordNumber
          + " is out-of-bounds");

    renderJSON(jsonMapper.writeValueAsString(
        ((inference.ResultSet.InferenceResultSet)
            tmpResult.getInfResults()).getOffRoadPaths()));
  }


  public static OtpGraph getGraph() {
    return graph;
  }

  public static void getObservationsForEdge(Integer edgeId)
      throws JsonGenerationException, JsonMappingException,
      IOException {
    final List<Coordinate> observations = Lists.newArrayList();
    for (final InferenceInstance instance : InferenceService
        .getInferenceInstances()) {
      observations.addAll(getObservationsForEdgeInternal(
          instance.getVehicleId(), edgeId));
    }
    renderJSON(jsonMapper.writeValueAsString(observations));
  }

  public static void getObservationsForEdge(String vehicleId,
    Integer edgeId) throws JsonGenerationException,
      JsonMappingException, IOException {
    renderJSON(jsonMapper
        .writeValueAsString(getObservationsForEdgeInternal(vehicleId,
            edgeId)));
  }

  private static List<Coordinate> getObservationsForEdgeInternal(
    String vehicleId, Integer edgeId) {
    final InferenceInstance instance =
        InferenceService.getInferenceInstance(vehicleId);
    if (instance == null)
      return Collections.emptyList();

    final Collection<InferenceResultRecord> resultRecords =
        instance.getResultRecords();
    if (resultRecords.isEmpty())
      return Collections.emptyList();

    final List<Coordinate> observations = Lists.newArrayList();
    for (final InferenceResultRecord record : instance
        .getResultRecords()) {
      if (record.getPostDistribution() == null)
        continue;
      for (final VehicleState state : record.getPostDistribution()
          .getDomain()) {
        if (state.getInferredEdge().getEdgeId().equals(edgeId))
          observations.add(state.getObservation().getObsPoint());
      }
    }

    return observations;
  }
  
  public static void getEdgeTransitionStats(String vehicleId)
      throws JsonGenerationException, JsonMappingException,
      IOException {

    final InferenceInstance instance =
        InferenceService.getInferenceInstance(vehicleId);
    if (instance == null)
      renderJSON(jsonMapper.writeValueAsString(null));

    final Collection<InferenceResultRecord> resultRecords =
        instance.getResultRecords();
    if (resultRecords.isEmpty())
      renderJSON(jsonMapper.writeValueAsString(null));

    List<Map<String, Object>> results = Lists.newArrayList();
    for (final InferenceResultRecord record : resultRecords) {
      
      /*
       * Only take stats for the diagonals, since the others
       * are degenerate (i.e. 1 - diag).
       */
//      final UnivariateGaussian.SufficientStatistic edgeTransOnOnStat =
//          new UnivariateGaussian.SufficientStatistic();
//      final UnivariateGaussian.SufficientStatistic freeTransOnOnStat =
//          new UnivariateGaussian.SufficientStatistic();
//    
//      for (VehicleState state : record.getPostDistribution().getDomain()) {
//        edgeTransOnOnStat.update(state.getEdgeTransitionDist().getEdgeMotionTransPrior().getMean().getElement(0));
//        freeTransOnOnStat.update(state.getEdgeTransitionDist().getFreeMotionTransPrior().getMean().getElement(0));
//      }
      VehicleState state = record.getPostDistribution().getMaxValueKey();
      
      BayesianCredibleInterval onInterval = BayesianCredibleInterval.compute(
          new BetaDistribution(state.getEdgeTransitionDist()
              .getEdgeMotionTransProbPrior().getParameters().getElement(0),
              state.getEdgeTransitionDist()
              .getEdgeMotionTransProbPrior().getParameters().getElement(1))
          , 0.95);
      
      BayesianCredibleInterval offInterval = BayesianCredibleInterval.compute(
          new BetaDistribution(state.getEdgeTransitionDist()
              .getFreeMotionTransProbPrior().getParameters().getElement(0),
              state.getEdgeTransitionDist()
              .getFreeMotionTransProbPrior().getParameters().getElement(1))
          , 0.95);
      
      Map<String, Object> recResults = Maps.newHashMap();
      recResults.put("time", new Long(record.getActualResults().getState().getObservation().getTimestamp().getTime()));
      recResults.put("actualEdgeDiagonal", new Double(record.getActualResults().getState()
          .getEdgeTransitionDist().getEdgeMotionTransPrior().getMean().getElement(0)));
      recResults.put("actualFreeDiagonal", new Double(record.getActualResults().getState()
          .getEdgeTransitionDist().getFreeMotionTransPrior().getMean().getElement(0)));
      
      recResults.put("infEdgeDiagonalMean", new Double(onInterval.getCentralValue()));
      recResults.put("infEdgeDiagonalUpper", new Double(onInterval.getUpperBound()));
      recResults.put("infEdgeDiagonalLower", new Double(onInterval.getLowerBound()));
      
      recResults.put("infFreeDiagonalMean", new Double(offInterval.getCentralValue()));
      recResults.put("infFreeDiagonalUpper", new Double(offInterval.getUpperBound()));
      recResults.put("infFreeDiagonalLower", new Double(offInterval.getLowerBound()));
      
//      recResults.put("infEdgeDiagonalMean", new Double(edgeTransOnOnStat.getMean()));
//      recResults.put("infEdgeDiagonalVar", new Double(edgeTransOnOnStat.getVariance()));
//      recResults.put("infFreeDiagonalMean", new Double(freeTransOnOnStat.getMean()));
//      recResults.put("infFreeDiagonalVar", new Double(freeTransOnOnStat.getVariance()));
      
      results.add(recResults);
    }

    renderJSON(jsonMapper.writeValueAsString(results));
  }

  public static VehicleStatePerformanceResult getPerformanceResultsData(String vehicleId) {
    final InferenceInstance instance =
        InferenceService.getInferenceInstance(vehicleId);
    
    if (instance == null)
      return null;

    final Collection<InferenceResultRecord> resultRecords =
        instance.getResultRecords();
    if (resultRecords.isEmpty())
      return null;

    final VehicleTrackingPerformanceEvaluator evaluator =
        new VehicleTrackingPerformanceEvaluator();

    final List<TargetEstimatePair<VehicleState, DataDistribution<VehicleState>>> pairs =
        Lists.newArrayList();
    for (final InferenceResultRecord record : resultRecords) {
      pairs.add(new DefaultTargetEstimatePair(record
          .getActualResults().getState(), record
          .getPostDistribution()));
    }
    final VehicleStatePerformanceResult result =
        evaluator.evaluatePerformance(pairs);
    
    return result;
  }

  public static void getPerformanceResults(String vehicleId)
      throws JsonGenerationException, JsonMappingException,
      IOException {

    VehicleStatePerformanceResult result = getPerformanceResultsData(vehicleId);
    
    if (result == null)
      renderJSON(jsonMapper.writeValueAsString(null));
    
    renderJSON(jsonMapper.writeValueAsString(result));
  }

  public static void location(String vehicleId, String timestamp,
    String latStr, String lonStr, String velocity, String heading,
    String accuracy) {

    try {

      final Observation location =
          Observation.createObservation(vehicleId, timestamp, latStr,
              lonStr, velocity, heading, accuracy);

      if (location != null) {
        Application.locationActor.tell(location);
      }

      ok();
    } catch (final Exception e) {
      Logger.error(e.getMessage());
      badRequest();
    }
  }

  public static void particleDetails(String vehicleId,
    int recordNumber) throws JsonGenerationException,
      JsonMappingException, IOException {

    final InferenceInstance instance =
        InferenceService.getInferenceInstance(vehicleId);
    if (instance == null)
      renderJSON(jsonMapper.writeValueAsString(null));

    final Collection<InferenceResultRecord> results =
        instance.getResultRecords();
    if (results.isEmpty())
      renderJSON(jsonMapper.writeValueAsString(null));

    final InferenceResultRecord tmpResult =
        Iterables.get(results, recordNumber, null);

    if (tmpResult == null)
      error(vehicleId + " result record " + recordNumber
          + " is out-of-bounds");

    final List<Map<String, Object>> jsonResults =
        Lists.newArrayList();
    for (final Entry<VehicleState> stateEntry : tmpResult
        .getPostDistribution().entrySet()) {
      final Map<String, Object> thisMap = Maps.newHashMap();
      thisMap.put("weight", stateEntry.getValue());
      thisMap.put("edgeId", stateEntry.getKey().getInferredEdge()
          .getEdgeId());
      thisMap.put("meanLoc", GeoUtils.getCoordinates(stateEntry
          .getKey().getMeanLocation()));
      jsonResults.add(thisMap);
    }

    renderJSON(jsonMapper.writeValueAsString(jsonResults));
  }

  public static void segment(Integer segmentId)
      throws JsonGenerationException, JsonMappingException,
      IOException {

    if (segmentId == null)
      badRequest();

    final Edge e = graph.getBaseGraph().getEdgeById(segmentId);

    if (e != null) {

      final OsmSegment osmSegment =
          new OsmSegment(segmentId, e.getGeometry(), e.toString());

      renderJSON(jsonMapper.writeValueAsString(osmSegment));
    } else
      badRequest();
  }

  public static void traceParticleRecord(String vehicleId,
    int recordNumber, Integer particleNumber, Boolean withParent,
    Boolean isPrior) throws JsonGenerationException,
      JsonMappingException, IOException {

    final InferenceInstance instance =
        InferenceService.getInferenceInstance(vehicleId);
    if (instance == null)
      renderJSON(jsonMapper.writeValueAsString(null));

    final Collection<InferenceResultRecord> resultRecords =
        instance.getResultRecords();
    if (resultRecords.isEmpty())
      renderJSON(jsonMapper.writeValueAsString(null));

    final List<InferenceResultRecord> results = Lists.newArrayList();
    DataDistribution<VehicleState> belief = null;
    if (particleNumber == null) {
      /*
       * Just return the "best" state
       */

      final InferenceResultRecord result =
          Iterables.get(resultRecords, recordNumber, null);

      if (result == null)
        renderJSON(jsonMapper.writeValueAsString(null));

      results.add(result);

    } else {

      final InferenceResultRecord tmpResult =
          Iterables.get(resultRecords, recordNumber, null);

      if (tmpResult == null)
        error(vehicleId + " result record " + recordNumber
            + " is out-of-bounds");

      if (!isPrior)
        belief = tmpResult.getPostDistribution();
      else
        belief = tmpResult.getResampleDistribution();

      if (belief == null)
        renderJSON(jsonMapper.writeValueAsString(null));

      if (particleNumber < 0) {
        for (final VehicleState infState : belief.getDomain()) {

          final VehicleState actualState =
              tmpResult.getActualResults() != null ? tmpResult
                  .getActualResults().getState() : null;
          final InferenceResultRecord result =
              InferenceResultRecord.createInferenceResultRecord(
                  infState.getObservation(), instance, actualState,
                  infState, 
                  !isPrior ? belief : null, 
                  isPrior ? belief : null, false);
          results.add(result);
        }
      } else {

        final VehicleState infState =
            Iterables.get(belief.getDomain(), particleNumber, null);

        if (infState == null)
          renderJSON(jsonMapper.writeValueAsString(null));

        final VehicleState actualState =
            tmpResult.getActualResults() != null ? tmpResult
                .getActualResults().getState() : null;

        final InferenceResultRecord result =
            InferenceResultRecord.createInferenceResultRecord(
                infState.getObservation(), instance, actualState,
                infState, null, null, false);
        results.add(result);
      }
    }

    final List<Map<String, Object>> mapResults = Lists.newArrayList();
    for (final InferenceResultRecord result : results) {
      final VehicleState state = result.getInfResults().getState();
      final InferenceResultRecord parent;
      if (withParent == Boolean.TRUE) {
        final VehicleState parentState = state.getParentState();
        if (parentState != null) {
          parent =
              InferenceResultRecord.createInferenceResultRecord(
                  parentState.getObservation(), instance, null,
                  parentState, null, null, false);
        } else {
          parent = null;
        }
      } else {
        parent = null;
      }

      final Map<String, Object> mapResult = Maps.newHashMap();
      mapResult.put("particle", result);

      mapResult.put("isBest", state.equals(belief.getMaxValueKey()));
      if (belief != null) {
        final double weight = belief.getFraction(state);
        mapResult.put("weight", weight);
      }

      if (parent != null)
        mapResult.put("parent", parent);

      mapResults.add(mapResult);
    }
    
    /*
     * Sort by likelihood
     */
    Collections.sort(mapResults, new Comparator<Map<String, Object>>() {
      @Override
      public int compare(Map<String, Object> o1,
        Map<String, Object> o2) {
        final Double p1 = (Double) o1.get("weight");
        final Double p2 = (Double) o2.get("weight");
        return -p1.compareTo(p2);
      }
    });

    renderJSON(jsonMapper.writeValueAsString(mapResults));
  }

  public static void traces(String vehicleId)
      throws JsonGenerationException, JsonMappingException,
      IOException {

    final InferenceInstance instance =
        InferenceService.getInferenceInstance(vehicleId);
    if (instance == null)
      renderJSON(jsonMapper.writeValueAsString(null));

    final Collection<InferenceResultRecord> resultRecords =
        instance.getResultRecords();
    if (resultRecords.isEmpty())
      renderJSON(jsonMapper.writeValueAsString(null));

    renderJSON(jsonMapper.writeValueAsString(resultRecords));
  }

  public static void vertex() {
    Logger.info("vertices: " + graph.getVertexCount());

    // TODO noop

    ok();
  }

}