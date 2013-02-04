package controllers;

import gov.sandia.cognition.collection.ScalarMap.Entry;
import gov.sandia.cognition.learning.data.DefaultTargetEstimatePair;
import gov.sandia.cognition.learning.data.TargetEstimatePair;
import gov.sandia.cognition.math.matrix.AbstractVector;
import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.mtj.DenseVector;
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
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Map;

import javax.annotation.Nullable;

import models.InferenceInstance;

import org.codehaus.jackson.JsonGenerationException;
import org.codehaus.jackson.map.JsonMappingException;
import org.codehaus.jackson.map.ObjectMapper;
import org.geotools.data.simple.SimpleFeatureSource;
import org.geotools.factory.FactoryRegistryException;
import org.geotools.feature.FeatureIterator;
import org.geotools.geometry.jts.JTSFactoryFinder;
import org.geotools.geometry.jts.ReferencedEnvelope;
import org.geotools.graph.util.geom.GeometryUtil;
import org.geotools.grid.Lines;
import org.geotools.grid.ortholine.LineOrientation;
import org.geotools.grid.ortholine.OrthoLineDef;
import org.geotools.referencing.CRS;
import org.opengis.feature.Feature;
import org.opengis.referencing.FactoryException;
import org.opengis.referencing.NoSuchAuthorityCodeException;
import org.opengis.referencing.crs.CoordinateReferenceSystem;
import org.opengis.referencing.operation.MathTransform;
import org.opengis.referencing.operation.NoninvertibleTransformException;
import org.opengis.referencing.operation.TransformException;
import org.opentrackingtools.GpsObservation;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.graph.edges.InferredEdge;
import org.opentrackingtools.graph.impl.GenericJTSGraph;
import org.opentrackingtools.graph.otp.impl.OtpGraph;
import org.opentrackingtools.graph.paths.impl.TrackingTestUtils;
import org.opentrackingtools.impl.VehicleState;
import org.opentrackingtools.impl.VehicleStatePerformanceResult;
import org.opentrackingtools.impl.VehicleTrackingPerformanceEvaluator;
import org.opentrackingtools.statistics.filters.vehicles.road.impl.AbstractRoadTrackingFilter;
import org.opentrackingtools.statistics.filters.vehicles.road.impl.ErrorEstimatingRoadTrackingFilter;
import org.opentrackingtools.statistics.impl.StatisticsUtil;
import org.opentrackingtools.util.GeoUtils;
import org.opentrackingtools.util.geom.ProjectedCoordinate;
import org.opentripplanner.common.geometry.GeometryUtils;
import org.opentripplanner.routing.graph.Edge;

import play.Logger;
import play.Play;
import play.mvc.Controller;
import utils.ObservationFactory;
import api.OsmSegment;

import com.google.common.base.Predicate;
import com.google.common.collect.Iterables;
import com.google.common.collect.Lists;
import com.google.common.collect.Maps;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Envelope;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.geom.MultiLineString;
import com.vividsolutions.jts.index.strtree.STRtree;
import com.vividsolutions.jts.operation.linemerge.LineMerger;

public class Api extends Controller {

  public static final SimpleDateFormat sdf = new SimpleDateFormat(
      "yyyy-MM-dd hh:mm:ss");

  public static InferenceGraph graph;
//    = new OtpGraph(
//        Play.configuration.getProperty("application.graphPath"), null);
  
  static {
      
    try {
      graph = new GenericJTSGraph(TrackingTestUtils.createGridGraph(
          new Coordinate(40.7549, -73.97749)));
    } catch (NoSuchAuthorityCodeException e) {
      e.printStackTrace();
    } catch (FactoryRegistryException e) {
      e.printStackTrace();
    } catch (IOException e) {
      e.printStackTrace();
    } catch (FactoryException e) {
      e.printStackTrace();
    }
  }
  
  public static ObjectMapper jsonMapper = new ObjectMapper();
  
  public static void getGraphCenter()
      throws JsonGenerationException, JsonMappingException,
      IOException {
    Coordinate center = graph.getGPSGraphExtent().centre();
    Map<String, Object> result = Maps.newHashMap();
    result.put("lat", center.x);
    result.put("lng", center.y);
    result.put("epsgCode", GeoUtils.getEPSGCodefromUTS(center));
    renderJSON(jsonMapper.writeValueAsString(result));
  }
  
  public static void convertToLatLon(String x, String y)
      throws JsonGenerationException, JsonMappingException,
      IOException, NoninvertibleTransformException, TransformException {

    final Coordinate rawCoords =
        new Coordinate(Double.parseDouble(x), Double.parseDouble(y));
    
    final Coordinate refLatLon = graph.getGPSGraphExtent().centre();
    final MathTransform transform = GeoUtils.getTransform(refLatLon);
    final String epsgCode = "EPSG:" + GeoUtils.getEPSGCodefromUTS(refLatLon);
    Coordinate coords = GeoUtils.convertToLatLon(transform, rawCoords);
    Map<String, Object> jsonResults = Maps.newHashMap();
    jsonResults.put("lat", coords.x);
    jsonResults.put("lng", coords.y);
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


  public static InferenceGraph getGraph() {
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
        if (state.getBelief().getEdge().getInferredEdge().getEdgeId().equals(edgeId))
          observations.add(state.getObservation().getObsProjected());
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

      final GpsObservation location =
          ObservationFactory.createObservation(vehicleId, timestamp, latStr,
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
      thisMap.put("edgeId", stateEntry.getKey().getBelief().getEdge().getInferredEdge()
          .getEdgeId());
      thisMap.put("meanLoc", GeoUtils.getCoordinates(stateEntry
          .getKey().getMeanLocation()));
      jsonResults.add(thisMap);
    }

    renderJSON(jsonMapper.writeValueAsString(jsonResults));
  }

  public static void segment(String segmentId)
      throws JsonGenerationException, JsonMappingException,
      IOException {

    if (segmentId == null)
      badRequest();

//    final Edge e = graph.getBaseGraph().getEdgeById(
//        Integer.parseInt(segmentId));
    
    final InferredEdge e = graph.getInferredEdge(segmentId);

    if (e != null) {

      final OsmSegment osmSegment =
          new OsmSegment(segmentId, e.getGeometry(), e.toString());

      renderJSON(jsonMapper.writeValueAsString(osmSegment));
    } else
      badRequest();
  }
  
  public static void particleHistoryDebug(String vehicleId,
    int recordNumber, Integer particleNumber) throws JsonGenerationException,
      JsonMappingException, IOException {
    
    final InferenceInstance instance =
        InferenceService.getInferenceInstance(vehicleId);
    if (instance == null)
      renderJSON(jsonMapper.writeValueAsString(null));

    final Collection<InferenceResultRecord> resultRecords =
        instance.getResultRecords();
    if (resultRecords.isEmpty())
      renderJSON(jsonMapper.writeValueAsString(null));

    
    final InferenceResultRecord record =
        Iterables.get(resultRecords, recordNumber, null);

    if (record == null)
      error(vehicleId + " result record " + recordNumber
          + " is out-of-bounds");

    DataDistribution<VehicleState> belief = record.getPostDistribution();
    
    VehicleState state =
        Iterables.get(belief.getDomain(), particleNumber, null);

    if (state == null)
      renderJSON(jsonMapper.writeValueAsString(null));

    final List<Map<String, Object>> mapResults = Lists.newArrayList();
    int currentRecordNumber = recordNumber;
    while (state != null) {
      final Map<String, Object> mapResult = Maps.newHashMap();
      mapResult.put("time", state.getObservation().getTimestamp().getTime());
      mapResult.put("recordNumber", currentRecordNumber);
      
      final int thisParticleNum;
      if (currentRecordNumber != recordNumber) {
        final InferenceResultRecord thisRecord =
            Iterables.get(resultRecords, currentRecordNumber, null);
        final VehicleState thisState = state;
        thisParticleNum = Iterables.indexOf(thisRecord.getPostDistribution().getDomain(), 
            new Predicate<VehicleState>() { 
            @Override
            public boolean
                apply(@Nullable VehicleState input) {
              return input.equals(thisState);
            }
        });
      } else {
        thisParticleNum = particleNumber;
      }
      mapResult.put("particleNumber", thisParticleNum);
      
      ErrorEstimatingRoadTrackingFilter filter = (ErrorEstimatingRoadTrackingFilter) state.getMovementFilter();  
      
      mapResult.put("obsCovMean",((DenseVector) 
          filter.getObsVariancePrior().getMean().convertToVector()).getArray());
      mapResult.put("onRoadCovMean", ((DenseVector) 
          filter.getOnRoadStateVariancePrior().getMean().convertToVector()).getArray());
      mapResult.put("offRoadCovMean", ((DenseVector) 
          filter.getOffRoadStateVariancePrior().getMean().convertToVector()).getArray());
      if (filter.getCurrentStateSample() != null) {
        final Matrix projMatrix = filter.getPrevStateSample().isOnRoad() ?
           filter.getRoadModel().getA() : filter.getGroundModel().getA();
        mapResult.put("stateSampleDiff", ((DenseVector) 
            filter.getCurrentStateSample().getGlobalState().minus(
                projMatrix.times(
                    filter.getPrevStateSample().getGlobalState()))).getArray());
        mapResult.put("stateSampleObsDiff", ((DenseVector) 
            state.getObservation().getProjectedPoint().minus(
                AbstractRoadTrackingFilter.getOg().times(
                    filter.getCurrentStateSample().getGroundState()))).getArray());
      }
      mapResults.add(mapResult);
      
      currentRecordNumber--;
      state = state.getParentState();
    }
    
    renderJSON(jsonMapper.writeValueAsString(mapResults));
    
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

    /*
     * Get best particles for every observation
     */
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
      
      final int thisParticleNumber = Lists.newArrayList(belief.getDomain()).indexOf(state);
      mapResult.put("particleNumber", thisParticleNumber);

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

}