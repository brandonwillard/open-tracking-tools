package models;

import gov.sandia.cognition.collection.ScalarMap;
import gov.sandia.cognition.math.MutableDouble;
import gov.sandia.cognition.math.RingAccumulator;
import gov.sandia.cognition.statistics.DataDistribution;
import inference.InferenceResultRecord;
import inference.InferenceService;
import inference.InferenceService.INFO_LEVEL;
import inference.ResultSet.OffRoadPath;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.net.URL;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Enumeration;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Queue;
import java.util.Random;
import java.util.concurrent.ConcurrentLinkedQueue;

import org.apache.log4j.Logger;
import org.opentrackingtools.GpsObservation;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.graph.edges.InferredEdge;
import org.opentrackingtools.graph.otp.impl.OtpGraph;
import org.opentrackingtools.graph.paths.edges.PathEdge;
import org.opentrackingtools.impl.VehicleState;
import org.opentrackingtools.impl.VehicleStateInitialParameters;
import org.opentrackingtools.impl.VehicleStatePerformanceResult;
import org.opentrackingtools.statistics.filters.vehicles.VehicleStateFilter;
import org.opentrackingtools.statistics.filters.vehicles.impl.FilterInformation;

import com.google.common.base.Stopwatch;
import com.google.common.collect.ComparisonChain;
import com.google.common.collect.ImmutableMap;
import com.google.common.collect.ImmutableMultimap;
import com.google.common.collect.Iterables;
import com.google.common.collect.Lists;
import com.google.common.collect.Maps;

import controllers.Api;
import controllers.Application;

/**
 * This class holds inference data for a particular vehicle
 * 
 * @author bwillard
 * 
 */
public class InferenceInstance implements Comparable<InferenceInstance> {

  final Logger log = Logger.getLogger(InferenceInstance.class);
  final public String vehicleId;

  public int recordsProcessed = 0;

  public long simSeed = 0l;

  public boolean isEnabled = true;

  private VehicleStateFilter<GpsObservation, VehicleState> filter;

  private final Queue<InferenceResultRecord> resultRecords =
      new ConcurrentLinkedQueue<InferenceResultRecord>();

  private DataDistribution<VehicleState> postBelief;
  private DataDistribution<VehicleState> resampleBelief;
  private VehicleState bestState;

  private final VehicleStateInitialParameters initialParameters;
  private final VehicleStateInitialParameters simulationParameters;

  public int totalRecords = 0;

  private final INFO_LEVEL infoLevel;
  private Map<VehicleState, List<Entry<Long, InferredEdge>>> statePaths;
  
  private static int _collectedPathLength = Integer.MAX_VALUE;
  private static final double _maxUpdateIntervalCutoff = 5d * 60d;

  private static InferenceGraph inferredGraph = Api.getGraph();

  private final RingAccumulator<MutableDouble> averager =
      new RingAccumulator<MutableDouble>();
  
  private final Class<? extends VehicleStateFilter> filterType;
  
  public InferenceInstance(String vehicleId, VehicleStateInitialParameters simParameters,
    INFO_LEVEL infoLevel, VehicleStateInitialParameters parameters) {
    this.initialParameters = parameters;
    this.simulationParameters = simParameters;
    this.vehicleId = vehicleId;
    this.simSeed = parameters.getSeed();
    this.infoLevel = infoLevel;
    this.filterType = Application.getFilters().get(parameters.getParticleFilterTypeName());
    
    // TODO FIXME: set collect_paths based on infoLevel?
    if (_collectedPathLength > 0) {
      statePaths = Maps.newHashMap();
    } else {
      statePaths = null;
    }
  }

  public RingAccumulator<MutableDouble> getAverager() {
    return averager;
  }

  public VehicleState getBestState() {
    return bestState;
  }

  public VehicleStateFilter getFilter() {
    return filter;
  }

  public INFO_LEVEL getInfoLevel() {
    return infoLevel;
  }

  public VehicleStateInitialParameters getInitialParameters() {
    return initialParameters;
  }

  public DataDistribution<VehicleState> getPostBelief() {
    return this.postBelief;
  }

  public int getRecordsProcessed() {
    return recordsProcessed;
  }

  public DataDistribution<VehicleState> getResampleBelief() {
    return this.resampleBelief;
  }

  public Collection<InferenceResultRecord> getResultRecords() {
    return Collections.unmodifiableCollection(this.resultRecords);
  }

  public long getSimSeed() {
    return simSeed;
  }

  public int getTotalRecords() {
    return totalRecords;
  }

  public String getVehicleId() {
    return vehicleId;
  }

  public boolean isSimulation() {
    return simulationParameters != null;
  }

  synchronized public void update(GpsObservation obs) {

    if (!shouldProcessUpdate(obs))
      return;
    
    updateFilter(obs);
    this.recordsProcessed++;

    final InferenceResultRecord infResult = InferenceResultRecord.createInferenceResultRecord(obs, this);

    if (infoLevel == INFO_LEVEL.SINGLE_RESULT
        && !this.resultRecords.isEmpty())
      this.resultRecords.poll();

    this.resultRecords.add(infResult);
  }

  synchronized public void update(VehicleState actualState, GpsObservation obs,
    boolean performInference, boolean updateOffRoad) {

    if (!shouldProcessUpdate(obs))
      return;
    
    if (performInference) {
        updateFilter(obs);
    }

    this.recordsProcessed++;

    final InferenceResultRecord result =
        InferenceResultRecord.createInferenceResultRecord(obs, this,
            actualState, postBelief.getMaxValueKey(), postBelief.clone(),
            resampleBelief != null ? resampleBelief.clone() : null,
                updateOffRoad);

    if (infoLevel == INFO_LEVEL.SINGLE_RESULT
        && !this.resultRecords.isEmpty())
      this.resultRecords.poll();

    this.resultRecords.add(result);
  }
  
  /**
   * We check basic conditions for processing the update and
   * also consider resetting the filter.
   * 
   */
  private boolean shouldProcessUpdate(GpsObservation obs) {
    if (filter != null) {
      final double timeDiff =
          filter.getLastProcessedTime() == null ? 1d
              : (obs.getTimestamp().getTime() - filter.getLastProcessedTime()) / 1000;
  
      if (timeDiff <= 0) {
        return false;
      } else if (timeDiff >= _maxUpdateIntervalCutoff) {
        /*
         * Note: we're not resetting the off-road paths, yet.
         */
        log.warn(" time diff (" + timeDiff + "s) is past update limit (" 
         + _maxUpdateIntervalCutoff + "s).  resetting filter...");
        postBelief = null;
        filter = null;
        return false;
      }
    }
    
    if (!isEnabled || InferenceService.getInferenceInstance(vehicleId) == null)
      return false;
    
    return true;
  }

  synchronized private void updateFilter(GpsObservation obs) {

    final Stopwatch watch = new Stopwatch();
    watch.start();

    if (filter == null || postBelief == null) {

      Constructor<? extends VehicleStateFilter> ctor;
      try {
        ctor = filterType.getConstructor(GpsObservation.class, InferenceGraph.class,
            VehicleStateInitialParameters.class, Boolean.class, Random.class);
        Random rng;
        if (initialParameters.getSeed() != 0)
          rng = new Random(initialParameters.getSeed());
        else
          rng = new Random();
          
        filter = ctor.newInstance(obs, inferredGraph, initialParameters,
          new Boolean(infoLevel.compareTo(INFO_LEVEL.DEBUG) >= 0), rng);
        filter.getRandom().setSeed(simSeed);
        postBelief = filter.createInitialLearnedObject();
      } catch (SecurityException e) {
        e.printStackTrace();
      } catch (NoSuchMethodException e) {
        e.printStackTrace();
      } catch (IllegalArgumentException e) {
        e.printStackTrace();
      } catch (InstantiationException e) {
        e.printStackTrace();
      } catch (IllegalAccessException e) {
        e.printStackTrace();
      } catch (InvocationTargetException e) {
        e.printStackTrace();
      }

    } else {
      filter.update(postBelief, obs);
      
      if (infoLevel.compareTo(INFO_LEVEL.DEBUG) >= 0) {
        final FilterInformation filterInfo =
            filter.getFilterInformation(obs);
        resampleBelief =
            filterInfo != null ? filterInfo.getResampleDist() : null;
      }
    }
    
    if (statePaths != null && !statePaths.isEmpty()) {
      Map<VehicleState, List<Entry<Long, InferredEdge>>> newPaths = Maps.newHashMap();
      for (VehicleState state : postBelief.getDomain()) {
        List<Entry<Long, InferredEdge>> path = Lists.newArrayList();
        
        List<Entry<Long, InferredEdge>> oldPath = statePaths.get(state.getParentState());
        if (oldPath != null)
            path.addAll(oldPath);
        /*
         * Make sure we don't add the start edge,
         * since we already have that as the old 
         * destination edge.
         */
        final List<? extends PathEdge> newPath = state.getBelief().getPath().getPathEdges();
        for (PathEdge edge : 
          (path.isEmpty() || (newPath.size() == 1 
            && Iterables.getOnlyElement(newPath).isNullEdge())
            || Iterables.getLast(path).getValue().isNullEdge()) 
            ? newPath : Iterables.skip(newPath, 1)) {
          path.add(Maps.immutableEntry(
              new Long(state.getObservation().getTimestamp().getTime()), 
              edge.getInferredEdge()));
        }
        
        if (_collectedPathLength < path.size())
          path = path.subList(0, _collectedPathLength);
        
        newPaths.put(state, path);
      }
      statePaths = newPaths;
    }

    watch.stop();
    averager.accumulate(new MutableDouble(watch.elapsedMillis()));

    if (recordsProcessed > 0 && recordsProcessed % 20 == 0)
      log.info("avg. records per sec = " + 1000d
          / this.getAverager().getMean().value);

    if (postBelief != null)
      this.bestState = postBelief.getMaxValueKey();
    
  }

  public static InferenceGraph getInferredGraph() {
    return inferredGraph;
  }

  public Class<? extends VehicleStateFilter> getFilterType() {
    return filterType;
  }

  private Map<VehicleState, List<OffRoadPath>> stateToOffRoadPaths = Maps.newHashMap();
  private VehicleStatePerformanceResult performanceResults;
  
  public Map<VehicleState, List<OffRoadPath>> getStateToOffRoadPaths() {
    return stateToOffRoadPaths;
  }

  public void setStateToOffRoadPaths(
    Map<VehicleState, List<OffRoadPath>> newMap) {
    this.stateToOffRoadPaths = newMap;
  }

  public VehicleStateInitialParameters getSimParameters() {
    return simulationParameters;
  }

  public VehicleStatePerformanceResult getPerformanceResults() {
    return this.performanceResults;
  }
  
  public void setPerformanceResults(VehicleStatePerformanceResult result) {
    this.performanceResults = result;
  }
  
  public List<Entry<Long, InferredEdge>> getStateCumulativePath(VehicleState state) {
    return statePaths.get(state);
  }

  public static int getCollectedPathLength() {
    return _collectedPathLength;
  }

  public static void setCollectedPathLength(int collectedPathLength) {
    InferenceInstance._collectedPathLength = collectedPathLength;
  }

  @Override
  public int compareTo(InferenceInstance o) {
    return ComparisonChain.start()
        .compare(this.vehicleId, o.vehicleId)
        .compare(this.initialParameters, o.initialParameters)
        .compare(this.simulationParameters, o.simulationParameters)
        .result();
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result =
        prime
            * result
            + ((initialParameters == null) ? 0
                : initialParameters.hashCode());
    result =
        prime
            * result
            + ((simulationParameters == null) ? 0
                : simulationParameters.hashCode());
    result =
        prime
            * result
            + ((vehicleId == null) ? 0 : vehicleId
                .hashCode());
    return result;
  }

  @Override
  public boolean equals(Object obj) {
    if (this == obj) {
      return true;
    }
    if (obj == null) {
      return false;
    }
    if (getClass() != obj.getClass()) {
      return false;
    }
    InferenceInstance other = (InferenceInstance) obj;
    if (initialParameters == null) {
      if (other.initialParameters != null) {
        return false;
      }
    } else if (!initialParameters
        .equals(other.initialParameters)) {
      return false;
    }
    if (simulationParameters == null) {
      if (other.simulationParameters != null) {
        return false;
      }
    } else if (!simulationParameters
        .equals(other.simulationParameters)) {
      return false;
    }
    if (vehicleId == null) {
      if (other.vehicleId != null) {
        return false;
      }
    } else if (!vehicleId.equals(other.vehicleId)) {
      return false;
    }
    return true;
  }

}
