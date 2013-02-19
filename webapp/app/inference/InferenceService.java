package inference;

import gov.sandia.cognition.math.matrix.VectorFactory;

import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import models.InferenceInstance;

import org.opentrackingtools.GpsObservation;
import org.opentrackingtools.impl.VehicleStateInitialParameters;
import org.opentrackingtools.statistics.estimators.vehicles.impl.StandardRoadTrackingEstimator;
import org.opentrackingtools.statistics.filters.vehicles.particle_learning.impl.VehicleStatePLFilter;

import play.Logger;
import play.mvc.Util;
import utils.ObservationFactory;
import akka.actor.UntypedActor;

import com.google.common.collect.HashMultimap;
import com.google.common.collect.Iterables;
import com.google.common.collect.Lists;
import com.google.common.collect.Maps;
import com.google.common.collect.Multimap;
import com.google.common.collect.TreeMultimap;

import controllers.Application;

/**
 * This class is an Actor that responds to LocationRecord messages and
 * processes. Note: this is essentially a thread(instance)
 * 
 * @author bwillard
 * 
 */
public class InferenceService extends UntypedActor {

  public enum INFO_LEVEL {
    SINGLE_RESULT, ALL_RESULTS, DEBUG
  }

  private static class UpdateRunnable implements Runnable {

    final List<GpsObservation> observations;
    final InferenceInstance ie;

    UpdateRunnable(List<GpsObservation> observations, InferenceInstance ie) {
      super();
      this.observations = observations;
      this.ie = ie;
    }

    @Override
    public void run() {
      for (GpsObservation obs : observations) 
        ie.update(obs);
    }

  }

  private static VehicleStateInitialParameters defaultVehicleStateInitialParams =
      new VehicleStateInitialParameters(
          VectorFactory.getDefault().createVector2D(100d, 100d), 20,
          VectorFactory.getDefault().createVector1D(0.000625), 20,
          VectorFactory.getDefault().createVector2D(0.000625, 0.000625), 20,
          VectorFactory.getDefault().createVector2D(5d, 95d),
          VectorFactory.getDefault().createVector2D(95d, 5d), 
          VehicleStatePLFilter.class.getName(),
          StandardRoadTrackingEstimator.class.getName(),
          25, 30, 0l);

  static public final int THREAD_COUNT;

  static {
    final int numProcessors =
        Runtime.getRuntime().availableProcessors();

    if (numProcessors <= 2) {
      THREAD_COUNT = numProcessors;
    } else {
      THREAD_COUNT = numProcessors - 1;
    }
  }

  private static final ExecutorService executor = Executors
      .newFixedThreadPool(THREAD_COUNT);

  private static final Map<String, InferenceInstance> vehicleToInstance =
      Maps.newConcurrentMap();

  public static final String defaultFilterName = 
      Iterables.getFirst(Application.getFilters().keySet(), null);

  public static INFO_LEVEL defaultInfoLevel = INFO_LEVEL.ALL_RESULTS;

  /**
   * See {@link #processRecord}
   */
  @Override
  public void onReceive(Object location) throws Exception {
    synchronized (this) {
      if (location instanceof GpsObservation) {
        final GpsObservation observation = (GpsObservation) location;
        if (!processRecord(observation)) {
          InferenceService.getOrCreateInferenceInstance(observation.getSourceId(), 
              defaultVehicleStateInitialParams, null, defaultInfoLevel);
        }

        Logger.info("Message received:  "
            + observation.getTimestamp().toString());
      }
    }

  }

  public static void clearInferenceData() {
    vehicleToInstance.clear();
  }

  public static INFO_LEVEL getDefaultInfoLevel() {
    return defaultInfoLevel;
  }

  public static VehicleStateInitialParameters
      getDefaultVehicleStateInitialParams() {
    return defaultVehicleStateInitialParams;
  }

  public static ExecutorService getExecutor() {
    return executor;
  }

  public static InferenceInstance getInferenceInstance(
    String vehicleId) {
    final InferenceInstance ie = vehicleToInstance.get(vehicleId);
    return ie;
  }

  public static List<InferenceInstance> getInferenceInstances() {
    return Lists.newArrayList(vehicleToInstance.values());
  }

  @Util
  public static InferenceInstance getOrCreateInferenceInstance(
    String vehicleId,
    VehicleStateInitialParameters initialParameters,
    VehicleStateInitialParameters simParameters, INFO_LEVEL infoLevel) {

    InferenceInstance ie = vehicleToInstance.get(vehicleId);

    if (ie == null) {
      ie =
          new InferenceInstance(vehicleId, simParameters, infoLevel,
              initialParameters);
      vehicleToInstance.put(vehicleId, ie);
    }

    return ie;
  }

  /**
   * This will process a record for an already existing
   * {@link #InferenceInstance}, or it will create a new one.
   * 
   * @param observation
   * @return boolean true if an instance was found and processed
   */
  public static boolean processRecord(GpsObservation observation) {

    final InferenceInstance ie =
        getInferenceInstance(observation.getSourceId());

    if (ie == null)
      return false;

    executor.execute(new UpdateRunnable(Collections.singletonList(observation), ie));

    return true;
  }

  public static void
      processRecords(List<GpsObservation> observations,
        VehicleStateInitialParameters initialParameters,
        String filterTypeName,
        INFO_LEVEL level) throws InterruptedException {

    final Multimap<InferenceInstance, GpsObservation> instanceToObs = TreeMultimap.create();
    
    for (final GpsObservation obs : observations) {
      final InferenceInstance ie =
          getOrCreateInferenceInstance(obs.getSourceId(),
              initialParameters, null, level);
      instanceToObs.put(ie, obs);
    }

    final List<Callable<Object>> tasks = Lists.newArrayList();
    for (InferenceInstance instance: instanceToObs.keySet()) {
      tasks.add(Executors.callable(new UpdateRunnable(
          Lists.newArrayList(instanceToObs.get(instance)), instance)));
    }
      
    executor.invokeAll(tasks);
  }

  public static void remove(String name) {
    vehicleToInstance.remove(name);
    ObservationFactory.remove(name);
  }

  public static void setDefaultInfoLevel(INFO_LEVEL defaultInfoLevel) {
    InferenceService.defaultInfoLevel = defaultInfoLevel;
  }

  public static void setDefaultVehicleStateInitialParams(
    VehicleStateInitialParameters defaultVehicleStateInitialParams) {
    InferenceService.defaultVehicleStateInitialParams =
        defaultVehicleStateInitialParams;
  }

}
