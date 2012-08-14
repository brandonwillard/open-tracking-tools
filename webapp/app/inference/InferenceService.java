package inference;

import gov.sandia.cognition.math.matrix.VectorFactory;

import java.util.List;
import java.util.Map;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import models.InferenceInstance;

import org.openplans.tools.tracking.impl.Observation;
import org.openplans.tools.tracking.impl.VehicleState.VehicleStateInitialParameters;
import org.openplans.tools.tracking.impl.statistics.filters.AbstractVehicleTrackingFilter;
import org.openplans.tools.tracking.impl.statistics.filters.VehicleTrackingPLFilter;

import play.Logger;
import akka.actor.UntypedActor;

import com.google.common.collect.Iterables;
import com.google.common.collect.Lists;
import com.google.common.collect.Maps;

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

    final Observation obs;
    final InferenceInstance ie;

    UpdateRunnable(Observation obs, InferenceInstance ie) {
      super();
      this.obs = obs;
      this.ie = ie;
    }

    @Override
    public void run() {
      ie.update(obs);
    }

  }

  private static VehicleStateInitialParameters defaultVehicleStateInitialParams =
      new VehicleStateInitialParameters(VectorFactory.getDefault()
          .createVector2D(150d, 150d), VectorFactory.getDefault()
          .createVector2D(0.000625, 0.000625), VectorFactory
          .getDefault().createVector2D(0.000625, 0.000625),
          VectorFactory.getDefault().createVector2D(0.05d, 1d),
          VectorFactory.getDefault().createVector2D(1d, 0.05d), 
          VehicleTrackingPLFilter.class.getName(),
          50,
          0l);

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
      if (location instanceof Observation) {
        final Observation observation = (Observation) location;
        if (!processRecord(observation)) {

          new InferenceInstance(observation.getVehicleId(), false,
              defaultInfoLevel, defaultVehicleStateInitialParams, defaultFilterName);
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

  public static InferenceInstance getOrCreateInferenceInstance(
    String vehicleId,
    VehicleStateInitialParameters initialParameters,
    String filterTypeName,
    boolean isSimulation, INFO_LEVEL infoLevel) {

    InferenceInstance ie = vehicleToInstance.get(vehicleId);

    if (ie == null) {
      ie =
          new InferenceInstance(vehicleId, isSimulation, infoLevel,
              initialParameters, filterTypeName);
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
  public static boolean processRecord(Observation observation) {

    final InferenceInstance ie =
        getInferenceInstance(observation.getVehicleId());

    if (ie == null)
      return false;

    executor.execute(new UpdateRunnable(observation, ie));

    return true;
  }

  public static void
      processRecords(List<Observation> observations,
        VehicleStateInitialParameters initialParameters,
        String filterTypeName,
        INFO_LEVEL level) throws InterruptedException {

    final List<Callable<Object>> tasks = Lists.newArrayList();
    for (final Observation obs : observations) {
      final InferenceInstance ie =
          getOrCreateInferenceInstance(obs.getVehicleId(),
              initialParameters, filterTypeName, false, level);
      tasks.add(Executors.callable(new UpdateRunnable(obs, ie)));
    }

    executor.invokeAll(tasks);
  }

  public static void remove(String name) {
    vehicleToInstance.remove(name);
    Observation.remove(name);
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
