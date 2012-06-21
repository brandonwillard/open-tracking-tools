package inference;

import java.util.Collection;
import java.util.List;
import java.util.Map;

import models.InferenceInstance;

import org.openplans.tools.tracking.impl.Observation;

import play.Logger;

import akka.actor.UntypedActor;
import akka.event.Logging;
import akka.event.LoggingAdapter;

import com.google.common.collect.LinkedHashMultimap;
import com.google.common.collect.Lists;
import com.google.common.collect.Maps;
import com.google.common.collect.Multimap;
import com.google.common.collect.Multimaps;

/**
 * This class is an Actor that responds to LocationRecord messages and processes.
 * Note: this is essentially a thread(instance)
 * 
 * @author bwillard
 * 
 */
public class InferenceService extends UntypedActor {

  private static final Map<String, InferenceInstance> vehicleToInstance = Maps
      .newConcurrentMap();

  public enum INFO_LEVEL {
    SINGLE_RESULT,
    ALL_RESULTS, 
    DEBUG
  }
  
  private static INFO_LEVEL currentLevel = INFO_LEVEL.ALL_RESULTS;

  public static void clearInferenceData() {
    vehicleToInstance.clear();
  }
  
  public static void processRecord(Observation observation) {

    final InferenceInstance ie = getOrCreateInferenceInstance(observation
        .getVehicleId(), false, currentLevel);

    ie.update(observation);
  }
  
  @Override
  public void onReceive(Object location) throws Exception {
    synchronized (this) {
      if (location instanceof Observation) {
        final Observation observation = (Observation) location;
        processRecord(observation);
        
        Logger.info("Message received:  "
            + observation.getTimestamp().toString());
      }
    }

  }
  
  public static InferenceInstance getInferenceInstance(String vehicleId) {
    InferenceInstance ie = vehicleToInstance.get(vehicleId);
    return ie;
  }

  public static InferenceInstance getOrCreateInferenceInstance(String vehicleId, boolean isSimulation, INFO_LEVEL currentLevel2) {
    InferenceInstance ie = vehicleToInstance.get(vehicleId);

    if (ie == null) {
      ie = new InferenceInstance(vehicleId, isSimulation, currentLevel);
      vehicleToInstance.put(vehicleId, ie);
    }

    return ie;
  }

  public static List<InferenceInstance> getInferenceInstances() {
    return Lists.newArrayList(vehicleToInstance.values());
  }

  public static void remove(String name) {
    vehicleToInstance.remove(name);
    Observation.remove(name);
  }

}
