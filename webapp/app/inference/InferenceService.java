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

  /**
   * This constant determines whether to collect debug information 
   * for new inference instances created on the fly.
   */
  private static final boolean DEBUG_DEFAULT = false;

  public static void clearInferenceData() {
    vehicleToInstance.clear();
  }
  
  public static void processRecord(Observation observation) {

    final InferenceInstance ie = getOrCreateInferenceInstance(observation
        .getVehicleId(), false, DEBUG_DEFAULT);

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

  public static InferenceInstance getOrCreateInferenceInstance(String vehicleId, boolean isSimulation, boolean isDebug) {
    InferenceInstance ie = vehicleToInstance.get(vehicleId);

    if (ie == null) {
      ie = new InferenceInstance(vehicleId, isSimulation, isDebug);
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
