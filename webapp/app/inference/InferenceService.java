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

  private static final Multimap<String, InferenceResultRecord> vehicleToTraceResults = 
      Multimaps.synchronizedSetMultimap(LinkedHashMultimap.<String, InferenceResultRecord>create());

  public static void clearInferenceData() {
    vehicleToInstance.clear();
    vehicleToTraceResults.clear();
  }
  
  public static void processRecord(Observation observation) {

    final InferenceInstance ie = getInferenceInstance(observation
        .getVehicleId(), false);

    ie.update(observation);

    final InferenceResultRecord infResult = InferenceResultRecord
        .createInferenceResultRecord(observation, ie);

    vehicleToTraceResults.put(observation.getVehicleId(), infResult);

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

  public static InferenceInstance getInferenceInstance(String vehicleId, boolean isSimulation) {
    InferenceInstance ie = vehicleToInstance.get(vehicleId);

    if (ie == null) {
      ie = new InferenceInstance(vehicleId, isSimulation);
      vehicleToInstance.put(vehicleId, ie);
    }

    return ie;
  }

  public static Collection<InferenceResultRecord> getTraceResults(
    String vehicleId) {
    return vehicleToTraceResults.get(vehicleId);
  }

  public static void addSimulationRecords(String simulationName,
    InferenceResultRecord result) {
    InferenceInstance instance = getInferenceInstance(simulationName, true);
    vehicleToTraceResults.put(simulationName, result);
    instance.recordsProcessed += 1;
    vehicleToInstance.put(simulationName, instance);
  }

  public static List<InferenceInstance> getInferenceInstances() {
    return Lists.newArrayList(vehicleToInstance.values());
  }

  public static void remove(String name) {
    
    vehicleToInstance.remove(name);
    vehicleToTraceResults.removeAll(name);
    Observation.remove(name);
    
  }

}
